"""YOLO segmentation + depth: average distance to each person.

Subscribes to a color image and the aligned depth image, runs YOLO instance
segmentation (person class only), averages depth over each mask and publishes
a PersonDistanceArray for other nodes to consume. Optionally publishes an
annotated debug image.
"""

import math
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import CameraInfo, Image
from person_distance_msgs.msg import PersonDistance, PersonDistanceArray

PERSON_CLASS_ID = 0  # COCO class 0 = person


class PersonDistanceNode(Node):

    def __init__(self):
        super().__init__('person_distance')

        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera/color/camera_info')
        self.declare_parameter('model', 'yolo11n-seg.pt')
        self.declare_parameter('device', '')  # '' = auto (cuda if available)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('max_distance', 10.0)  # meters; depth beyond this is ignored
        self.declare_parameter('publish_debug_image', True)

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.info_topic = self.get_parameter('camera_info_topic').value
        self.conf_threshold = float(self.get_parameter('confidence_threshold').value)
        self.max_distance = float(self.get_parameter('max_distance').value)
        self.publish_debug = bool(self.get_parameter('publish_debug_image').value)

        self.bridge = CvBridge()
        self.intrinsics = None  # (fx, fy, cx, cy)

        # Latest synchronized frame pair; the inference loop consumes it so the
        # subscription callback never blocks on the GPU.
        self._frame_lock = threading.Lock()
        self._latest_frames = None
        self._frame_event = threading.Event()

        self.pub_persons = self.create_publisher(PersonDistanceArray, 'person_distances', 10)
        self.pub_debug = (
            self.create_publisher(Image, 'person_distances/debug_image', 1)
            if self.publish_debug else None
        )

        self.sub_info = self.create_subscription(
            CameraInfo, self.info_topic, self._info_callback, qos_profile_sensor_data)
        self.sub_color = Subscriber(self, Image, self.color_topic,
                                    qos_profile=qos_profile_sensor_data)
        self.sub_depth = Subscriber(self, Image, self.depth_topic,
                                    qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer(
            [self.sub_color, self.sub_depth], queue_size=5, slop=0.1)
        self.sync.registerCallback(self._frames_callback)

        self.model = None
        self._stop = False
        self._worker = threading.Thread(target=self._inference_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f'Waiting for frames on {self.color_topic} + {self.depth_topic}')

    def _info_callback(self, msg: CameraInfo):
        if self.intrinsics is None:
            k = msg.k
            self.intrinsics = (k[0], k[4], k[2], k[5])  # fx, fy, cx, cy
            self.get_logger().info(
                f'Camera intrinsics: fx={k[0]:.1f} fy={k[4]:.1f} cx={k[2]:.1f} cy={k[5]:.1f}')

    def _frames_callback(self, color_msg: Image, depth_msg: Image):
        with self._frame_lock:
            self._latest_frames = (color_msg, depth_msg)
        self._frame_event.set()

    def _load_model(self):
        from ultralytics import YOLO
        import torch

        model_path = self.get_parameter('model').value
        device = self.get_parameter('device').value
        if not device:
            device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Loading {model_path} on {device} ...')
        model = YOLO(model_path)
        if model_path.endswith('.pt'):
            model.to(device)  # exported models (.engine/.onnx) are device-bound already
        # Warm up so the first real frame is not slow
        model.predict(np.zeros((480, 640, 3), dtype=np.uint8), verbose=False)
        self.get_logger().info('Model ready')
        return model

    def _inference_loop(self):
        try:
            self.model = self._load_model()
        except Exception as e:  # noqa: BLE001 - surface any load failure and exit
            self.get_logger().fatal(f'Failed to load YOLO model: {e}')
            rclpy.try_shutdown()
            return

        while not self._stop and rclpy.ok():
            if not self._frame_event.wait(timeout=0.5):
                continue
            self._frame_event.clear()
            with self._frame_lock:
                frames = self._latest_frames
                self._latest_frames = None
            if frames is None:
                continue
            try:
                self._process(*frames)
            except Exception as e:  # noqa: BLE001 - keep the loop alive
                self.get_logger().error(f'Processing failed: {e}')

    def _process(self, color_msg: Image, depth_msg: Image):
        color = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
        depth = self.bridge.imgmsg_to_cv2(depth_msg)
        if depth.dtype == np.uint16:
            depth_m = depth.astype(np.float32) * 1e-3
        else:
            depth_m = depth.astype(np.float32)

        # retina_masks=True returns masks at the original image resolution,
        # so they overlay the aligned depth image directly.
        results = self.model.predict(
            color, classes=[PERSON_CLASS_ID], conf=self.conf_threshold,
            retina_masks=True, verbose=False)
        result = results[0]

        out = PersonDistanceArray()
        out.header = color_msg.header

        if result.masks is not None and len(result.masks.data) > 0:
            masks = result.masks.data.cpu().numpy()  # (N, H, W), float 0/1
            confs = result.boxes.conf.cpu().numpy()
            for i, (mask, conf) in enumerate(zip(masks, confs)):
                if mask.shape != depth_m.shape:
                    continue  # depth not aligned to color resolution
                person = self._measure(i, mask > 0.5, float(conf), depth_m)
                if person is not None:
                    out.persons.append(person)

        self.pub_persons.publish(out)

        if self.pub_debug is not None and self.pub_debug.get_subscription_count() > 0:
            self._publish_debug(result, out, color_msg.header)

    def _measure(self, idx, mask, conf, depth_m):
        d = depth_m[mask]
        d = d[(d > 0.1) & (d < self.max_distance)]
        person = PersonDistance()
        person.id = idx
        person.confidence = conf
        person.valid_depth_pixels = int(d.size)

        ys, xs = np.nonzero(mask)
        if xs.size == 0:
            return None
        u, v = float(xs.mean()), float(ys.mean())
        person.pixel_x = int(u)
        person.pixel_y = int(v)

        if d.size < 20:
            person.distance = math.nan
            return person

        # Trim tails so background bleeding at the mask border does not skew the mean
        lo, hi = np.percentile(d, [5, 95])
        person.distance = float(d[(d >= lo) & (d <= hi)].mean())

        if self.intrinsics is not None:
            fx, fy, cx, cy = self.intrinsics
            z = person.distance
            person.position.x = (u - cx) * z / fx
            person.position.y = (v - cy) * z / fy
            person.position.z = z
        return person

    def _publish_debug(self, result, out, header):
        import cv2

        annotated = result.plot()  # boxes + masks drawn by ultralytics
        for p in out.persons:
            label = f'{p.distance:.2f} m' if not math.isnan(p.distance) else 'no depth'
            cv2.circle(annotated, (p.pixel_x, p.pixel_y), 5, (0, 0, 255), -1)
            cv2.putText(annotated, label, (p.pixel_x + 8, p.pixel_y),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        msg.header = header
        self.pub_debug.publish(msg)

    def destroy_node(self):
        self._stop = True
        self._worker.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PersonDistanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
