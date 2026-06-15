"""Benchmark the YOLO11-seg perception: inference time + achievable FPS.

Runs the SAME model and parameters as person_distance_node on the live colour
stream, but does nothing else (no depth fusion, no publishing), so the numbers
reflect the detector's own speed. Run it INSTEAD of the person_distance node so
the two do not share the GPU.

It prints a rolling line while running and a final summary on Ctrl-C (or after
`duration_s`) that you can quote directly in the report; with `csv_path` set it
also dumps every frame's timing for an offline plot.

    # one command (brings up the camera + this node):
    ros2 launch person_distance benchmark.launch.py

    # or, if the camera is already running:
    ros2 run person_distance perception_benchmark
    ros2 run person_distance perception_benchmark --ros-args -p duration_s:=30.0 \
        -p csv_path:=~/SLAM/bags/perception_benchmark.csv
"""
import os
import statistics
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from cv_bridge import CvBridge
from sensor_msgs.msg import Image

PERSON_CLASS_ID = 0  # COCO class 0 = person


class PerceptionBenchmark(Node):

    def __init__(self):
        super().__init__('perception_benchmark')

        self.declare_parameter('color_topic', '/camera/camera/color/image_raw')
        self.declare_parameter('model', os.path.expanduser('~/SLAM/models/yolo11n-seg.pt'))
        self.declare_parameter('device', '')                 # '' = auto (cuda if available)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('warmup_frames', 15)          # ignored in the stats
        self.declare_parameter('report_period_s', 2.0)       # rolling print cadence
        self.declare_parameter('duration_s', 0.0)            # 0 = run until Ctrl-C
        self.declare_parameter('csv_path', '')               # '' = no per-frame CSV

        self.color_topic = self.get_parameter('color_topic').value
        self.conf = float(self.get_parameter('confidence_threshold').value)
        self.warmup = int(self.get_parameter('warmup_frames').value)
        self.report_period = float(self.get_parameter('report_period_s').value)
        self.duration = float(self.get_parameter('duration_s').value)
        self.csv_path = self.get_parameter('csv_path').value

        self.bridge = CvBridge()
        self.device = ''
        self.model = self._load_model()

        # post-warmup measurement state
        self.frame_count = 0
        self.rows = []          # (t_rel, predict_ms, infer_ms, pre_ms, post_ms, n_persons)
        self.t0 = None          # completion time of the first measured frame
        self._win_t0 = None
        self._win_n = 0
        self.done = False

        # depth-1 best-effort: always hand us the freshest frame and drop stale
        # ones, so the measured rate is the achievable processing rate.
        qos = QoSProfile(depth=1,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT,
                         history=QoSHistoryPolicy.KEEP_LAST)
        self.sub = self.create_subscription(Image, self.color_topic, self._cb, qos)
        self.get_logger().info(
            f'Benchmarking YOLO11-seg on {self.color_topic} '
            f'(warmup {self.warmup} frames). Ctrl-C for the final summary.')

    def _load_model(self):
        from ultralytics import YOLO
        import torch

        model_path = self.get_parameter('model').value
        device = self.get_parameter('device').value
        if not device:
            device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f'Loading {model_path} on {device} ...')
        model = YOLO(model_path)
        if str(model_path).endswith('.pt'):
            model.to(device)  # exported .engine/.onnx are device-bound already
        model.predict(np.zeros((480, 640, 3), dtype=np.uint8), verbose=False)  # warm up
        self.device = device
        self.get_logger().info('Model ready')
        return model

    def _cb(self, msg: Image):
        if self.done:
            return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        t = time.perf_counter()
        result = self.model.predict(
            img, classes=[PERSON_CLASS_ID], conf=self.conf,
            retina_masks=True, verbose=False)[0]
        predict_ms = (time.perf_counter() - t) * 1e3

        sp = result.speed  # ultralytics' own ms: preprocess / inference / postprocess
        n_persons = 0 if result.boxes is None else int(len(result.boxes))

        self.frame_count += 1
        if self.frame_count <= self.warmup:
            return

        now = time.perf_counter()
        if self.t0 is None:
            self.t0 = now
            self._win_t0 = now
        self.rows.append((now - self.t0, predict_ms, float(sp['inference']),
                          float(sp['preprocess']), float(sp['postprocess']), n_persons))
        self._win_n += 1

        if now - self._win_t0 >= self.report_period:
            fps = self._win_n / (now - self._win_t0)
            recent_infer = [r[2] for r in self.rows[-self._win_n:]]
            self.get_logger().info(
                f'fps={fps:5.1f}   inference={statistics.mean(recent_infer):5.1f} ms '
                f'(p95 {np.percentile(recent_infer, 95):4.1f})   persons={n_persons}')
            self._win_t0 = now
            self._win_n = 0

        if self.duration > 0.0 and (now - self.t0) >= self.duration:
            self.done = True

    def summary(self):
        if len(self.rows) < 2:
            self.get_logger().warn('Not enough frames measured for a summary.')
            return
        predict = [r[1] for r in self.rows]
        infer = [r[2] for r in self.rows]
        pre = [r[3] for r in self.rows]
        post = [r[4] for r in self.rows]
        span = self.rows[-1][0]
        n = len(self.rows)
        fps = (n - 1) / span if span > 0 else 0.0

        def st(a):
            return (statistics.mean(a), statistics.median(a),
                    float(np.percentile(a, 95)), min(a), max(a))

        im, pm = st(infer), st(predict)
        line = '=' * 66
        print('\n' + line)
        print(f' PERCEPTION BENCHMARK  --  YOLO11n-seg on {self.device}')
        print(line)
        print(f'  frames measured (post-warmup) : {n}')
        print(f'  wall time                     : {span:.1f} s')
        print(f'  achievable throughput (FPS)   : {fps:.1f} Hz')
        print(f'  model inference [ms]  mean/median/p95/min/max : '
              f'{im[0]:.1f} / {im[1]:.1f} / {im[2]:.1f} / {im[3]:.1f} / {im[4]:.1f}')
        print(f'  full predict()  [ms]  mean/median/p95/min/max : '
              f'{pm[0]:.1f} / {pm[1]:.1f} / {pm[2]:.1f} / {pm[3]:.1f} / {pm[4]:.1f}')
        print(f'  preprocess / postprocess [ms] mean : '
              f'{statistics.mean(pre):.1f} / {statistics.mean(post):.1f}')
        print(line)
        print('  Report-ready numbers:')
        print(f'    FPS               : {fps:.0f} Hz (640x480 stream)')
        print(f'    mean inference    : {im[0]:.0f} ms   (p95 {im[2]:.0f} ms)')
        print(line + '\n')

        if self.csv_path:
            path = os.path.expanduser(self.csv_path)
            os.makedirs(os.path.dirname(path) or '.', exist_ok=True)
            with open(path, 'w') as f:
                f.write('t_s,predict_ms,inference_ms,preprocess_ms,postprocess_ms,n_persons\n')
                for r in self.rows:
                    f.write(f'{r[0]:.4f},{r[1]:.3f},{r[2]:.3f},{r[3]:.3f},{r[4]:.3f},{r[5]}\n')
            self.get_logger().info(f'wrote {n} per-frame rows to {path}')


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionBenchmark()
    try:
        if node.duration > 0.0:
            # spin until the measurement window closes (with generous slack for warmup)
            deadline = time.perf_counter() + node.duration + 10.0
            while rclpy.ok() and not node.done and time.perf_counter() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
        else:
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.summary()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
