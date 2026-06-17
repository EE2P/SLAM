# One person detected by YOLO segmentation, with distance computed from the
# aligned depth image over the segmentation mask.
int32 id                      # detection index within this frame
int32 track_id                # stable tracking id across frames (-1 = untracked)
float32 confidence            # YOLO confidence score [0..1]
float32 distance              # average distance over the mask, meters (NaN if no valid depth)
geometry_msgs/Point position  # 3D position of the mask centroid in the camera optical frame, meters
uint32 pixel_x                # mask centroid, image pixels
uint32 pixel_y
uint32 valid_depth_pixels     # number of depth pixels used for the average
