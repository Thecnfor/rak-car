# LaneResult.msg — Lane segmentation inference result.
# Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §自定义消息
#
# Published by lane inference component to /vehicle_wbt/v1/perception/lane.
# Consumed by control loop to compute steering correction.

std_msgs/Header header
float32 deviation_distance    # lateral offset from image centerline (m)
float32 deviation_angle       # heading error (rad)
float32 confidence           # 0.0-1.0
bool valid                   # false if inference failed
float32 inference_ms         # wall time for this frame
