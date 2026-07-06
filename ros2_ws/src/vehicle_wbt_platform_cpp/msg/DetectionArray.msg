# DetectionArray.msg — Generic detection result with flat bbox list.
# Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §自定义消息
#
# Published by task / front detection components to
# /vehicle_wbt/v1/perception/detections/<model_id>.
# model_id field replaces the old per-model topic names (task_wbt2025,
# front_model2) so adding a new model is a config change, not a code change.
#
# Detections are flattened into parallel arrays (rather than Detection[] of
# sub-messages) to avoid a separate .msg file. For richer per-detection
# fields, switch to vision_msgs/Detection2DArray in Plan B.

std_msgs/Header header
string model_id               # e.g. "task_wbt2025", "front_model2", "arm_wrist"
string[] class_names         # parallel arrays; len(class_names) == len(scores)
int32[] class_ids
float32[] scores
float32[] xs                  # bbox top-left x, in pixels
float32[] ys                  # bbox top-left y, in pixels
float32[] widths              # bbox width, in pixels
float32[] heights             # bbox height, in pixels
uint32 image_width
uint32 image_height
