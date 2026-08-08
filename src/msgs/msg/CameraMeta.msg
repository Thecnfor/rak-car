# CameraMeta.msg — vendor / driver-level metadata for a single camera frame.
# Spec: docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §Camera 抽象
#
# sensor_msgs/CameraInfo is the canonical calibration topic. THIS message is
# for runtime driver-level data (exposure, gain, white balance, sensor temp)
# that change every frame, so it doesn't belong in CameraInfo.
#
# Published alongside image_raw at a lower rate (e.g. 1 Hz) so the topic
# doesn't carry duplicate static data every frame.
#
# All numeric fields default to NaN if the driver does not report them.

std_msgs/Header header
string camera_id                  # mirrors parameter, e.g. "front", "arm"

# Exposure
float32 exposure_us               # exposure time, microseconds; NaN if unknown
bool   exposure_auto              # true if auto-exposure is enabled

# Gain / white balance
float32 gain                      # analog gain, unitless multiplier; NaN if unknown
bool   gain_auto
uint16 white_balance_kelvin       # color temperature setting; 0 if unknown
bool   white_balance_auto

# Sensor health
float32 temperature_c             # sensor die temperature; NaN if no sensor

# Frame source attribution (for multi-camera / multi-source debugging)
string device_path                # /dev/cam4 (no hardcoding — passed as param)
