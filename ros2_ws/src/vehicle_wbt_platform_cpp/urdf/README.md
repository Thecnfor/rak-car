# vehicle_wbt URDF

The canonical robot description. Both `urdf/vehicle_wbt.urdf.xacro` (this directory, symlinked from root `urdf/`) and the source-of-truth for the platform.

## Source of truth

`docs/hardware-port-mapping.md` is the source of truth for which physical port connects to what. The xacro translates that mapping into a URDF that `robot_state_publisher` + `ros2_control` controller_manager can consume.

## Validation

```bash
# 1. Check xacro expands to valid URDF
xacro urdf/vehicle_wbt.urdf.xacro > /tmp/check.urdf
check_urdf /tmp/check.urdf

# 2. Visualize in RViz
ros2 launch robot_state_publisher robot_state_publisher.launch.py \
  robot_description:="$(xacro urdf/vehicle_wbt.urdf.xacro)"
ros2 run rviz2 rviz2

# 3. View TF tree
ros2 run tf2_tools view_frames
```

## Spec ref

`docs/superpowers/specs/2026-07-05-ros2-sidecar-design.md §Chassis 抽象` + `§机械臂抽象`
