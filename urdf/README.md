# URDF

This directory is a symlink to the canonical URDF at `ros2_ws/src/vehicle_wbt_platform_cpp/urdf/vehicle_wbt.urdf.xacro`.

```bash
# Validate
xacro urdf/vehicle_wbt.urdf.xacro | head -50
# Or use check_urdf
xacro urdf/vehicle_wbt.urdf.xacro > /tmp/check.urdf && check_urdf /tmp/check.urdf
```

**Edit the source file** at `ros2_ws/src/vehicle_wbt_platform_cpp/urdf/`, not here.
