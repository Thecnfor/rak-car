# Pose2D.msg — 底盘系 2D 位姿。
# Spec: docs/superpowers/specs/2026-08-09-ros2-layering-interfaces-design.md §7.1
#
# 新版 ROS2(Lyrical 起)已从 geometry_msgs 移除废弃的 Pose2D —— 自建轻量
# 类型,避免跨发行版漂移,且比 3D Pose(四元数)更省。与 C++ hardware::Pose2D
# (base_chassis.hpp) 同构,x/y 米,theta 弧度。

float64 x       # 米
float64 y       # 米
float64 theta   # 弧度,[-pi, pi]
