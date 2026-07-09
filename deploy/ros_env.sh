# /etc/vehicle-wbt/ros.env - Jetson 端 MC602 节点环境变量
# 由 vehicle-wbt-*.service 的 EnvironmentFile= 加载

# ROS2 网络(对齐同事 dev box)
ROS_DOMAIN_ID=42
ROS_LOCALHOST_ONLY=0

# DDS: CycloneDDS
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# 仓库路径 + 串口(可被 launch / service 引用)
VEHICLE_WBT_REPO=/home/xrak/workspace/rak-car
VEHICLE_WBT_SERIAL=/dev/ttyUSB0
VEHICLE_WBT_BAUD=1000000
