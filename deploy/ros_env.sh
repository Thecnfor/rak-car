# /etc/vehicle-wbt/ros.env - Jetson 端 MC602 节点环境变量
# 由 vehicle-wbt-mc602.service 的 EnvironmentFile= 加载

# ROS2 网络
ROS_DOMAIN_ID=42
ROS_LOCALHOST_ONLY=0

# DDS: CycloneDDS(对齐同事 dev box)
RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
CYCLONEDDS_URI=file:///etc/cyclonedds.xml

# 串口(launch 也用得到)
VEHICLE_WBT_SERIAL=/dev/ttyUSB1
