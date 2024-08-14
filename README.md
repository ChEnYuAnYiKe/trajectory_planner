## TURTLEBOT操作指南
### 1. robot上

robot_5(anc 3) IP: 192.168.31.121

* 提前在PC端启动roscore

* 启动turtlebot，并桥接控制话题

```bash
sudo ntpdate -u 192.168.31.176
sros1
sros2
ros2 run ros1_bridge dynamic_bridge
```

* 更改livox_ws/FAST-LIO下`laserMapping.cpp`文件第851行里程计输出话题`/Odometry`为`/robot_X/Odometry`

* 启动FAST-LIO

```bash
# 更改以太网口IP地址
sudo ifconfig eth0 192.168.1.50
source /livox_ws/devel/setup.bash
roslaunch livox_ros_driver2 msg_MID360.launch
roslaunch fast_lio mapping_mid360_msg.launch
```

### 2. PC上

PC IP: 192.168.31.176
运行pub_tbX_vel_cmd.sh
