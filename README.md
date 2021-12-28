# robot_simulation
文档解析查看
chargerKong.github.io
ROS2 全套仿真小车

打开模拟机器人，机器人包括IMU, 单线雷达，里程计
```
ros2 launch robot_description gazebo_lab_world,launch.py
```

cartographer建图
```
ros2 launch my_slam cartographer.launch.py
```

打开导航
```
ros2 launch nav2_robot nav2_gazebo.launch.py
```