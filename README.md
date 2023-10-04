# onrobot

[![support level: community](https://img.shields.io/badge/support%20level-community-lightgray.svg)](https://rosindustrial.org/news/2016/10/7/better-supporting-a-growing-ros-industrial-software-platform)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

ROS 2 drivers for OnRobot Grippers.
This repository was inspired by [Osaka-University-Harada-Laboratory/onrobot](https://github.com/Osaka-University-Harada-Laboratory/onrobot.git).

## Features

- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html)
- Controller for OnRobot [RG2](https://onrobot.com/en/products/rg2-gripper) / [RG6](https://onrobot.com/en/products/rg6-gripper) via Modbus/TCP


## Dependency

- pymodbus==2.5.3  
 
## Installation

```bash
cd my_ws/src
git clone https://github.com/Tiqo01/onrobot2.git 
cd ../
sudo rosdep install --from-paths ./src --ignore-packages-from-source 
colcon build
```
### RG2 / RG6

#### Send motion commands

##### Interactive mode

```bash
ros2 launch onrobot_rg_control bringup.launch.py 
ros2 run onrobot_rg_control OnRobotRGSimpleController.py
```

##### ROS service call

```bash
ros2 launch onrobot_rg_control bringup.launch.py 
ros2 run onrobot_rg_control OnRobotRGSimpleControllerServer.py
ros2 service call /onrobot_rg/set_command "{command: 'c'}"
ros2 service call /onrobot_rg/set_command "{command: 'o'}"
ros2 service call /onrobot_rg/set_command "{command: 'c'}"
```
## License

This software is released under the MIT License, see [LICENSE](./LICENSE).

