# PX4-ROS2 bridge

[![GitHub license](https://img.shields.io/github/license/PX4/px4_ros_com.svg)](https://github.com/PX4/px4_ros_com/blob/master/LICENSE) [![GitHub (pre-)release](https://img.shields.io/github/release-pre/PX4/px4_ros_com.svg)](https://github.com/PX4/px4_ros_com/releases/tag/beta) [![DOI](https://zenodo.org/badge/142936318.svg)](https://zenodo.org/badge/latestdoi/142936318) [![Build and Test package](https://github.com/PX4/px4_ros_com/workflows/Build%20and%20Test%20package/badge.svg?branch=master)](https://github.com/PX4/px4_ros_com/actions)

[![Discord Shield](https://discordapp.com/api/guilds/1022170275984457759/widget.png?style=shield)](https://discord.gg/dronecode)

This package provides example nodes for exchanging data and commands between ROS2 and PX4.
It also provides a [library](./include/px4_ros_com/frame_transforms.h) to ease the conversion between ROS2 and PX4 frame conventions.
It has a straight dependency on the [`px4_msgs`](https://github.com/PX4/px4_msgs) package.

## Install, build and usage

Check the [uXRCE-DDS](https://docs.px4.io/main/en/middleware/uxrce_dds.html) and the [ROS2 Interface](https://docs.px4.io/main/en/ros/ros2_comm.html) sections on the PX4 Devguide for details on how to install the required dependencies, build the package and use it.

## Bug tracking and feature requests

Use the [Issues](https://github.com/PX4/px4_ros_com/issues) section to create a new issue. Report your issue or feature request [here](https://github.com/PX4/px4_ros_com/issues/new).

## Questions and troubleshooting

Reach the PX4 development team on the [PX4 Discord Server](https://discord.gg/dronecode).

## Run instructions

1) Start SITL: 
```
PX4_GZ_WORLD=walls make px4_sitl gz_x500_depth
```
Variables `PX4_GZ_WORLD=<world_name>`. 

2) Run MicroDDS: 
```
MicroXRCEAgent udp4 -p 8888; bash
```

3) Run offboard control as follows: 
```
ros2 launch px4_ros_com offboard_control.launch.py 
```

Gazebo world is located in: 
```
Starting gazebo with world: /root/sitl/PX4-Autopilot/Tools/simulation/gz/worlds/default.sdf
```

### Run bridge

Run ros_gz_bridge to move image from the gz_sim to the ROS 2: 
```
ros2 run ros_gz_image image_bridge /world/walls/model/x500_depth_0/link/camera_link/sensor/IMX214/image /rgb_cam/image
```
Run ros_gz_bridge to move wrench from the gz_sim to ROS 2
```
ros2 run ros_gz_bridge parameter_bridge /world/walls/model/x500_stick_0/joint/cap_fixed_joint/sensor/force_torque/forcetorque@geometry_msgs/msg/Wrench@gz.msgs.Wrench
```

Full force torque mapping command with topic name remapping: 
```
ros2 run ros_gz_bridge parameter_bridge /world/walls/model/x500_stick_0/joint/cap_fixed_joint/sensor/force_torque/forcetorque@geometry_msgs/msg/Wrench@gz.msgs.Wrench --ros-args -r /world/walls/model/x500_stick_0/joint/cap_fixed_joint/sensor/force_torque/forcetorque:=/measured_force
```

### Params to disable GCS check 

Set params here: 
```
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
```

