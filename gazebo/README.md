# Gazebo

As Current JetPack 6 is a derived of Ubuntu 22.04, we will stick with Ubuntu 22.04 to minimize compatibility issues.
In addition, we will use ROS2 Humble as it is a LTS version that supports Ubuntu 22.04.
With this bundle of environments, it is recommended to use Gazebo Fortress.
See this [document](https://gazebosim.org/docs/fortress/ros_installation) for ROS2 and Gazebo version pairing.

## Installation

You must install ROS2 before installing Gazebo.
Then run this following command.

```bash
 sudo apt install ros-humble-ros-gz
```

Noted: if you use other version of ROS, you might need to compile by source
Check this [document](https://github.com/gazebosim/ros_gz).

## ROS-Gazebo integration

You might see `ros_ign_xxx` in some documents.
These commands are likely to be missing as the names have been changed to `ros_gz_xxx`.
If you just copy commands from the tutorial and run into errors, check the command's name.
