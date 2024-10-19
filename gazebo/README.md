# Gazebo

As Current JetPack 6 is a derived of Ubuntu 22.04, we will stick with Ubuntu 22.04 to minimize compatibility issues.
In addition, we will use ROS2 Humble as it is a LTS version that supports Ubuntu 22.04.
With this bundle of environments, it is recommended to use Gazebo Fortress.
See this [document](https://gazebosim.org/docs/fortress/ros_installation) for ROS2 and Gazebo version pairing.
Make sure you are looking for the documents with the right version.
For example,

| Differences  | Fortress           | Harmonic |
| ------------ | ------------------ | -------- |
| sdf version  | 1.8                | 1.10     |
| plug-in name | libignition-gazebo | gz-sim   |

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

## Run remote control simulation

First we need to start a remote control node from [base_station](https://github.com/boilerrobotics/base-station/tree/main/src/joystick) repo.

```bash
ros2 launch joystick diff_drive_launch.yml
```

Joy node will read inputs from the joystick then map those inputs into Twist message.  
Then navigate to Gazebo folder and run following command to bridge Twist message from ROS to Gazebo.
See [ref1](https://gazebosim.org/docs/fortress/ros2_integration) and [ref2](https://index.ros.org/p/ros_gz_bridge/) for more detail about parameters.

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=config.yaml
```

Then start Gazebo.
After you start simulation and click on `play` button (or press space on your keyboard), you will be able to control the robot with the joystick

```bash
ign gazebo tutorial.sdf
```

### Run IMU

```bash
ign topic -e -t /imu
```

# View camera

```bash
rqt
```
