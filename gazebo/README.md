# Gazebo

As Current JetPack 6 is a derived of Ubuntu 22.04, we will stick with Ubuntu 22.04 to minimize compatibility issues.
In addition, we will use ROS2 Humble as it is a LTS version that supports Ubuntu 22.04.
With this bundle of environments, it is recommended to use Gazebo Fortress.
See this [document](https://gazebosim.org/docs/fortress/ros_installation) for ROS2 and Gazebo version pairing.

Nvidia announced plan to port Jetpack 7 (based on Ubuntu 24.04 Noble) back to Jetson Orin series in Q2 of 2026.
We will upgrade our working environment to Jetpack 7, therefore our Ubuntu version will be 24.04.
Thus, ROS2 will be also upgraded to ROS2 Jazzy.
The official Gazebo [documenet](https://gazebosim.org/docs/harmonic/ros_installation/) recommend using Gazebo Harmonic on Ubuntu 24.04.
If you are using Ubuntu 24.04, please refer to instruction for Gazebo Harmonic.

Please be aware of key changes between Gazebo Fortress (and earlier versions) and Gazebo Harmonic (and later version) as follow.

| Differences  | Fortress           | Harmonic |
| ------------ | ------------------ | -------- |
| sdf version  | 1.8                | 1.10     |
| plug-in name | libignition-gazebo | gz-sim   |

## Installation

You should install ROS2 before installing Gazebo.
Then run this following command.

```bash
 sudo apt install ros-{ROS_DISTRO}-ros-gz
```

Noted: if you use other version of ROS, you might need to compile by source
Check this [document](https://github.com/gazebosim/ros_gz).

## Docker for non-Linux machine

If you use Linux, you can install ROS2 and Gazebo directly.
But if your machine is not Linux, you can

1. Use Virtual Box or other emulator to run Linux
2. Use Docker

If you choose to use Docker, you will to install Docker Engine (Docker Desktop is optional).
It is recommended to manage Docker by VS Code extension.
To do so, you will need 2 VS Code Extensions: Docker and Dev Containers.

In VS Code, open `rover-code` repo then open command palette and search for `Dev Containers: Reopen in Container`.
The VS Code will

1. Build a container following the instructions in `.devcontainer/devcontainer.json`
2. Run the container
3. Attach VS Code to the container
4. Open `rover-code` folder in the container

As a result, you will have VS Code open in a container that has ROS2 Jazzy and Gazebo Harmonic installed with the `rover-code` repo ready for development/test.

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
