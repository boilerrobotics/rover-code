# BRC

### For Communication and Intellgence:
- Setup ROS Melodic on Ubuntu 18.03 follwing this link: http://wiki.ros.org/melodic/Installation/Ubuntu
- Clone this project into your machine
- Go to commu-intell -> ros
- Run `catkin_make`
- Run `setup.bash`

### For Robot Arm:
- Setup ROS Noetic on Ubuntu 20.04 follwing this [link](http://wiki.ros.org/noetic/Installation/Ubuntu).
- Clone this project into your machine, and use the [arm](https://github.com/boilerrobotics/BRC2019-2020/tree/arm) branch
- Navigate to the **robot-arm** directory
    - The ros directory contains code used for the robot arm driver and communication with the rest of the rover
    - The ws_moveit directory containts code for controlling simulated robot arms and inverse kinematics
        - Follow the [Getting Started](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html) tutorial. Some files will already be included by this repository.
    - Both directories will eventually be integrated
- More information about MoveIt can be learned using this set of [tutorials](https://ros-planning.github.io/moveit_tutorials/).
