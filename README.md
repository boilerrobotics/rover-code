# For Communication and Intellgence

### Make sure to work on branches instead of master.

<ul>
    <li> <b>components</b>: This folder is for individual parts, components, or experiments
    <li> <b>src</b>: This folder contains the code that will be running on the rover. It is ROS2 workspace. 
    <li> <b>tutorial</b>: A trining module for new members.
</ul>

The folder [src](./src/) is the main folder to work with.
It is already a ROS2 workspace. To run the code, you will need to compile and install.
To compile

```bash
colcon build
```

Note: You might need to install colcon by

```bash
sudo apt install python3-colcon-common-extensions
```

You can selectively build packages by

```bash
colcon build --packages-select <name-of-pkg>
```

After compilation, you will see a folder `install` appear.
You can install packages by

```bash
source install/setup.bash
```

Some packages have dependencies.
We use `rosdep` to manage them.

```bash
sudo apt-get install python3-rosdep # install rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

All dependencies should be mentioned in each package's `package.xml` whenever possible.

Use the following list and this [tutorial](https://docs.ros.org/en/iron/Tutorials/Intermediate/Rosdep.html).

- [ROS base](https://github.com/ros/rosdistro/blob/master/rosdep/base.yaml)
- [Python](https://github.com/ros/rosdistro/blob/master/rosdep/python.yaml)
