## Install or Update SDK

[Reference from official ZED2 document](https://www.stereolabs.com/docs/development/zed-sdk/jetson)

The SDK is available to download with this [link](https://www.stereolabs.com/developers/release).
You will need to choose SDK for Nvidia Jetson with the correct JetPack version.
Refer to this [document](../jetson/README.md) if you need to update JetPack version.
You can use command `cat /etc/nv_tegra_release` to check currently installed JetPack version (look for release and revision numbers).
Or observe JetPack (Nvidia Linux version) during the boot up process.
Run installation file and follow the instructions.
You can check verify SDK version by running `ZED_Explorer` and look at the number at the bottom panel.

## Calibration

Run `ZED_Calibration` and follow instructions.
You can use other ZED tools as list [here](https://www.stereolabs.com/docs/development/zed-tools).

## ZED + ROS2

We will use ROS2 Wrapper to integrate with other components.
ZED provide

1. [ROS2 Wrapper](https://github.com/stereolabs/zed-ros2-wrapper): the ZED camera functions in ROS2 workspace.
2. [ROS2 Example](https://github.com/stereolabs/zed-ros2-examples): the example repo
3. [ROS2 Document](https://www.stereolabs.com/docs/ros2): Official document

Be sure that you match wrapper and example release version with the ZED SDK version.
You will need to compile these two repos (using `colcon build`) and install (`source install/local_setup.bash`) in the same way as other ROS2 workspace.
Optional, you can add `source install/local_setup.bash` into `.bashrc` so you don't need to run the command every time you open a new terminal.

To start the camera, run `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2`.
The camera will create a node and publish data in several topics as list [here](https://www.stereolabs.com/docs/ros2/zed-node#published-topics).

## Configuration

Full list of configuration is [here](https://www.stereolabs.com/docs/ros2/zed-node#configuration-parameters)

There are some configurations that need to be changed from the default.

- `zed2-ros2-wrapper/zed_wrapper/config/zed2.yaml`
  - Change `grab_resolution` to `VGA` to lower bandwidth needs to stream video back.
  - Change `grab_frame_rate` to `15` to lower bandwidth needs and reduce image processing workload.

- `zed2-ros-wrapper/zed_wrapper/config/common_stereo.yaml`
  - Change `pub_resolution` to `NATIVE`
  - Change `pub_frame_rate` to `15`
  - Change `point_cloud_freq` to `15.0`
