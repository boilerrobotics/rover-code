## Configuration

There are some configurations that need to be changed from the default.

- `zed2-ros2-wrapper/zed_wrapper/config/zed2.yaml`
  - Change `grab_resolution` to `VGA` to lower bandwidth needs to stream video back.
  - Change `grab_frame_rate` to `15` to lower bandwidth needs and reduce image processing workload.

- `zed2-ros-wrapper/zed_wrapper/config/common_stereo.yaml`
  - Change `pub_resolution` to `NATIVE`
  - Change `pub_frame_rate` to `15`
  - Change `point_cloud_freq` to `15.0`
