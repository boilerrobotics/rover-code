# AR Tag Detection System


### Launch camera
`ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2`

### View live video (compressed)
`ros2 run image_view image_view --ros-args --remap image/compressed:=/zed/zed_node/rgb/image_rect_color/compressed -p image_transport:=compressed`

### View debug video
`ros2 run image_view image_view --ros-args --remap image:=/aruco_box`

### Run ARtag detection code
`ros2 run artag detect`

## Archive

This folder contains two scripts designed to detect AR (Augmented Reality) tags in images and live video feeds using OpenCV. The detection process involves filtering contours, identifying square shapes, and determining the position of potential AR tags relative to a camera.

### Files:
1. **artagdetectionimage.py**: Detects AR tags from pre-captured images in a specified folder.
2. **artagdetection.py**: Continuously detects AR tags from a live camera feed, providing feedback on the tag's position relative to the camera.

> **Note:** The real-time detection script (`artagdetection.py`) includes more up-to-date changes in terms of contour detection and filtering. The image-based detection script (`artagdetectionimage.py`) is not as recent and may have less efficient methods.


## Workflow

### 1. Image-Based Detection (`artagdetectionimage.py`)

This script processes static images located in an input folder. The workflow includes:

- **Input**: Takes `.jpg` images from a specified input directory.
- **Filtering**: Applies a series of image filters (blurring, thresholding, etc.) to prepare the images for contour detection.
- **Contour Detection**: Detects contours in the image using OpenCV.
- **Square Contour Filtering**: Filters the contours to isolate potential AR tags based on their shape and size (focusing on square contours).
- **Output**: Displays two visual outputs:
  - The original image with filters and thresholds applied.
  - The same image with contours overlaid for easier inspection.

### 2. Real-Time Detection (`artagdetection.py`)

This script uses similar logic to the image-based detection but works in real-time, continuously analyzing frames from a camera feed. The workflow includes:

- **Input**: Captures frames from the default camera (or a specified video input).
- **Filtering**: Applies the same series of image filters as the image-based script.
- **Contour Detection**: Continuously detects contours in the video frames.
- **Square Contour Filtering**: Filters contours to detect potential AR tags, focusing on square shapes.
- **Positioning Feedback**: Identifies the largest potential AR tag and calculates its center. Based on the center's position in the frame, the script prints whether the tag is:
  - **Left**: If the tag is to the left of the camera's center.
  - **Right**: If the tag is to the right of the camera's center.
  - **Centered**: If the tag is within the margin of error in the center of the frame.

The real-time detection script includes more up to date changes in terms of contour detection and filtering. The image-based detection script is not as recent and is to show the basic logic in identifying ar tags.

## Current Limitations & Future Improvements

The AR tag detection logic is currently lacking and needs several improvements to be fully reliable:

- **False Positives**: Currently, the system detects many false positives for AR tags. More robust filtering or shape analysis is needed to ensure only valid tags are detected.
- **Sensitivity to Angles & Distance**: The system struggles to accurately detect AR tags that are positioned at angles or certain distances from the camera. Future work will focus on improving detection in these cases.
- **Fine-Tuning**: Additional work will be needed to fine-tune the filtering and detection process to better identify AR tags in real-world conditions.

Once these improvements are made, the logic in `artagdetection.py` should hopefully be used as a starting point for the rover's AR tag detection software.