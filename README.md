# peak_cam

A Linux ROS 2 C++ node that wraps the IDS peak driver API for IDS vision cameras.

## Prerequisites

1. Install ROS 2.
2. Install IDS peak and make sure the `ids_peak` and `ids_peak_ipl` CMake packages are available.

## Build

1. Create a ROS 2 workspace:

   ```bash
   mkdir -p camera_ws/src
   ```

2. Put `peak_cam` into the workspace:

   ```bash
   cp -r peak_cam camera_ws/src/
   ```

3. Source your ROS 2 installation and build the package:

   ```bash
   cd camera_ws
   source /opt/ros/<ros-distro>/setup.bash
   colcon build --packages-select peak_cam
   source install/setup.bash
   ```

## Configuration

Edit [`params/settings/peak_cam_params.yaml`](params/settings/peak_cam_params.yaml) before launching.

Common parameters:

- `selectedDevice`: serial number of the IDS camera to open
- `image_topic`: image topic suffix published by the node
- `frame_id`: frame id used in the published messages
- `ImageWidth` and `ImageHeight`: requested image size
- `AcquisitionFrameRate`: requested frame rate
- `ExposureTime`, `ExposureAuto`, `GainAuto`, `GainSelector`, `PixelFormat`

## Run

Launch the ROS 2 node with:

```bash
ros2 launch peak_cam peak_cam.launch.py
```

The launch file loads parameters from `params/settings/peak_cam_params.yaml`.

If no valid camera calibration file is available, the node will still run and publish uncalibrated `CameraInfo`.

For multiple cameras, create separate parameter files and launch descriptions for each camera instance.

> Hint: Sometimes the cameras are only accessible as root. If needed, try `sudo -s` and launch the node again.

Copyright (c) 2020, Sherif Nekkah

All rights reserved.

BSD license: see LICENSE file
