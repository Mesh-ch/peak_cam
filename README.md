# peak_cam

A Linux ROS 2 launch and configuration package that drives IDS cameras through
[`camera_aravis2`](https://github.com/FraunhoferIOSB/camera_aravis2).

`camera_aravis2` uses the open-source [Aravis](https://github.com/AravisProject/aravis)
library and communicates with cameras via the GigE Vision and USB3 Vision
(GenICam) protocols, which are fully supported by IDS cameras.

## Prerequisites

1. Install ROS 2 (Humble or newer recommended).
2. Install `camera_aravis2` and its dependencies (Aravis library):

   ```bash
   # Replace <ros-distro> with your ROS 2 distribution name, e.g. humble or jazzy
   sudo apt-get install ros-<ros-distro>-camera-aravis2
   ```

   Or build from source:

   ```bash
   cd camera_ws/src
   git clone https://github.com/FraunhoferIOSB/camera_aravis2.git
   ```

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

- `guid`: GUID or serial number of the IDS camera to open (e.g. `"4103929441"`).
  Leave empty (`""`) to open the first available camera.
- `frame_id`: frame id used in the published messages
- `Width` and `Height`: requested image size
- `AcquisitionFrameRate`: requested frame rate (Hz)
- `ExposureTime`: exposure time in microseconds (used when `ExposureAuto` is `"Off"`)
- `ExposureAuto`, `GainAuto`, `GainSelector`, `PixelFormat`, `Gamma`

For a full list of available GenICam features, refer to your camera's
SFNC (Standard Features Naming Convention) documentation or the
[camera_aravis2 documentation](https://github.com/FraunhoferIOSB/camera_aravis2).

## Run

Launch the ROS 2 node with:

```bash
ros2 launch peak_cam peak_cam.launch.py
```

The launch file loads parameters from `params/settings/peak_cam_params.yaml`
and starts a `camera_aravis2::CameraAravisNode` composable node inside a
component container.

> Hint: Sometimes cameras are only accessible as root. If needed, try `sudo -s`
> and launch the node again, or configure udev rules to give regular users access.

Copyright (c) 2020, Sherif Nekkah

All rights reserved.

BSD license: see LICENSE file
