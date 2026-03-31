# peak_cam

A Linux ROS 2 C++ node that wraps the IDS peak driver API for IDS vision cameras.

| Platform | ROS 2 distro | Status |
|---|---|---|
| Ubuntu 22.04 (Jammy) | Humble Hawksbill | Supported |
| Ubuntu 24.04 (Noble) — incl. WSL2 | Jazzy Jalopy | Supported |

## Prerequisites

### 1. Install ROS 2

Follow the official instructions for your Ubuntu release:

- **Ubuntu 22.04**: [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- **Ubuntu 24.04**: [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

### 2. Install IDS peak

The node requires the IDS peak SDK (≥ 2.x for Ubuntu 22/24) with the
`ids_peak` and `ids_peak_ipl` CMake packages.

Download the latest IDS peak installer for Linux from the IDS website:

> <https://en.ids-imaging.com/ids-peak.html>

Install the downloaded `.deb` package:

```bash
sudo apt-get install -y libqt5core5a libqt5gui5 libqt5widgets5 libusb-1.0-0 libatomic1
sudo dpkg -i ids-peak-linux-x86-<version>-64.deb
sudo apt-get install -f   # fix any remaining dependency issues
```

After installation, verify that CMake can find the packages:

```bash
dpkg -L ids_peak | grep cmake
```

> **Note (Ubuntu 22/24):** IDS peak 1.x was built for Ubuntu 18/20 and will
> not install cleanly on Ubuntu 22 or 24.  Use IDS peak **2.x or later**
> (available from the IDS download page above).

## WSL2 Setup (Ubuntu 22/24 on Windows)

Running the node inside WSL2 requires forwarding the USB camera to the Linux
guest with [usbipd-win](https://github.com/dorssel/usbipd-win).

### Windows host

1. Install **usbipd-win** (≥ 4.x):

   ```powershell
   winget install --interactive --exact dorssel.usbipd-win
   ```

2. Identify and attach the IDS camera:

   ```powershell
   usbipd list                    # note the BUSID for your camera
   usbipd bind   --busid <BUSID>  # one-time: share the device
   usbipd attach --wsl --busid <BUSID>
   ```

### WSL2 guest (Ubuntu)

Verify the camera is visible inside WSL2:

```bash
lsusb   # should show the IDS camera
```

Then proceed with the normal build and run steps below.

> **Hint:** After attaching via usbipd the camera may only be accessible as
> root.  If needed, run `sudo -s` before launching the node, or add a udev
> rule so that non-root users can access it:
>
> ```bash
> echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="<vendor_id>", MODE="0666"' \
>   | sudo tee /etc/udev/rules.d/99-ids-camera.rules
> sudo udevadm control --reload-rules && sudo udevadm trigger
> ```
>
> Replace `<vendor_id>` with the hex vendor ID shown by `lsusb` (IDS cameras
> typically use `0x2474`).

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
   source /opt/ros/<ros-distro>/setup.bash   # e.g. jazzy or humble
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
