# peak_cam

A ROS 2 package for IDS peak cameras.  The package ships **two equivalent
nodes** that can be used interchangeably:

| Node | Executable | Language | SDK requirement |
|---|---|---|---|
| C++ node | `peak_cam_node` | C++ / ament_cmake | Full IDS peak C++ SDK (headers + shared libs) |
| **Python node** | `peak_cam_py_node` | Python / rclpy | IDS peak native transport layers + `pip install ids-peak ids-peak-ipl` |

The Python node (`peak_cam_py_node`) is the **recommended path for new
installations** on Ubuntu 22/24 and WSL2: no C++ compilation is required.

| Platform | ROS 2 distro | Status |
|---|---|---|
| Ubuntu 22.04 (Jammy) | Humble Hawksbill | Supported |
| Ubuntu 24.04 (Noble) — incl. WSL2 | Jazzy Jalopy | Supported |

---

## Python node (pip-based, no C++ compilation)

### Why the PyPI package is not a full SDK replacement

The [`ids-peak`](https://pypi.org/project/ids-peak/) and
[`ids-peak-ipl`](https://pypi.org/project/ids-peak-ipl/) packages on PyPI
provide **Python bindings only**.  As stated in their documentation:

> *"ids_peak … requires at least the drivers and GenICam transport layers to
> be installed, which are included in the IDS peak SDK."*

This means:
- ✅ You do **not** need the IDS peak C++ headers or CMake integration
- ✅ You do **not** need a C++ compiler or build toolchain
- ❌ You **still need** the IDS peak SDK installed for the native shared
  libraries (`libids_peak.so`, `libids_peak_ipl.so`) and GenICam transport
  layer (CTI) files

### 1. Install ROS 2

Follow the official instructions for your Ubuntu release:

- **Ubuntu 22.04**: [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- **Ubuntu 24.04**: [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

### 2. Install the minimal IDS peak SDK

Download the latest IDS peak installer for Linux from the IDS website:

> <https://en.ids-imaging.com/ids-peak.html>

Install the `.deb` package (the full SDK is needed for the shared libs and
CTI transport layer files):

```bash
sudo apt-get install -y libusb-1.0-0 libatomic1
sudo dpkg -i ids-peak-linux-x86-<version>-64.deb
sudo apt-get install -f   # fix any remaining dependency issues
```

> **Note (Ubuntu 22/24):** IDS peak 1.x was built for Ubuntu 18/20.
> Use IDS peak **2.x or later** on Ubuntu 22 or 24.

### 3. Install the Python bindings from PyPI

```bash
pip install ids-peak ids-peak-ipl
```

Also install the Python ROS 2 messaging dependencies:

```bash
sudo apt-get install -y python3-numpy ros-<distro>-camera-info-manager
```

### 4. Build

```bash
mkdir -p camera_ws/src
cp -r peak_cam camera_ws/src/
cd camera_ws
source /opt/ros/<ros-distro>/setup.bash   # e.g. jazzy or humble
colcon build --packages-select peak_cam
source install/setup.bash
```

### 5. Run

```bash
ros2 run peak_cam peak_cam_py_node --ros-args --params-file \
  install/peak_cam/share/peak_cam/params/settings/peak_cam_params.yaml
```

---

## C++ node (requires full IDS peak C++ SDK)

### Prerequisites

#### 1. Install ROS 2

See links above.

#### 2. Install IDS peak C++ SDK

The C++ node requires the IDS peak SDK (≥ 2.x for Ubuntu 22/24) with the
`ids_peak` and `ids_peak_ipl` CMake packages.

```bash
sudo apt-get install -y libqt5core5a libqt5gui5 libqt5widgets5 libusb-1.0-0 libatomic1
sudo dpkg -i ids-peak-linux-x86-<version>-64.deb
sudo apt-get install -f
```

Verify CMake can find the packages:

```bash
dpkg -L ids_peak | grep cmake
```

> **Note (Ubuntu 22/24):** IDS peak 1.x will not install cleanly.
> Use IDS peak **2.x or later**.

### Build

```bash
mkdir -p camera_ws/src
cp -r peak_cam camera_ws/src/
cd camera_ws
source /opt/ros/<ros-distro>/setup.bash
colcon build --packages-select peak_cam
source install/setup.bash
```

### Run

```bash
ros2 launch peak_cam peak_cam.launch.py
```

---

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

Then proceed with the build and run steps above.

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

---

## Configuration

Edit [`params/settings/peak_cam_params.yaml`](params/settings/peak_cam_params.yaml) before launching.

Common parameters:

- `selectedDevice`: serial number of the IDS camera to open
- `image_topic`: image topic suffix published by the node
- `frame_id`: frame id used in the published messages
- `ImageWidth` and `ImageHeight`: requested image size
- `AcquisitionFrameRate`: requested frame rate
- `ExposureTime`, `ExposureAuto`, `GainAuto`, `GainSelector`, `PixelFormat`

Both nodes accept the same parameter file.

If no valid camera calibration file is available, the node will still run and publish uncalibrated `CameraInfo`.

For multiple cameras, create separate parameter files and launch descriptions for each camera instance.

> Hint: Sometimes the cameras are only accessible as root. If needed, try `sudo -s` and launch the node again.

Copyright (c) 2020, Sherif Nekkah

All rights reserved.

BSD license: see LICENSE file
