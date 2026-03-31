#!/usr/bin/env python3
# Copyright (c) 2020, Sherif Nekkah
# All rights reserved.
#
# BSD license: see LICENSE file
#
# Python ROS 2 node for IDS peak cameras.
#
# This node uses the `ids-peak` and `ids-peak-ipl` Python packages from PyPI
# instead of the C++ SDK headers, so no C++ compilation or CMake SDK integration
# is required to run it.
#
# IMPORTANT — runtime requirement:
#   The `ids-peak` PyPI package is Python bindings only.  It still requires the
#   IDS peak SDK's native transport layer (CTI files) and shared libraries to be
#   present on the system.  Install the minimal IDS peak SDK from:
#       https://en.ids-imaging.com/ids-peak.html
#   and then install the Python bindings with:
#       pip install ids-peak ids-peak-ipl
#
# Usage:
#   ros2 run peak_cam peak_cam_py_node
#   ros2 run peak_cam peak_cam_py_node --ros-args --params-file <path-to-yaml>

import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

try:
    from camera_info_manager import CameraInfoManager
    _CAMERA_INFO_MANAGER_AVAILABLE = True
except ImportError:
    _CAMERA_INFO_MANAGER_AVAILABLE = False

try:
    from ids_peak import ids_peak
    from ids_peak import ids_peak_ipl_extension
    import ids_peak_ipl
    _IDS_PEAK_AVAILABLE = True
except ImportError:
    _IDS_PEAK_AVAILABLE = False

# Pixel-format name → (ROS encoding, channels-per-pixel)
_PIXEL_FORMAT_MAP = {
    'Mono8': ('mono8', 1),
    'RGB8': ('rgb8', 3),
    'BGR8': ('bgr8', 3),
}


class PeakCamNode(Node):
    """ROS 2 Python node that publishes images from an IDS peak camera."""

    def __init__(self):
        super().__init__('peak_cam_node')

        self._declare_and_get_params()

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)

        node_name = self.get_name()
        self._image_pub = self.create_publisher(
            Image,
            f'{node_name}/{self._image_topic}',
            qos)
        self._camera_info_pub = self.create_publisher(
            CameraInfo,
            f'{node_name}/camera_info',
            qos)

        if _CAMERA_INFO_MANAGER_AVAILABLE:
            self._camera_info_manager = CameraInfoManager(
                self, self._frame_id, self._camera_info_url)
            if self._camera_info_manager.validateURL(self._camera_info_url):
                self._camera_info_manager.loadCameraInfo(self._camera_info_url)
            else:
                self.get_logger().warn(
                    'The Provided Camera Info URL is invalid or file does not exist:')
                self.get_logger().warn(f'  {self._camera_info_url}')
                self.get_logger().warn(
                    'Uncalibrated Camera Info will be published...')
        else:
            self.get_logger().warn(
                '[PeakCamNode]: camera_info_manager Python package not found. '
                'Install ros-<distro>-camera-info-manager. '
                'Publishing empty CameraInfo.')
            self._camera_info_manager = None

        if not _IDS_PEAK_AVAILABLE:
            self.get_logger().error(
                '[PeakCamNode]: ids-peak Python packages are not installed.\n'
                '  pip install ids-peak ids-peak-ipl\n'
                'Note: the IDS peak SDK transport layers must also be installed '
                '(https://en.ids-imaging.com/ids-peak.html).')
            return

        self._device = None
        self._data_stream = None
        self._remote_nodemap = None
        self._acquisition_running = False
        self._acquisition_thread = None

        ids_peak.Library.Initialize()
        self._open_device()
        if self._acquisition_running:
            self._acquisition_thread = threading.Thread(
                target=self._acquisition_loop, daemon=True)
            self._acquisition_thread.start()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def destroy_node(self):
        self.get_logger().info('Shutting down')
        self._acquisition_running = False
        if self._data_stream:
            try:
                self._data_stream.KillWait()
            except Exception:
                pass
        if self._acquisition_thread and self._acquisition_thread.is_alive():
            self._acquisition_thread.join(timeout=5.0)
        if self._remote_nodemap:
            try:
                self._remote_nodemap.FindNode('AcquisitionStop').Execute()
                self._remote_nodemap.FindNode('AcquisitionStop').WaitUntilDone()
            except Exception as exc:
                self.get_logger().error(
                    f'[PeakCamNode]: Exception stopping acquisition: {exc}')
        self._close_device()
        if _IDS_PEAK_AVAILABLE:
            try:
                ids_peak.Library.Close()
            except Exception:
                pass
        self.get_logger().info('Peak library closed')
        super().destroy_node()

    # ------------------------------------------------------------------
    # Parameter handling
    # ------------------------------------------------------------------

    def _declare_and_get_params(self):
        self.declare_parameter('frame_id', 'peak_cam')
        self.declare_parameter('image_topic', 'image_raw')
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('selectedDevice', '000000')
        self.declare_parameter('ExposureTime', 100)
        self.declare_parameter('AcquisitionFrameRate', 1)
        self.declare_parameter('ImageHeight', 480)
        self.declare_parameter('ImageWidth', 640)
        self.declare_parameter('UseOffset', False)
        self.declare_parameter('OffsetHeight', 0)
        self.declare_parameter('OffsetWidth', 0)
        self.declare_parameter('Gamma', 1.2)
        self.declare_parameter('ExposureAuto', 'Off')
        self.declare_parameter('GainAuto', 'Off')
        self.declare_parameter('GainSelector', '')
        self.declare_parameter('PixelFormat', 'RGB8')
        self.declare_parameter('TriggerMode', 'Off')
        self.declare_parameter('TriggerSource', 0)

        self._frame_id = self.get_parameter('frame_id').value
        self._image_topic = self.get_parameter('image_topic').value
        self._camera_info_url = self.get_parameter('camera_info_url').value
        self._selected_device = self.get_parameter('selectedDevice').value
        self._exposure_time = self.get_parameter('ExposureTime').value
        self._acquisition_frame_rate = \
            self.get_parameter('AcquisitionFrameRate').value
        self._image_height = self.get_parameter('ImageHeight').value
        self._image_width = self.get_parameter('ImageWidth').value
        self._use_offset = self.get_parameter('UseOffset').value
        self._offset_height = self.get_parameter('OffsetHeight').value
        self._offset_width = self.get_parameter('OffsetWidth').value
        self._gamma = self.get_parameter('Gamma').value
        self._exposure_auto = self.get_parameter('ExposureAuto').value
        self._gain_auto = self.get_parameter('GainAuto').value
        self._gain_selector = self.get_parameter('GainSelector').value
        self._pixel_format = self.get_parameter('PixelFormat').value
        self._trigger_mode = self.get_parameter('TriggerMode').value
        self._trigger_source = self.get_parameter('TriggerSource').value

        self.get_logger().info('Setting parameters to:')
        self.get_logger().info(f'  frame_id: {self._frame_id}')
        self.get_logger().info(f'  image_topic: {self._image_topic}')
        self.get_logger().info(f'  camera_info_url: {self._camera_info_url}')
        self.get_logger().info(f'  ExposureTime: {self._exposure_time}')
        self.get_logger().info(
            f'  AcquisitionFrameRate: {self._acquisition_frame_rate}')
        self.get_logger().info(f'  Gamma: {self._gamma}')
        self.get_logger().info(f'  ImageHeight: {self._image_height}')
        self.get_logger().info(f'  ImageWidth: {self._image_width}')
        self.get_logger().info(f'  OffsetHeight: {self._offset_height}')
        self.get_logger().info(f'  OffsetWidth: {self._offset_width}')
        self.get_logger().info(f'  UseOffset: {self._use_offset}')
        self.get_logger().info(
            f'  selectedDevice: {self._selected_device}')
        self.get_logger().info(f'  ExposureAuto: {self._exposure_auto}')
        self.get_logger().info(f'  GainAuto: {self._gain_auto}')
        self.get_logger().info(f'  GainSelector: {self._gain_selector}')
        self.get_logger().info(f'  PixelFormat: {self._pixel_format}')
        self.get_logger().info(f'  TriggerMode: {self._trigger_mode}')
        self.get_logger().info(f'  TriggerSource: {self._trigger_source}')

    # ------------------------------------------------------------------
    # Device management
    # ------------------------------------------------------------------

    def _align_integer(self, name, value, node):
        """Clamp and snap *value* to the nearest valid increment for *node*."""
        min_val = node.Minimum()
        max_val = node.Maximum()
        increment = max(1, node.Increment())

        clamped = max(min_val, min(max_val, int(value)))
        delta = clamped - min_val
        aligned = min_val + (delta // increment) * increment

        upper = aligned + increment
        if upper <= max_val:
            if (upper - clamped) < (clamped - aligned):
                aligned = upper

        if aligned != value:
            self.get_logger().warn(
                f"[PeakCamNode]: '{name}' requested={value} adjusted to "
                f'{aligned} (min={min_val}, max={max_val}, inc={increment})')

        node.SetValue(int(aligned))
        return aligned

    def _open_device(self):
        device_manager = ids_peak.DeviceManager.Instance()

        while not self._acquisition_running:
            try:
                device_manager.Update()

                if device_manager.Devices().empty():
                    self.get_logger().info(
                        '[PeakCamNode]: No device found. Exiting program')
                    ids_peak.Library.Close()
                    return

                self.get_logger().info('[PeakCamNode]: Devices available:')
                for i, dev in enumerate(device_manager.Devices()):
                    self.get_logger().info(
                        f'  {i}: {dev.DisplayName()}')

                selected_idx = 0
                for i, dev in enumerate(device_manager.Devices()):
                    if self._selected_device == dev.SerialNumber():
                        selected_idx = i
                        break

                self._device = (
                    device_manager.Devices()[selected_idx]
                    .OpenDevice(ids_peak.DeviceAccessType_Control))
                self.get_logger().info(
                    f'[PeakCamNode]: {self._device.ModelName()} found')

                self._remote_nodemap = \
                    self._device.RemoteDevice().NodeMaps()[0]

                self._set_device_parameters()

                self._data_stream = \
                    self._device.DataStreams()[0].OpenDataStream()

                payload_size = \
                    self._remote_nodemap.FindNode('PayloadSize').Value()
                buf_count = \
                    self._data_stream.NumBuffersAnnouncedMinRequired()
                for _ in range(buf_count):
                    buf = self._data_stream.AllocAndAnnounceBuffer(
                        payload_size)
                    self._data_stream.QueueBuffer(buf)

                self._data_stream.StartAcquisition()
                self._remote_nodemap.FindNode('AcquisitionStart').Execute()
                self._remote_nodemap.FindNode(
                    'AcquisitionStart').WaitUntilDone()

                self.get_logger().info(
                    f'[PeakCamNode]: {self._device.ModelName()} connected')
                self._acquisition_running = True

            except Exception as exc:
                self.get_logger().error(
                    f'[PeakCamNode]: EXCEPTION: {exc}')
                self.get_logger().error(
                    f"[PeakCamNode]: Could not initialize device "
                    f"'{self._selected_device}'. Common causes are invalid "
                    "camera parameters (ROI/offset increments), unavailable "
                    "device, or insufficient permissions (udev/root).")
                break

    def _set_device_parameters(self):
        max_width = self._remote_nodemap.FindNode('WidthMax').Value()
        max_height = self._remote_nodemap.FindNode('HeightMax').Value()

        width_node = self._remote_nodemap.FindNode('Width')
        height_node = self._remote_nodemap.FindNode('Height')
        offset_x_node = self._remote_nodemap.FindNode('OffsetX')
        offset_y_node = self._remote_nodemap.FindNode('OffsetY')

        image_width = self._align_integer(
            'Width', self._image_width, width_node)
        self.get_logger().info(
            f"[PeakCamNode]: ImageWidth is set to '{image_width}'")
        image_height = self._align_integer(
            'Height', self._image_height, height_node)
        self.get_logger().info(
            f"[PeakCamNode]: ImageHeight is set to '{image_height}'")

        if self._use_offset:
            target_x = self._offset_width
            target_y = self._offset_height
        else:
            target_x = (max_width - image_width) // 2
            target_y = (max_height - image_height) // 2

        self._align_integer('OffsetX', target_x, offset_x_node)
        self._align_integer('OffsetY', target_y, offset_y_node)

        self._remote_nodemap.FindNode('GainAuto').SetCurrentEntry(
            self._gain_auto)
        self.get_logger().info(
            f"[PeakCamNode]: GainAuto is set to '{self._gain_auto}'")

        if self._gain_selector:
            self._remote_nodemap.FindNode('GainSelector').SetCurrentEntry(
                self._gain_selector)
            self.get_logger().info(
                f"[PeakCamNode]: GainSelector is set to "
                f"'{self._gain_selector}'")

        self._remote_nodemap.FindNode('ExposureAuto').SetCurrentEntry(
            self._exposure_auto)
        self.get_logger().info(
            f"[PeakCamNode]: ExposureAuto is set to '{self._exposure_auto}'")

        if self._exposure_auto == 'Off':
            self._remote_nodemap.FindNode('ExposureTime').SetValue(
                float(self._exposure_time))
            self.get_logger().info(
                f'[PeakCamNode]: ExposureTime is set to '
                f'{self._exposure_time} microseconds')

        self._remote_nodemap.FindNode('AcquisitionFrameRate').SetValue(
            float(self._acquisition_frame_rate))
        self.get_logger().info(
            f'[PeakCamNode]: AcquisitionFrameRate is set to '
            f'{self._acquisition_frame_rate} Hz')

        self._remote_nodemap.FindNode('Gamma').SetValue(float(self._gamma))
        self.get_logger().info(
            f'[PeakCamNode]: Gamma is set to {self._gamma}')

        self._remote_nodemap.FindNode('PixelFormat').SetCurrentEntry(
            self._pixel_format)
        self.get_logger().info(
            f"[PeakCamNode]: PixelFormat is set to '{self._pixel_format}'")

        if self._trigger_mode == 'On':
            nm = self._remote_nodemap
            nm.FindNode('TriggerSelector').SetCurrentEntry('ExposureStart')
            nm.FindNode('TriggerMode').SetCurrentEntry('On')
            nm.FindNode('TriggerSource').SetCurrentEntry('Timer0Active')
            nm.FindNode('TriggerActivation').SetCurrentEntry('LevelHigh')
            nm.FindNode('TimerSelector').SetCurrentEntry('Timer0')
            nm.FindNode('TimerDuration').SetValue(500000.0)
            line_in = f'Line{self._trigger_source}'
            nm.FindNode('TimerTriggerSource').SetCurrentEntry(line_in)
            nm.FindNode('TimerTriggerActivation').SetCurrentEntry(
                'RisingEdge')
        else:
            self.get_logger().info(
                '[PeakCamNode]: No Trigger Specified, running continuously')

        if self._pixel_format not in _PIXEL_FORMAT_MAP:
            raise RuntimeError(
                f"[PeakCamNode]: Unsupported PixelFormat parameter "
                f"'{self._pixel_format}'. Supported values: "
                + ', '.join(_PIXEL_FORMAT_MAP.keys()))

    def _close_device(self):
        if self._device:
            try:
                self._remote_nodemap.FindNode('AcquisitionStop').Execute()
                self.get_logger().info("Executing 'AcquisitionStop'")
            except Exception as exc:
                self.get_logger().error(f'EXCEPTION: {exc}')
        if self._data_stream:
            try:
                self._data_stream.KillWait()
                self._data_stream.StopAcquisition(
                    ids_peak.AcquisitionStopMode_Default)
                self._data_stream.Flush(
                    ids_peak.DataStreamFlushMode_DiscardAll)
                for buf in self._data_stream.AnnouncedBuffers():
                    self._data_stream.RevokeBuffer(buf)
                self.get_logger().info("'AcquisitionStop' Successful")
            except Exception as exc:
                self.get_logger().error(f'EXCEPTION: {exc}')

    # ------------------------------------------------------------------
    # Acquisition loop
    # ------------------------------------------------------------------

    def _acquisition_loop(self):
        self.get_logger().info('[PeakCamNode]: Acquisition started')

        while self._acquisition_running:
            try:
                buffer = self._data_stream.WaitForFinishedBuffer(5000)

                # Drop stale frames, keep only the latest
                dropped = 0
                while self._data_stream.NumBuffersAwaitDelivery() > 0:
                    self._data_stream.QueueBuffer(buffer)
                    buffer = self._data_stream.WaitForFinishedBuffer(0)
                    dropped += 1
                if dropped > 0:
                    self.get_logger().warn(
                        f'[PeakCamNode]: Dropped {dropped} stale frame(s) '
                        'to keep the latest image.')

                stamp = self.get_clock().now().to_msg()

                # Convert buffer to ids_peak_ipl Image
                ipl_image = ids_peak_ipl_extension.BufferToImage(buffer)

                # Determine target pixel format
                encoding, channels = _PIXEL_FORMAT_MAP.get(
                    self._pixel_format, ('rgb8', 3))
                target_pf_map = {
                    'Mono8': ids_peak_ipl.PixelFormatName_Mono8,
                    'RGB8': ids_peak_ipl.PixelFormatName_RGB8,
                    'BGR8': ids_peak_ipl.PixelFormatName_BGR8,
                }
                target_pf = target_pf_map[self._pixel_format]

                # Convert pixel format if necessary
                if ipl_image.PixelFormat().PixelFormatName() != target_pf:
                    converter = ids_peak_ipl.ImageConverter()
                    supported = converter.SupportedOutputPixelFormatNames(
                        ipl_image.PixelFormat())
                    if target_pf in supported:
                        ipl_image = ipl_image.ConvertTo(target_pf)
                    else:
                        # Fallback: BGR8 → RGB8 → Mono8
                        converted = False
                        for fallback_pf, fallback_enc, fallback_ch in [
                            (ids_peak_ipl.PixelFormatName_BGR8, 'bgr8', 3),
                            (ids_peak_ipl.PixelFormatName_RGB8, 'rgb8', 3),
                            (ids_peak_ipl.PixelFormatName_Mono8, 'mono8', 1),
                        ]:
                            if fallback_pf in supported:
                                ipl_image = ipl_image.ConvertTo(fallback_pf)
                                encoding, channels = fallback_enc, fallback_ch
                                converted = True
                                break
                        if not converted:
                            raise RuntimeError(
                                '[PeakCamNode]: No supported conversion path '
                                'to publish image data.')

                # Get image bytes as a numpy array (1-D, uint8)
                np_data = np.frombuffer(
                    ipl_image.get_numpy_1D(), dtype=np.uint8)

                # Build and publish Image message
                img_msg = Image()
                img_msg.header.stamp = stamp
                img_msg.header.frame_id = self._frame_id
                img_msg.height = ipl_image.Height()
                img_msg.width = ipl_image.Width()
                img_msg.encoding = encoding
                img_msg.is_bigendian = 0
                img_msg.step = ipl_image.Width() * channels
                img_msg.data = np_data.tobytes()
                self._image_pub.publish(img_msg)

                # Build and publish CameraInfo message
                if self._camera_info_manager is not None:
                    ci = self._camera_info_manager.getCameraInfo()
                    ci.header.stamp = stamp
                    ci.header.frame_id = self._frame_id
                else:
                    ci = CameraInfo()
                    ci.header.stamp = stamp
                    ci.header.frame_id = self._frame_id
                    ci.width = ipl_image.Width()
                    ci.height = ipl_image.Height()
                self._camera_info_pub.publish(ci)

                self._data_stream.QueueBuffer(buffer)

            except Exception as exc:
                self.get_logger().error(
                    f'[PeakCamNode]: EXCEPTION: {exc}')
                self.get_logger().error(
                    '[PeakCamNode]: Acquisition loop stopped, '
                    'device may be disconnected!')
                self.get_logger().error(
                    '[PeakCamNode]: Restart peak cam node!')
                self._acquisition_running = False


def main(args=None):
    rclpy.init(args=args)
    node = PeakCamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
