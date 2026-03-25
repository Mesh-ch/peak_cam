// Copyright (c) 2020, Sherif Nekkah
// All rights reserved.
//
// DISCLAMER:
//
//
// This package was created and used within an academic project and should
// be considered as experimental code. There may be bugs and deficiencies in the
// software. Feel free for suggestions, pull requests or any possible issue.
//
//
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include "peak_cam/peak_cam_node.hpp"
#include "image_transport/image_transport.hpp"
#include <algorithm>
#include <cstdint>
#include <stdexcept>

namespace peak_cam
{
namespace
{
int64_t setAlignedIntegerNodeValue(
  const rclcpp::Logger & logger,
  const std::string & nodeName,
  const std::shared_ptr<peak::core::nodes::IntegerNode> & node,
  int64_t requestedValue)
{
  const int64_t minValue = node->Minimum();
  const int64_t maxValue = node->Maximum();
  const int64_t increment = std::max<int64_t>(1, node->Increment());

  int64_t clampedValue = std::clamp(requestedValue, minValue, maxValue);
  const int64_t deltaFromMin = clampedValue - minValue;
  int64_t alignedValue = minValue + (deltaFromMin / increment) * increment;

  const int64_t upperCandidate = alignedValue + increment;
  if (upperCandidate <= maxValue)
  {
    const int64_t lowerDistance = clampedValue - alignedValue;
    const int64_t upperDistance = upperCandidate - clampedValue;
    if (upperDistance < lowerDistance)
    {
      alignedValue = upperCandidate;
    }
  }

  if (alignedValue != requestedValue)
  {
    RCLCPP_WARN_STREAM(
      logger,
      "[PeakCamNode]: '" << nodeName << "' requested=" << requestedValue <<
      " adjusted to " << alignedValue << " (min=" << minValue <<
      ", max=" << maxValue << ", inc=" << increment << ")");
  }

  node->SetValue(alignedValue);
  return alignedValue;
}
} // namespace

PeakCamNode::PeakCamNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("peak_cam_node", options)
{
  getParams();

  auto imageQos = rclcpp::QoS(rclcpp::KeepLast(1));
  imageQos.reliable();
  m_cameraPublisher = image_transport::create_camera_publisher(
    this,
    std::string(this->get_name()) + "/" + m_imageTopic,
    imageQos.get_rmw_qos_profile());
  
  // Initialize header messages
  m_header.reset(new std_msgs::msg::Header());
  m_header->frame_id = m_frameId;

  // Initialize Camera Info Manager
  m_cameraInfoManager =
    std::make_shared<camera_info_manager::CameraInfoManager>(
      this,
      m_frameId,
      m_cameraInfoUrl);

  m_cameraInfoManager->setCameraName(m_frameId);

  if(m_cameraInfoManager->validateURL(m_cameraInfoUrl)) {
    m_cameraInfoManager->loadCameraInfo(m_cameraInfoUrl);
  } else {
    RCLCPP_WARN(this->get_logger(), "The Provided Camera Info URL is invalid or file does not exist:");
    RCLCPP_WARN(this->get_logger(), "  %s", m_cameraInfoUrl.c_str());
    RCLCPP_WARN(this->get_logger(), "Uncalibrated Camera Info will be published...");
  }
  
  peak::Library::Initialize();
  openDevice();
  if (m_acquisitionLoopRunning) {
    m_acquisitionThread = std::thread(&PeakCamNode::acquisitionLoop, this);
  }
}

PeakCamNode::~PeakCamNode()
{
  RCLCPP_INFO(this->get_logger(), "Shutting down");
  m_acquisitionLoopRunning = false;
  if (m_dataStream) {
    m_dataStream->KillWait();
  }
  if (m_acquisitionThread.joinable()) {
    m_acquisitionThread.join();
  }
  if (m_nodeMapRemoteDevice) {
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop")->Execute();
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop")->WaitUntilDone();
  }
  // closing camera und peak library
  closeDevice();
  peak::Library::Close();
  RCLCPP_INFO(this->get_logger(), "Peak library closed");
}

void PeakCamNode::getParams()
{
  try {
    m_frameId = this->declare_parameter<std::string>("frame_id", "peak_cam");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The frame_id provided was invalid");
    throw ex;
  }
  
  try {
    m_imageTopic = this->declare_parameter<std::string>("image_topic", "image_raw");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The image_topic provided was invalid");
    throw ex;
  }

  try {
    m_cameraInfoUrl = this->declare_parameter<std::string>("camera_info_url", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The camera_info_url provided was invalid");
    throw ex;
  }

  try {
    m_peakParams.ExposureTime = this->declare_parameter<int>("ExposureTime", m_peakParams.ExposureTime);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The ExposureTime provided was invalid");
    throw ex;
  }
  
  try {
    m_peakParams.AcquisitionFrameRate =
      this->declare_parameter<int>("AcquisitionFrameRate", m_peakParams.AcquisitionFrameRate);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The AcquisitionFrameRate provided was invalid");
    throw ex;
  }
  
  try {
    m_peakParams.ImageHeight = this->declare_parameter<int>("ImageHeight", m_peakParams.ImageHeight);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The ImageHeight provided was invalid");
    throw ex;
  }
  
  try {
    m_peakParams.ImageWidth = this->declare_parameter<int>("ImageWidth", m_peakParams.ImageWidth);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The ImageWidth provided was invalid");
    throw ex;
  }
  
  try {
    m_peakParams.UseOffset = this->declare_parameter<bool>("UseOffset", m_peakParams.UseOffset);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The UseOffset provided was invalid");
    throw ex;
  }
  
  try {
    m_peakParams.OffsetHeight = this->declare_parameter<int>("OffsetHeight", m_peakParams.OffsetHeight);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The OffsetHeight provided was invalid");
    throw ex;
  }
  
  try {
    m_peakParams.OffsetWidth = this->declare_parameter<int>("OffsetWidth", m_peakParams.OffsetWidth);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The OffsetWidth provided was invalid");
    throw ex;
  }
  
  try {
    m_peakParams.Gamma = this->declare_parameter<double>("Gamma", m_peakParams.Gamma);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The Gamma provided was invalid");
    throw ex;
  }

  try {
    m_peakParams.selectedDevice =
      this->declare_parameter<std::string>("selectedDevice", m_peakParams.selectedDevice);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The selectedDevice provided was invalid");
    throw ex;
  }

  try {
    m_peakParams.ExposureAuto =
      this->declare_parameter<std::string>("ExposureAuto", m_peakParams.ExposureAuto);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The ExposureAuto provided was invalid");
    throw ex;
  }

  try {
    m_peakParams.GainAuto = this->declare_parameter<std::string>("GainAuto", m_peakParams.GainAuto);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The GainAuto provided was invalid");
    throw ex;
  }
  
  try {
    m_peakParams.PixelFormat = this->declare_parameter<std::string>("PixelFormat", m_peakParams.PixelFormat);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The PixelFormat provided was invalid");
    throw ex;
  }
  
  try {
    m_peakParams.GainSelector =
      this->declare_parameter<std::string>("GainSelector", m_peakParams.GainSelector);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The GainSelector provided was invalid");
    throw ex;
  }

  RCLCPP_INFO(this->get_logger(), "Setting parameters to:");
  RCLCPP_INFO(this->get_logger(), "  frame_id: %s", m_frameId.c_str());
  RCLCPP_INFO(this->get_logger(), "  image_topic: %s", m_imageTopic.c_str());
  RCLCPP_INFO(this->get_logger(), "  camera_info_url: %s", m_cameraInfoUrl.c_str());
  RCLCPP_INFO(this->get_logger(), "  ExposureTime: %i", m_peakParams.ExposureTime);
  RCLCPP_INFO(this->get_logger(), "  AcquisitionFrameRate: %i", m_peakParams.AcquisitionFrameRate);
  RCLCPP_INFO(this->get_logger(), "  Gamma: %f", m_peakParams.Gamma);
  RCLCPP_INFO(this->get_logger(), "  ImageHeight: %i", m_peakParams.ImageHeight);
  RCLCPP_INFO(this->get_logger(), "  ImageWidth: %i", m_peakParams.ImageWidth);
  RCLCPP_INFO(this->get_logger(), "  OffsetHeight: %i", m_peakParams.OffsetHeight);
  RCLCPP_INFO(this->get_logger(), "  OffsetWidth: %i", m_peakParams.OffsetWidth);
  RCLCPP_INFO(this->get_logger(), "  UseOffset: %i", m_peakParams.UseOffset);
  RCLCPP_INFO(this->get_logger(), "  selectedDevice: %s", m_peakParams.selectedDevice.c_str());
  RCLCPP_INFO(this->get_logger(), "  ExposureAuto: %s", m_peakParams.ExposureAuto.c_str());
  RCLCPP_INFO(this->get_logger(), "  GainAuto: %s", m_peakParams.GainAuto.c_str());
  RCLCPP_INFO(this->get_logger(), "  GainSelector: %s", m_peakParams.GainSelector.c_str());
  RCLCPP_INFO(this->get_logger(), "  PixelFormat: %s", m_peakParams.PixelFormat.c_str());
  RCLCPP_INFO(this->get_logger(), "  TriggerMode: %s", m_peakParams.TriggerMode.c_str());
  RCLCPP_INFO(this->get_logger(), "  TriggerSource: %i", m_peakParams.TriggerSource);
}

void PeakCamNode::openDevice()
{
  auto& deviceManager = peak::DeviceManager::Instance();  
  //Select Device and set Parameters Once
  while (!m_acquisitionLoopRunning)
  {
    try
    {
      // update the device manager
      deviceManager.Update();
     
      // exit program if no device was found
      if (deviceManager.Devices().empty())
      {
        RCLCPP_INFO(this->get_logger(), "No device found. Exiting program");
        // close library before exiting program
        peak::Library::Close();
        return;
      }
      
      // list all available devices
      size_t i = 0;
      RCLCPP_INFO_ONCE(this->get_logger(), "Devices available: ");
      for (const auto& deviceDescriptor : deviceManager.Devices())
      {
        RCLCPP_INFO(this->get_logger(), "  %lu: %s", i, deviceDescriptor->DisplayName().c_str());
        ++i;
      }
        
      // set i back to 0
      i = 0;
      size_t selectedDevice = 0;
      for (const auto& deviceDescriptor : deviceManager.Devices())
      {
        if (m_peakParams.selectedDevice == deviceDescriptor->SerialNumber())
        {
          RCLCPP_INFO_ONCE(this->get_logger(), "SELECTING NEW DEVICE: %lu", i);
          selectedDevice = i;
        }
        ++i;
      }
   
      // get vector of device descriptors
      auto deviceDesrciptors = deviceManager.Devices();
      // open the selected device
      m_device =
        deviceManager.Devices().at(selectedDevice)->OpenDevice(
          peak::core::DeviceAccessType::Control);
      RCLCPP_INFO_STREAM(
        this->get_logger(), "[PeakCamNode]: " << m_device->ModelName() << " found");
      // get the remote device node map
      m_nodeMapRemoteDevice = m_device->RemoteDevice()->NodeMaps().at(0); 
      std::vector<std::shared_ptr<peak::core::nodes::Node>> nodes = m_nodeMapRemoteDevice->Nodes();
      // sets Acquisition Parameters of the camera -> see yaml
      setDeviceParameters();
      // open the first data stream
      m_dataStream = m_device->DataStreams().at(0)->OpenDataStream(); 
      // get payload size
      auto payloadSize =
        m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("PayloadSize")->Value();
      
      // get number of buffers to allocate the buffer count depends on your application
      // here the minimum required number for the data stream
      auto bufferCountMax = m_dataStream->NumBuffersAnnouncedMinRequired();
      
      // allocate and announce image buffers and queue them
      for (uint64_t bufferCount = 0; bufferCount < bufferCountMax; ++bufferCount)
      {
          auto buffer = m_dataStream->AllocAndAnnounceBuffer(static_cast<size_t>(payloadSize), nullptr);
          m_dataStream->QueueBuffer(buffer);
      }
      // Lock critical features to prevent them from changing during acqusition
      // m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("TLParamsLocked")->SetValue(1);

      // start the data stream
      m_dataStream->StartAcquisition();
      // start the device
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>(
        "AcquisitionStart")->Execute();
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>(
        "AcquisitionStart")->WaitUntilDone();
      RCLCPP_INFO_STREAM(
        this->get_logger(), "[PeakCamNode]: " << m_device->ModelName() << " connected");
      m_acquisitionLoopRunning = true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR_STREAM_ONCE(
        this->get_logger(), "[PeakCamNode]: EXCEPTION: " << e.what());
      RCLCPP_ERROR_STREAM_ONCE(
        this->get_logger(),
        "[PeakCamNode]: Could not initialize device '" << m_peakParams.selectedDevice <<
        "'. Common causes are invalid camera parameters (ROI/offset increments), unavailable device, "
        "or insufficient permissions (udev/root).");
    }
  }
}

void PeakCamNode::setDeviceParameters()
{
  int64_t maxWidth, maxHeight = 0;
  maxWidth = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("WidthMax")->Value();
  // RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: maxWidth '" << maxWidth << "'");
  maxHeight = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("HeightMax")->Value();
  // RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: maxHeight '" << maxHeight << "'");
  const auto widthNode = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Width");
  const auto heightNode = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("Height");
  const auto offsetXNode = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetX");
  const auto offsetYNode = m_nodeMapRemoteDevice->FindNode<peak::core::nodes::IntegerNode>("OffsetY");

  const int64_t imageWidth = setAlignedIntegerNodeValue(
    this->get_logger(), "Width", widthNode, m_peakParams.ImageWidth);
  RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: ImageWidth is set to '" << imageWidth << "'");
  const int64_t imageHeight = setAlignedIntegerNodeValue(
    this->get_logger(), "Height", heightNode, m_peakParams.ImageHeight);
  RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: ImageHeight is set to '" << imageHeight << "'");
  
  int64_t targetOffsetX = 0;
  int64_t targetOffsetY = 0;
  if (m_peakParams.UseOffset) {
    targetOffsetX = m_peakParams.OffsetWidth;
    targetOffsetY = m_peakParams.OffsetHeight;
  } else {
    // auto-center if UseOffset is set to False
    targetOffsetX = (maxWidth - imageWidth) / 2;
    targetOffsetY = (maxHeight - imageHeight) / 2;
  }
  setAlignedIntegerNodeValue(this->get_logger(), "OffsetX", offsetXNode, targetOffsetX);
  setAlignedIntegerNodeValue(this->get_logger(), "OffsetY", offsetYNode, targetOffsetY);

  //Set GainAuto Parameter
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainAuto")->SetCurrentEntry(m_peakParams.GainAuto);
  RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: GainAuto is set to '" << m_peakParams.GainAuto << "'");
  //Set GainSelector Parameter
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("GainSelector")->SetCurrentEntry(m_peakParams.GainSelector);
  RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: GainSelector is set to '" << m_peakParams.GainSelector << "'");
  //Set ExposureAuto Parameter
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("ExposureAuto")->SetCurrentEntry(m_peakParams.ExposureAuto);
  RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: ExposureAuto is set to '" << m_peakParams.ExposureAuto << "'");

  //Set ExposureTime Parameter
  if(m_peakParams.ExposureAuto == "Off")
  {
    m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("ExposureTime")->SetValue(m_peakParams.ExposureTime);
    RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: ExposureTime is set to " << m_peakParams.ExposureTime << " microseconds");
  }
  //Set AcquisitionFrameRate Parameter
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("AcquisitionFrameRate")->SetValue(m_peakParams.AcquisitionFrameRate);
  RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: AcquisitionFrameRate is set to " << m_peakParams.AcquisitionFrameRate << " Hz");
  //Set Gamma Parameter
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("Gamma")->SetValue(m_peakParams.Gamma);
  RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: Gamma is set to " << m_peakParams.Gamma);
  //Set PixelFormat Parameter
  m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("PixelFormat")->SetCurrentEntry(m_peakParams.PixelFormat);
  RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode]: PixelFormat is set to '" << m_peakParams.PixelFormat << "'");

  // Set TriggerMode
  if (m_peakParams.TriggerMode == "On" ) {
    // TODO(flynneva): add more parameters for customizing trigger
    // trigger acqusition, delayed trigger, etc.
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")
      ->SetCurrentEntry("ExposureStart");
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerMode")->SetCurrentEntry("On");
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSource")
      ->SetCurrentEntry("Timer0Active");
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerActivation")
      ->SetCurrentEntry("LevelHigh");
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TimerSelector")
      ->SetCurrentEntry("Timer0");
     m_nodeMapRemoteDevice->FindNode<peak::core::nodes::FloatNode>("TimerDuration")->SetValue(500000.0);
    std::string triggerTypeStart = "Timer0";
    // set hardline trigger settings
    std::string lineIn = "Line";
    lineIn.push_back(m_peakParams.TriggerSource);  // GPIO pin number

    // already including some future logic for other trigger types
    if (triggerTypeStart == "Counter0")
    {
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("CounterSelector")
       ->SetCurrentEntry("Counter0");
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("CounterTriggerSource")
       ->SetCurrentEntry(lineIn);
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("CounterTriggerActivation")
       ->SetCurrentEntry("RisingEdge");
    }
    else if (triggerTypeStart == "Timer0")
    {
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TimerSelector")
       ->SetCurrentEntry("Timer0");
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TimerTriggerSource")
       ->SetCurrentEntry(lineIn);
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TimerTriggerActivation")
       ->SetCurrentEntry("RisingEdge");
    }
    else
    {
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSelector")
       ->SetCurrentEntry(triggerTypeStart);
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerSource")
       ->SetCurrentEntry(lineIn);
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::EnumerationNode>("TriggerActivation")
       ->SetCurrentEntry("RisingEdge");
    }
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "[PeakCamNode] No Trigger Specified, running continously");
  }
    
  // Set Parameters for ROS Image
  if (m_peakParams.PixelFormat == "Mono8") {
    m_pixelFormat = peak::ipl::PixelFormatName::Mono8;
    m_image_encoding = sensor_msgs::image_encodings::MONO8;
    m_bytesPerPixel = 1;
  } else if (m_peakParams.PixelFormat == "RGB8") {
    m_pixelFormat = peak::ipl::PixelFormatName::RGB8;
    m_image_encoding = sensor_msgs::image_encodings::RGB8;
    m_bytesPerPixel = 3;
  } else if (m_peakParams.PixelFormat == "BGR8") {
    m_pixelFormat = peak::ipl::PixelFormatName::BGR8;
    m_image_encoding = sensor_msgs::image_encodings::BGR8;
    m_bytesPerPixel = 3;
  } else {
    throw std::runtime_error(
      "[PeakCamNode]: Unsupported PixelFormat parameter '" + m_peakParams.PixelFormat +
      "'. Supported values in this node are Mono8, RGB8, BGR8.");
  }
}

void PeakCamNode::acquisitionLoop()
{
  while (m_acquisitionLoopRunning) {
    try {
      RCLCPP_INFO_ONCE(this->get_logger(), "[PeakCamNode]: Acquisition started");
      // get buffer from data stream and process it
      auto buffer = m_dataStream->WaitForFinishedBuffer(5000);
      size_t droppedBuffers = 0;
      while (m_dataStream->NumBuffersAwaitDelivery() > 0) {
        m_dataStream->QueueBuffer(buffer);
        buffer = m_dataStream->WaitForFinishedBuffer(0);
        ++droppedBuffers;
      }
      if (droppedBuffers > 0) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(),
          *this->get_clock(),
          2000,
          "[PeakCamNode]: Dropped %zu stale frame(s) to keep the latest image.",
          droppedBuffers);
      }
      m_header->stamp = this->now();

      
      auto ci = m_cameraInfoManager->getCameraInfo();
      m_cameraInfo.reset(new sensor_msgs::msg::CameraInfo(ci));
      m_cameraInfo->header = *m_header;

      // buffer processing start
      auto inputImage = peak::BufferTo<peak::ipl::Image>(buffer);
      auto image = inputImage;
      auto publishEncoding = m_image_encoding;
      auto publishBytesPerPixel = m_bytesPerPixel;
      const auto inputPixelFormat = inputImage.PixelFormat();
      const auto inputPixelFormatName = inputPixelFormat.PixelFormatName();

      if (inputPixelFormatName != m_pixelFormat)
      {
        peak::ipl::ImageConverter imageConverter;
        const auto supportedOutputFormats =
          imageConverter.SupportedOutputPixelFormatNames(inputPixelFormat);

        const auto supportsFormat =
          [&](peak::ipl::PixelFormatName outputFormat)
          {
            return std::find(
              supportedOutputFormats.begin(), supportedOutputFormats.end(), outputFormat) !=
                   supportedOutputFormats.end();
          };

        const auto convertIfSupported =
          [&](peak::ipl::PixelFormatName outputFormat, const std::string & outputEncoding, uint8_t bytesPerPixel)
          {
            if (!supportsFormat(outputFormat))
            {
              return false;
            }
            image = inputImage.ConvertTo(outputFormat);
            publishEncoding = outputEncoding;
            publishBytesPerPixel = bytesPerPixel;
            return true;
          };

        if (!convertIfSupported(m_pixelFormat, m_image_encoding, m_bytesPerPixel))
        {
          std::string supportedFormatsText;
          for (size_t i = 0; i < supportedOutputFormats.size(); ++i)
          {
            if (i > 0)
            {
              supportedFormatsText += ", ";
            }
            supportedFormatsText += peak::ipl::ToString(supportedOutputFormats[i]);
          }

          RCLCPP_WARN_STREAM_ONCE(
            this->get_logger(),
            "[PeakCamNode]: Requested conversion from " << peak::ipl::ToString(inputPixelFormatName) <<
            " to " << peak::ipl::ToString(m_pixelFormat) <<
            " is not supported. Supported outputs are: [" << supportedFormatsText <<
            "]. Trying fallback formats.");

          if (!convertIfSupported(
              peak::ipl::PixelFormatName::BGR8,
              sensor_msgs::image_encodings::BGR8,
              3) &&
            !convertIfSupported(
              peak::ipl::PixelFormatName::RGB8,
              sensor_msgs::image_encodings::RGB8,
              3) &&
            !convertIfSupported(
              peak::ipl::PixelFormatName::Mono8,
              sensor_msgs::image_encodings::MONO8,
              1))
          {
            throw std::runtime_error(
              "[PeakCamNode]: No supported conversion path to publish image data.");
          }
        }
      }

      const auto imageBufferSize = image.Width() * image.Height() * publishBytesPerPixel;
      cv::Mat cvImage;
      if (publishEncoding == sensor_msgs::image_encodings::MONO8)
        cvImage = cv::Mat::zeros(image.Height(), image.Width(), CV_8UC1);
      else
        cvImage = cv::Mat::zeros(image.Height(), image.Width(), CV_8UC3);
      int sizeBuffer = static_cast<int>(image.ByteCount());
      if (static_cast<size_t>(sizeBuffer) != imageBufferSize)
      {
        RCLCPP_WARN_STREAM_ONCE(
          this->get_logger(),
          "[PeakCamNode]: Byte count mismatch. expected=" << imageBufferSize <<
          " actual=" << sizeBuffer << " for encoding " << publishEncoding);
      }
      // Device buffer is being copied into cv_bridge format
      std::memcpy(cvImage.data, image.Data(), static_cast<size_t>(sizeBuffer));
      // cv_bridge Image is converted to sensor_msgs/Image to publish on ROS Topic
      RCLCPP_INFO_ONCE(this->get_logger(), "[PeakCamNode]: cv bridge image");
      m_cvImage.reset(new cv_bridge::CvImage());
      m_cvImage->header = *m_header;
      m_cvImage->encoding = publishEncoding;
      m_cvImage->image = cvImage;
      m_cameraPublisher.publish(*m_cvImage->toImageMsg(), *m_cameraInfo);
      RCLCPP_INFO_STREAM_ONCE(this->get_logger(), "[PeakCamNode]: Publishing data");
      // queue buffer
      m_dataStream->QueueBuffer(buffer);
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "[PeakCamNode]: EXCEPTION: " << e.what());
      RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]: Acquisition loop stopped, device may be disconnected!");
      RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]: No device reset available");
      RCLCPP_ERROR(this->get_logger(), "[PeakCamNode]: Restart peak cam node!");
      m_acquisitionLoopRunning = false;
    }
  }
}

void PeakCamNode::closeDevice()
{
  // if device was opened, try to stop acquisition
  if (m_device) {
    try {
      m_nodeMapRemoteDevice->FindNode<peak::core::nodes::CommandNode>("AcquisitionStop")->Execute();
      RCLCPP_INFO(this->get_logger(), "Executing 'AcquisitionStop'");
      m_acquisitionLoopRunning = false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "EXCEPTION: " << e.what());
    }
  }
  // if data stream was opened, try to stop it and revoke its image buffers
  if (m_dataStream) {
    try {
      m_dataStream->KillWait(); //->KillOneWait();
      m_dataStream->StopAcquisition(peak::core::AcquisitionStopMode::Default);
      m_dataStream->Flush(peak::core::DataStreamFlushMode::DiscardAll);
      for (const auto &buffer : m_dataStream->AnnouncedBuffers()) {
        m_dataStream->RevokeBuffer(buffer);
      }
      RCLCPP_INFO(this->get_logger(), "'AcquisitionStop' Succesful");
      m_acquisitionLoopRunning = false;
    } catch (const std::exception &e) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "EXCEPTION: " << e.what());
    }
  }
}
} // namespace peak_cam

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(peak_cam::PeakCamNode)
