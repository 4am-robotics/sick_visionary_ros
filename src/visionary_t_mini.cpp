//
// Copyright (c) 2023 SICK AG, Waldkirch
//
// SPDX-License-Identifier: Unlicense

#include <boost/thread.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <memory>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>

#include "VisionaryControl.h"
#include "VisionaryDataStream.h"
#include "VisionaryTMiniData.h" // Header specific for the Time of Flight data

using namespace visionary;

std::shared_ptr<rclcpp::Node> gNode;

std::shared_ptr<VisionaryControl> gControl;

std::shared_ptr<VisionaryTMiniData> gDataHandler;

image_transport::Publisher                                  gPubDepth, gPubIntensity, gPubStatemap;
rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr  gPubCameraInfo;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr gPubPoints;

std::shared_ptr<diagnostic_updater::Updater>         updater;
std::shared_ptr<diagnostic_updater::TopicDiagnostic> gPubDepth_freq, gPubIntensity_freq, gPubStatemap_freq;
std::shared_ptr<diagnostic_updater::TopicDiagnostic> gPubCameraInfo_freq, gPubPoints_freq;

std::string gFrameId;
std::string gDeviceIdent;
bool        gEnableDepth, gEnableIntensity, gEnableStatemap, gEnablePoints;

boost::mutex gDataMtx;
bool         gReceive = true;

int gNumSubs = 0;

void diag_timer_cb()
{
  updater->force_update();
}

void driver_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "driver running");
  stat.add("frame_id", gFrameId);
  stat.add("device_ident", gDeviceIdent);
  stat.add("NumSubscribers_CameraInfo", gPubCameraInfo->get_subscription_count());
  stat.add("gEnablePoints", gEnablePoints);
  if (gEnablePoints)
    stat.add("NumSubscribers_Points", gPubPoints->get_subscription_count());
  stat.add("gEnableDepth", gEnableDepth);
  if (gEnableDepth)
    stat.add("NumSubscribers_Depth", gPubDepth.getNumSubscribers());
  stat.add("gEnableIntensity", gEnableIntensity);
  if (gEnableIntensity)
    stat.add("NumSubscribers_Intensity", gPubIntensity.getNumSubscribers());
  stat.add("gEnableStatemap", gEnableStatemap);
  if (gEnableStatemap)
    stat.add("NumSubscribers_Statemap", gPubStatemap.getNumSubscribers());
}

void publishCameraInfo(std_msgs::msg::Header header, VisionaryTMiniData& dataHandler)
{
  sensor_msgs::msg::CameraInfo ci;
  ci.header = header;

  ci.height = dataHandler.getHeight();
  ci.width  = dataHandler.getWidth();

  ci.d.clear();
  ci.d.resize(5, 0);
  ci.d[0] = dataHandler.getCameraParameters().k1;
  ci.d[1] = dataHandler.getCameraParameters().k2;
  ci.d[2] = dataHandler.getCameraParameters().p1;
  ci.d[3] = dataHandler.getCameraParameters().p2;
  ci.d[4] = dataHandler.getCameraParameters().k3;

  for (int i = 0; i < 9; i++)
  {
    ci.k[i] = 0;
  }
  ci.k[0] = dataHandler.getCameraParameters().fx;
  ci.k[4] = dataHandler.getCameraParameters().fy;
  ci.k[2] = dataHandler.getCameraParameters().cx;
  ci.k[5] = dataHandler.getCameraParameters().cy;
  ci.k[8] = 1;

  for (int i = 0; i < 12; i++)
    ci.p[i] = 0; // data.getCameraParameters().cam2worldMatrix[i];
  // TODO:....
  ci.p[0]  = dataHandler.getCameraParameters().fx;
  ci.p[5]  = dataHandler.getCameraParameters().fy;
  ci.p[10] = 1;
  ci.p[2]  = dataHandler.getCameraParameters().cx;
  ci.p[6]  = dataHandler.getCameraParameters().cy;

  gPubCameraInfo->publish(ci);
}

void publishDepth(std_msgs::msg::Header header, VisionaryTMiniData& dataHandler)
{
  std::vector<uint16_t> vec = dataHandler.getDistanceMap();
  cv::Mat               m   = cv::Mat(dataHandler.getHeight(), dataHandler.getWidth(), CV_16UC1);
  memcpy(m.data, vec.data(), vec.size() * sizeof(uint16_t));
  sensor_msgs::msg::Image::SharedPtr msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, m).toImageMsg();

  msg->header = header;
  gPubDepth.publish(msg);
}

void publishIntensity(std_msgs::msg::Header header, VisionaryTMiniData& dataHandler)
{
  std::vector<uint16_t> vec = dataHandler.getIntensityMap();
  cv::Mat               m   = cv::Mat(dataHandler.getHeight(), dataHandler.getWidth(), CV_16UC1);
  memcpy(m.data, vec.data(), vec.size() * sizeof(uint16_t));
  sensor_msgs::msg::Image::SharedPtr msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, m).toImageMsg();

  msg->header = header;
  gPubIntensity.publish(msg);
}

void publishStateMap(std_msgs::msg::Header header, VisionaryTMiniData& dataHandler)
{
  std::vector<uint16_t> vec = dataHandler.getStateMap();
  cv::Mat               m   = cv::Mat(dataHandler.getHeight(), dataHandler.getWidth(), CV_16UC1);
  memcpy(m.data, vec.data(), vec.size() * sizeof(uint16_t));
  sensor_msgs::msg::Image::SharedPtr msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::TYPE_16UC1, m).toImageMsg();

  msg->header = header;
  gPubStatemap.publish(msg);
}

void publishPointCloud(std_msgs::msg::Header header, VisionaryTMiniData& dataHandler)
{
  typedef sensor_msgs::msg::PointCloud2 PointCloud;

  // Allocate new point cloud message
  PointCloud::SharedPtr cloudMsg(new PointCloud);
  cloudMsg->header       = header;
  cloudMsg->height       = dataHandler.getHeight();
  cloudMsg->width        = dataHandler.getWidth();
  cloudMsg->is_dense     = false;
  cloudMsg->is_bigendian = false;

  cloudMsg->fields.resize(5);
  cloudMsg->fields[0].name = "x";
  cloudMsg->fields[1].name = "y";
  cloudMsg->fields[2].name = "z";
  cloudMsg->fields[3].name = "intensity";
  int offset               = 0;
  for (size_t d = 0; d < 3; ++d, offset += sizeof(float))
  {
    cloudMsg->fields[d].offset   = offset;
    cloudMsg->fields[d].datatype = int(sensor_msgs::msg::PointField::FLOAT32);
    cloudMsg->fields[d].count    = 1;
  }

  cloudMsg->fields[3].offset   = offset;
  cloudMsg->fields[3].datatype = int(sensor_msgs::msg::PointField::UINT16);
  cloudMsg->fields[3].count    = 1;
  offset += sizeof(uint16_t);

  cloudMsg->point_step = offset;
  cloudMsg->row_step   = cloudMsg->point_step * cloudMsg->width;
  cloudMsg->data.resize(cloudMsg->height * cloudMsg->row_step);

  std::vector<PointXYZ> pointCloud;
  dataHandler.generatePointCloud(pointCloud);
  dataHandler.transformPointCloud(pointCloud);

  // simple copy to create a XYZ point cloud
  // memcpy(&cloud_msg->data[0], &pointCloud[0], pointCloud.size()*sizeof(PointXYZ));

  std::vector<uint16_t>::const_iterator itIntens  = dataHandler.getIntensityMap().begin();
  std::vector<PointXYZ>::const_iterator itPC      = pointCloud.begin();
  size_t                                cloudSize = dataHandler.getHeight() * dataHandler.getWidth();
  for (size_t index = 0; index < cloudSize; ++index, ++itIntens, ++itPC)
  {
    memcpy(&cloudMsg->data[index * cloudMsg->point_step + cloudMsg->fields[0].offset], &*itPC, sizeof(PointXYZ));
    memcpy(&cloudMsg->data[index * cloudMsg->point_step + cloudMsg->fields[3].offset], &*itIntens, sizeof(uint16_t));
  }
  gPubPoints->publish(*cloudMsg);
}

void publish_frame(VisionaryTMiniData& dataHandler)
{
  bool publishedAnything = false;

  std_msgs::msg::Header header;
  header.stamp    = gNode->now();
  header.frame_id = gFrameId;

  if (gPubCameraInfo->get_subscription_count() > 0)
  {
    publishedAnything = true;
    publishCameraInfo(header, dataHandler);
    // gPubCameraInfo_freq->tick(header.stamp);
  }
  if (gEnableDepth && gPubDepth.getNumSubscribers() > 0)
  {
    publishedAnything = true;
    publishDepth(header, dataHandler);
    // gPubDepth_freq->tick(header.stamp);
  }
  if (gEnableIntensity && gPubIntensity.getNumSubscribers() > 0)
  {
    publishedAnything = true;
    publishIntensity(header, dataHandler);
    // gPubIntensity_freq->tick(header.stamp);
  }
  if (gEnableStatemap && gPubStatemap.getNumSubscribers() > 0)
  {
    publishedAnything = true;
    publishStateMap(header, dataHandler);
    // gPubStatemap_freq->tick(header.stamp);
  }
  if (gEnablePoints && gPubPoints->get_subscription_count() > 0)
  {
    publishedAnything = true;
    publishPointCloud(header, dataHandler);
    // gPubPoints_freq->tick(header.stamp);
  }

  if (publishedAnything)
  {
    gPubCameraInfo_freq->tick(header.stamp);
    if (gEnableDepth)
      gPubDepth_freq->tick(header.stamp);
    if (gEnableIntensity)
      gPubIntensity_freq->tick(header.stamp);
    if (gEnableStatemap)
      gPubStatemap_freq->tick(header.stamp);
    if (gEnablePoints)
      gPubPoints_freq->tick(header.stamp);
  }
  else
  {
    RCLCPP_DEBUG(gNode->get_logger(), "Nothing published");
  }
}

void thr_publish_frame()
{
  gDataMtx.lock();
  publish_frame(*gDataHandler);
  gDataMtx.unlock();
}

void thr_receive_frame(std::shared_ptr<VisionaryDataStream> pDataStream,
                       std::shared_ptr<VisionaryTMiniData>  pDataHandler)
{
  while (gReceive)
  {
    if (!pDataStream->getNextFrame())
    {
      continue; // No valid frame received
    }
    if (gDataMtx.try_lock())
    {
      gDataHandler = pDataHandler;
      gDataMtx.unlock();
      boost::thread thr(&thr_publish_frame);
    }
    else
      RCLCPP_INFO(gNode->get_logger(), "skipping frame with number %d", pDataHandler->getFrameNum());
  }
}

void _on_new_subscriber()
{
  gNumSubs++;
  RCLCPP_DEBUG(gNode->get_logger(), "Got new subscriber, total amount of subscribers: %d", gNumSubs);
  if (gControl)
    gControl->startAcquisition();
}

void _on_subscriber_disconnected()
{
  gNumSubs--;
  RCLCPP_DEBUG(gNode->get_logger(), "Subscriber disconnected, total amount of subscribers: %d", gNumSubs);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  gNode = std::make_shared<rclcpp::Node>("sick_visionary_t_mini");

  // default parameters
  std::string remoteDeviceIp = "192.168.1.10";
  gFrameId                   = "camera";

  gNode->declare_parameter<std::string>("remote_device_ip", remoteDeviceIp);
  gNode->declare_parameter<std::string>("frame_id", gFrameId);
  gNode->declare_parameter<bool>("enable_depth", true);
  gNode->declare_parameter<bool>("enable_intensity", true);
  gNode->declare_parameter<bool>("enable_statemap", true);
  gNode->declare_parameter<bool>("enable_points", true);
  gNode->declare_parameter<double>("desired_frequency", 15.0);

  gNode->get_parameter("remote_device_ip", remoteDeviceIp);
  gNode->get_parameter("frame_id", gFrameId);
  gNode->get_parameter("enable_depth", gEnableDepth);
  gNode->get_parameter("enable_intensity", gEnableIntensity);
  gNode->get_parameter("enable_statemap", gEnableStatemap);
  gNode->get_parameter("enable_points", gEnablePoints);

  std::shared_ptr<VisionaryTMiniData>  pDataHandler = std::make_shared<VisionaryTMiniData>();
  std::shared_ptr<VisionaryDataStream> pDataStream  = std::make_shared<VisionaryDataStream>(pDataHandler);
  gControl                                          = std::make_shared<VisionaryControl>();

  RCLCPP_INFO(gNode->get_logger(), "Connecting to device at %s", remoteDeviceIp.c_str());
  if (!gControl->open(VisionaryControl::ProtocolType::COLA_2, remoteDeviceIp.c_str(), 5000 /*ms*/))
  {
    RCLCPP_ERROR(gNode->get_logger(), "Connection with devices control channel failed");
    return -1;
  }
  // Tell the device to send blob data on the data channel.
  if (!gControl->getDataStreamConfig())
  {
    RCLCPP_ERROR(gNode->get_logger(), "Failed to configure data stream on device");
    return -1;
  }
  // To be sure the acquisition is currently stopped.
  gControl->stopAcquisition();

  if (!pDataStream->open(remoteDeviceIp.c_str(), 2114u))
  {
    RCLCPP_ERROR(gNode->get_logger(), "Connection with devices data channel failed");
    return -1;
  }

  // TODO: add get device name and device version and print to ros info.
  RCLCPP_INFO(gNode->get_logger(), "Connected with Visionary-T Mini");

  // make me public (after init.)
  image_transport::ImageTransport it(gNode);
  gPubCameraInfo = gNode->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);
  if (gEnableDepth)
    gPubDepth = it.advertise("depth", 1);
  if (gEnablePoints)
    gPubPoints = gNode->create_publisher<sensor_msgs::msg::PointCloud2>("points", 2);
  if (gEnableIntensity)
    gPubIntensity = it.advertise("intensity", 1);
  if (gEnableStatemap)
    gPubStatemap = it.advertise("statemap", 1);

  gDeviceIdent = gControl->getDeviceIdent();

  // diagnostics
  updater.reset(new diagnostic_updater::Updater(gNode));
  updater->setHardwareID(gNode->get_namespace());
  updater->add("driver", driver_diagnostics);

  double desiredFreq; // device max freq is 30FPS
  gNode->get_parameter("desired_frequency", desiredFreq);
  double min_freq = desiredFreq * 0.9;
  double max_freq = desiredFreq * 1.1;

  gPubCameraInfo_freq.reset(
    new diagnostic_updater::TopicDiagnostic("camera_info",
                                            *updater,
                                            diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq),
                                            diagnostic_updater::TimeStampStatusParam()));
  if (gEnableDepth)
    gPubDepth_freq.reset(
      new diagnostic_updater::TopicDiagnostic("depth",
                                              *updater,
                                              diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq),
                                              diagnostic_updater::TimeStampStatusParam()));
  if (gEnablePoints)
    gPubPoints_freq.reset(
      new diagnostic_updater::TopicDiagnostic("points",
                                              *updater,
                                              diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq),
                                              diagnostic_updater::TimeStampStatusParam()));
  if (gEnableIntensity)
    gPubIntensity_freq.reset(
      new diagnostic_updater::TopicDiagnostic("intensity",
                                              *updater,
                                              diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq),
                                              diagnostic_updater::TimeStampStatusParam()));
  if (gEnableStatemap)
    gPubStatemap_freq.reset(
      new diagnostic_updater::TopicDiagnostic("statemap",
                                              *updater,
                                              diagnostic_updater::FrequencyStatusParam(&min_freq, &max_freq),
                                              diagnostic_updater::TimeStampStatusParam()));

  auto timer = gNode->create_wall_timer(std::chrono::seconds(1), &diag_timer_cb);

  // ROS 2 has no SubscriberStatusCallback equivalent — start streaming once.
  gControl->startAcquisition();

  // start receiver thread for camera images
  boost::thread rec_thr(boost::bind(&thr_receive_frame, pDataStream, pDataHandler));

  // wait til end of exec.
  rclcpp::spin(gNode);

  gReceive = false;
  rec_thr.join();

  gControl->stopAcquisition();
  gControl->close();
  pDataStream->close();

  rclcpp::shutdown();

  return 0;
}
