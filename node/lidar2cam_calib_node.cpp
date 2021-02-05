/******************************************************************************
 * Copyright 2017 cicv All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.cicv.ai

 * This software is provided to you directly by cicv and might
 * only be used to LiDAR and camera calibration. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without cicv's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL cicv BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include <boost/bind.hpp>
#include <thread>
#include <signal.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgcodecs.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "lidar2cam_calib.h"

std::unique_ptr<cicv::Lidar2CamCalib> g_lidar_camera_ptr;
ros::Publisher g_marker_image_pub;
ros::Publisher g_input_cloud_pub, g_board_cloud_pub;
ros::Publisher g_edge_cloud_pub, g_corner_cloud_pub;

std::vector<std::size_t> g_time_v;
bool data_ready_ = false;
bool calib_finish_ = false;

static void calibrationHandler(int sig)
{
  calib_finish_ = true;
  ros::shutdown();
}

void publishImage(const ros::Publisher& image_pub, const std_msgs::Header& header, const cv::Mat& image)
{
  cv_bridge::CvImage output_image;
  output_image.header.frame_id = header.frame_id;
  output_image.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
  output_image.image = image;
  image_pub.publish(output_image);
}

void publishCloud(const ros::Publisher& cloud_pub, const std_msgs::Header& header,
                  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  sensor_msgs::PointCloud2 output_msg;
  pcl::toROSMsg(*cloud, output_msg);
  output_msg.header = header;
  cloud_pub.publish(output_msg);
}

bool isFileExist(const std::string& file_name)
{
  if (boost::filesystem::exists(file_name) && boost::filesystem::is_regular_file(file_name))
  {
    return true;
  }
  return false;
}

void callback(const sensor_msgs::CompressedImageConstPtr input_image_msg,
              const sensor_msgs::PointCloud2ConstPtr input_cloud_msg)
{
  if (calib_finish_)
  {
    INFO << "Calibration is finished! Please stop the process!" << FLUSHEND;
    return;
  }
  if (data_ready_)
  {
    INFO << "Data capture is ready! Calibration..." << FLUSHEND;
    return;
  }
  std_msgs::Header image_header = input_image_msg->header;
  std_msgs::Header cloud_header = input_cloud_msg->header;

  cv::Mat input_image;
  try
  {
    input_image = cv::imdecode(cv::Mat(input_image_msg->data), 1);
  }
  catch (cv_bridge::Exception& e)
  {
    WARN << "could not convert sensor_msg image into opencv image!" << REND;
    return;
  }
  if (input_image.empty())
  {
    WARN << "input image is empty, please check it out!" << REND;
    return;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*input_cloud_msg, *input_cloud_ptr);
  if (input_cloud_ptr->size() == 0)
  {
    WARN << "input cloud is empty, please check it out!" << REND;
    return;
  }

  // process
  if (g_lidar_camera_ptr->process(input_image, input_cloud_ptr) > 0)
  {
    data_ready_ = true;
    return;
  }

  // rviz
  cv::Mat marker_image = g_lidar_camera_ptr->getMarkerImage();
  publishImage(g_marker_image_pub, image_header, marker_image);

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_xyz_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*input_cloud_ptr, *input_cloud_xyz_ptr);
  publishCloud(g_input_cloud_pub, cloud_header, input_cloud_xyz_ptr);

  pcl::PointCloud<pcl::PointXYZ>::Ptr board_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *board_cloud_ptr = g_lidar_camera_ptr->getBoardCloud();
  publishCloud(g_board_cloud_pub, cloud_header, board_cloud_ptr);

  pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *edge_cloud_ptr = g_lidar_camera_ptr->getEdgeCloud();
  publishCloud(g_edge_cloud_pub, cloud_header, edge_cloud_ptr);

  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_corner_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  *lidar_corner_cloud_ptr = g_lidar_camera_ptr->getCornerCloud();
  publishCloud(g_corner_cloud_pub, cloud_header, lidar_corner_cloud_ptr);
}

void getCalibResult()
{
  while (ros::ok() && !calib_finish_)
  {
    if (data_ready_)
    {
      INFO << "Start calibration!" << REND;
      g_lidar_camera_ptr->compute();
      data_ready_ = false;
      calib_finish_ = true;
    }
    boost::this_thread::sleep_for(boost::chrono::seconds(1));
  }
}

int main(int argc, char** argv)
{
  signal(SIGINT, calibrationHandler);
  ros::init(argc, argv, "rs_lidar2cam_calib_node", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::NodeHandle priv_nh("~");

  std::string config_path, file_name;
  if (priv_nh.hasParam("path") && priv_nh.hasParam("file_name"))
  {
    priv_nh.getParam("path", config_path);
    priv_nh.getParam("file_name", file_name);
  }
  else
  {
    WARN << "Config file is empty!" << REND;
    return 0;
  }
  std::string config_file_name = config_path + "/" + file_name;
  INFO << "config file name: " << config_file_name << REND;
  cv::FileStorage fs_reader(config_file_name, cv::FileStorage::READ);
  if (!fs_reader.isOpened())
  {
    WARN << config_file_name << " is wrong!" << REND;
    return false;
  }
  std::string camera_topic, lidar_topic;
  fs_reader["lidar_topic"] >> lidar_topic;
  fs_reader["camera_topic"] >> camera_topic;
  fs_reader.release();
  INFO << "lidar topic: " << lidar_topic << REND;
  INFO << "camera topic: " << camera_topic << REND;
  if (lidar_topic.empty() || camera_topic.empty())
  {
    WARN << "lidar topic or camera topic is empty!" << REND;
    return 0;
  }

  g_marker_image_pub = nh.advertise<sensor_msgs::Image>("marker_image", 10);
  g_input_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("input_cloud", 10);
  g_board_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("board_cloud", 10);
  g_edge_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("edge_cloud", 10);
  g_corner_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("corner_cloud", 10);

  g_lidar_camera_ptr.reset(new cicv::Lidar2CamCalib(config_path, file_name));
  message_filters::Subscriber<sensor_msgs::CompressedImage> camera_sub(nh, camera_topic, 10);
  message_filters::Subscriber<sensor_msgs::PointCloud2> lidar_sub(nh, lidar_topic, 10);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::PointCloud2>
      MySyncPolicy;
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), camera_sub, lidar_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  std::thread compute_thread;
  const auto& func1 = [] { getCalibResult(); };
  compute_thread = std::thread(func1);

  ros::spin();
  if (compute_thread.joinable())
  {
    compute_thread.join();
  }

  return 0;
}
