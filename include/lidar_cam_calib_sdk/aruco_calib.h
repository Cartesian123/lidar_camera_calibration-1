/******************************************************************************
 * Copyright 2017 cicv All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.cicv.ai

 * This software is provided to you directly by cicv and might
 * only be used to access cicv LiDAR. Any compilation,
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

#ifndef ARUCO_CALIB_H
#define ARUCO_CALIB_H

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <Eigen/Dense>

#include "common.h"
#include "mono_pattern.h"
#include "laser_pattern.h"

namespace cicv
{
class ArucoCalib
{
public:
  ArucoCalib(const std::string& config_path, const std::string& file_name);
  ~ArucoCalib();

  int process(const cv::Mat& input_image, const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr);
  void computeTransform();
  const cv::Mat& getMarkerImage();
  const pcl::PointCloud<pcl::PointXYZ>& getBoardCloud();
  const pcl::PointCloud<pcl::PointXYZ>& getEdgeCloud();
  const pcl::PointCloud<pcl::PointXYZ>& getLidarCornerCloud();

private:
  bool readConfigFile(const std::string& config_file_name);
  bool processImage(pcl::PointCloud<pcl::PointXYZ>::Ptr camera_corner_cloud_ptr);
  bool processCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr laser_corner_cloud_ptr);
  bool isValidCorners(const pcl::PointCloud<pcl::PointXYZ>::Ptr corner_cloud);
  bool computeTransICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, Eigen::Matrix4d& trans);
  bool computeTransSVD(const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, Eigen::Matrix4d& trans);
  bool computeTransOpt(const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                       const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, Eigen::Matrix4d& trans);
  double computeCloudDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr_1,
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr_2);
  Eigen::Matrix4d getLidar2CamTrans();
  void detectDataStream();

private:
  std::unique_ptr<cicv::MonoPattern> mono_proc_ptr_;
  std::unique_ptr<cicv::LaserPattern> laser_proc_ptr_;

  // config
  int frame_num_;
  int valid_num_;
  int calib_frame_num_;
  boost::mutex detection_mutex_;
  boost::shared_ptr<boost::thread> detection_thread_;

  std::string result_path_;
  std::string calib_result_file_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr camera_corners_ptr_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_corners_ptr_;

  Eigen::Matrix4d final_trans_;
  std::vector<double> final_pose_;

  // output image and cloud
  cv::Mat marker_image_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr board_cloud_ptr_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud_ptr_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_corner_cloud_ptr_;
};

}  // namespace cicv
#endif
