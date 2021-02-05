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
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 WHETHER IN CONTRACT, * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/
#include <exception>

#include <boost/bind.hpp>
#include <boost/chrono/chrono.hpp>
#include <boost/filesystem.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/common/distances.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/common/eigen.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "aruco_calib.h"

namespace cicv
{
struct ProjectionCost
{
  ProjectionCost(const pcl::PointCloud<pcl::PointXYZ>& laser_corners,
                 const pcl::PointCloud<pcl::PointXYZ>& camera_corners)
    : laser_corners_(laser_corners), camera_corners_(camera_corners)
  {
  }
  template <typename T>
  bool operator()(const T* const angle_axis, const T* const t, T* residual) const
  {
    if (laser_corners_.size() != camera_corners_.size() || laser_corners_.size() != 4)
    {
      return false;
    }
    pcl::PointXYZ laser_corner, camera_corner;
    T s_p[3], t_p[3], dist[3];
    residual[0] = T(0.0);
    residual[1] = T(0.0);
    residual[2] = T(0.0);
    for (size_t i = 0; i < laser_corners_.size(); ++i)
    {
      // The value angle_axis is a triple whose norm is an angle in radians, and whose direction is aligned with the
      // axis
      // of rotation.
      laser_corner = laser_corners_.points[i];
      camera_corner = camera_corners_.points[i];
      s_p[0] = T(laser_corner.x);
      s_p[1] = T(laser_corner.y);
      s_p[2] = T(laser_corner.z);
      ceres::AngleAxisRotatePoint(angle_axis, s_p, t_p);
      t_p[0] = t_p[0] + t[0];
      t_p[1] = t_p[1] + t[1];
      t_p[2] = t_p[2] + t[2];
      residual[0] = residual[0] + ceres::abs(t_p[0] - T(camera_corner.x));
      residual[1] = residual[1] + ceres::abs(t_p[1] - T(camera_corner.y));
      residual[2] = residual[2] + ceres::abs(t_p[2] - T(camera_corner.z));
    }

    residual[0] /= T(4.0);
    residual[1] /= T(4.0);
    residual[2] /= T(4.0);

    return true;
  }
  const pcl::PointCloud<pcl::PointXYZ> laser_corners_;
  const pcl::PointCloud<pcl::PointXYZ> camera_corners_;
};

ArucoCalib::ArucoCalib(const std::string& config_path, const std::string& file_name)
{
  result_path_ = config_path;
  frame_num_ = 0;
  valid_num_ = 0;

  // read config file
  std::string config_file_name = config_path + "/" + file_name;
  if (!readConfigFile(config_file_name))
  {
    return;
  }

  mono_proc_ptr_.reset(new cicv::MonoPattern(config_path, file_name));
  laser_proc_ptr_.reset(new cicv::LaserPattern(config_path, file_name));

  // debug
  board_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  edge_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  lidar_corner_cloud_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  camera_corners_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  lidar_corners_ptr_.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

ArucoCalib::~ArucoCalib()
{
  if (detection_thread_->joinable())
  {
    detection_thread_->join();
  }
}

bool ArucoCalib::readConfigFile(const std::string& config_file_name)
{
  cv::FileStorage fs_reader(config_file_name, cv::FileStorage::READ);
  if (!fs_reader.isOpened())
  {
    WARN << config_file_name << " is wrong!" << REND;
    return false;
  }
  fs_reader["calib_frame_num"] >> calib_frame_num_;
  fs_reader["calib_result_file"] >> calib_result_file_;
  fs_reader.release();
  if (calib_frame_num_ < 0)
  {
    if (calib_frame_num_ == -1)
    {
      detection_thread_.reset(new boost::thread(boost::bind(&ArucoCalib::detectDataStream, this)));
      return true;
    }
    ERROR << "calib frame num should large than 0 or equal to -1!" << REND;
    return false;
  }
  return true;
}

// 返回值: -1-不是有效帧, 0-有效帧但未结束, 1-有效帧且结束
int ArucoCalib::process(const cv::Mat& input_image, const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr)
{
  INFO << "\nCurrent frame number is: " << frame_num_ << REND;
  {
    boost::unique_lock<boost::mutex> process_lock(detection_mutex_);
    ++frame_num_;
  }
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);  // forbid pcl_debug output

  // process image
  pcl::PointCloud<pcl::PointXYZ>::Ptr camera_corner_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  marker_image_ = input_image.clone();
  if (processImage(camera_corner_cloud_ptr))
  {
    INFO << "[Mono Pattern]: CORRECT!" << REND;
  }
  else
  {
    WARN << "[Mono Pattern]: WRONG!" << REND;
    INFO << "Valid frame number is: " << valid_num_ << REND;
    return -1;
  }

  // process pointcloud
  if (processCloud(input_cloud_ptr, lidar_corner_cloud_ptr_))
  {
    INFO << "[Laser Pattern]: CORRECT!" << REND;
  }
  else
  {
    WARN << "[Laser Pattern]: WRONG!" << REND;
    INFO << "Valid frame number is: " << valid_num_ << REND;
    return -1;
  }

  *camera_corners_ptr_ += *camera_corner_cloud_ptr;
  *lidar_corners_ptr_ += *lidar_corner_cloud_ptr_;
  {
    boost::unique_lock<boost::mutex> process_lock(detection_mutex_);
    ++valid_num_;
  }

  INFO << "Valid frame number is: " << valid_num_ << REND;
  if (valid_num_ == calib_frame_num_)
  {
    INFO << "\nUse " << valid_num_ << " frames to calibration." << REND;
    return 1;
  }
  if (calib_frame_num_ == -1 && valid_num_ > 50)
  {
    INFO << "\nUse " << valid_num_ << " frames to calibration." << REND;
    return 1;
  }
  return 0;
}

bool ArucoCalib::processImage(pcl::PointCloud<pcl::PointXYZ>::Ptr camera_corner_cloud_ptr)
{
  // compute marker pose
  cv::Mat transform_matrix;
  if (!mono_proc_ptr_->computeMarkerBoardPose(marker_image_, transform_matrix))
  {
    return false;
  }

  // compute board corner(camera coordinate)
  mono_proc_ptr_->computeCornerCloud(transform_matrix, camera_corner_cloud_ptr);

  //  judge corner cloud is valid or not
  if (!isValidCorners(camera_corner_cloud_ptr))
  {
#if LCDEBUG
    WARN << "corner points is invalid!" << REND;
#endif
    camera_corner_cloud_ptr->clear();
    return false;
  }

#if LCDEBUG
  // test points order
  std::vector<cv::Scalar> colors{ cv::Scalar(0, 0, 255), cv::Scalar(0, 255, 0), cv::Scalar(255, 0, 0),
                                  cv::Scalar(255, 255, 0) };
  std::vector<cv::Point3f> point3d_v(4);
  std::vector<cv::Point2f> point2d_v(4);
  for (size_t i = 0; i < camera_corner_cloud_ptr->size(); ++i)
  {
    point3d_v[i] = cv::Point3f(camera_corner_cloud_ptr->points[i].x, camera_corner_cloud_ptr->points[i].y,
                               camera_corner_cloud_ptr->points[i].z);
  }
  cv::projectPoints(point3d_v, cv::Vec3f(0, 0, 0), cv::Vec3f(0, 0, 0), mono_proc_ptr_->cam_param_.CameraMatrix,
                    mono_proc_ptr_->cam_param_.Distorsion, point2d_v);
  for (size_t i = 0; i < camera_corner_cloud_ptr->size(); ++i)
  {
    cv::circle(marker_image_, cv::Point(std::lround(point2d_v[i].x), std::lround(point2d_v[i].y)), 2, colors[i % 4],
               -1);
  }
#endif
  return true;
}

bool ArucoCalib::processCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_corner_cloud_ptr)
{
  // extract board cloud and edge cloud
  if (!laser_proc_ptr_->extractEdgeCloud(input_cloud_ptr, board_cloud_ptr_, edge_cloud_ptr_))
  {
    return false;
  }

  // compute board corner
  if (!laser_proc_ptr_->computeCloudCornersAuto(edge_cloud_ptr_, lidar_corner_cloud_ptr))
  {
    return false;
  }

  // judge corners is valid or not
  if (!isValidCorners(lidar_corner_cloud_ptr))
  {
#if LCDEBUG
    WARN << "corner points is invalid!" << REND;
#endif
    lidar_corner_cloud_ptr->clear();
    return false;
  }
  return true;
}

bool ArucoCalib::isValidCorners(const pcl::PointCloud<pcl::PointXYZ>::Ptr corners_ptr)
{
  int corner_num = corners_ptr->size();
  if (corner_num != 4)
  {
    return false;
  }

  const float board_length = laser_proc_ptr_->board_length_;  // meter
  const float board_angle = M_PI / 2.0;

  // 计算距离评分和角度评分
  pcl::PointXYZ pre_corner, cur_corner, next_corner;
  float length_diff = 0.0, angle_diff = 0.0;
  for (size_t i = 0; i < corner_num; ++i)
  {
    pre_corner = corners_ptr->points[(i + 3) % 4];
    cur_corner = corners_ptr->points[i];
    next_corner = corners_ptr->points[(i + 1) % 4];
    float dist = pcl::euclideanDistance(cur_corner, next_corner);
    length_diff += std::abs(dist - board_length);

    Eigen::Vector3f a, b;
    a << (cur_corner.x - pre_corner.x), (cur_corner.y - pre_corner.y), (cur_corner.z - pre_corner.z);
    b << (cur_corner.x - next_corner.x), (cur_corner.y - next_corner.y), (cur_corner.z - next_corner.z);
    float angle = std::acos(a.dot(b) / (a.norm() * b.norm()));
    angle_diff += std::abs(angle - board_angle);
  }
  length_diff /= corner_num;
  angle_diff /= corner_num;

  float length_core = 1 - length_diff / board_length;
  float angle_core = 1 - angle_diff / board_angle;

#if LCDEBUG
  INFO << "length core is: " << length_core * 100 << REND;
  INFO << "angle core is: " << angle_core * 100 << REND;
#endif

  if (length_core < 0.95 || angle_core < 0.95)
  {
    return false;
  }
  return true;
}

void ArucoCalib::computeTransform()
{
  // // trans lidar corner 2 camera corner
  // Eigen::Matrix4d lidar2cam_trans = getLidar2CamTrans();
  // pcl::PointCloud<pcl::PointXYZ>::Ptr lidar2cam_corner_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::transformPointCloud(*lidar_corners_ptr_, *lidar2cam_corner_ptr, lidar2cam_trans);

  Eigen::Matrix4d temp_trans;
  if (computeTransOpt(lidar_corners_ptr_, camera_corners_ptr_, temp_trans))
  {
    // final_trans_ = lidar2cam_trans * temp_trans;
    final_trans_ = temp_trans;

    pcl::PointCloud<pcl::PointXYZ>::Ptr project2cam_corner(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*lidar_corners_ptr_, *project2cam_corner, final_trans_);
    double cloud_distance = computeCloudDistance(camera_corners_ptr_, project2cam_corner);

    INFO << "calibration done!" << REND;
    transform2Pose(final_trans_, final_pose_);
    printCalibration(final_pose_, final_trans_);
    std::string file_name;
    file_name = result_path_ + "/" + calib_result_file_;
    INFO << "Calibration result is saved at: " << file_name << REND;
    saveYamlFile(file_name, mono_proc_ptr_->cam_param_, final_pose_, final_trans_);
  }
}

bool ArucoCalib::computeTransOpt(const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, Eigen::Matrix4d& trans)
{
  double angle_axis[3] = { 0 };
  double t[3] = { 0 };
  ceres::Problem problem;
  pcl::PointXYZ laser_point, camera_point;
  pcl::PointCloud<pcl::PointXYZ> laser_cloud, camera_cloud;
  for (std::size_t i = 0; i < source_cloud->size(); ++i)
  {
    laser_point = source_cloud->points[i];
    camera_point = target_cloud->points[i];
    laser_cloud.points.emplace_back(laser_point);
    camera_cloud.points.emplace_back(camera_point);
    if ((i + 1) % 4 == 0)
    {
      ceres::CostFunction* cost_function =
          new ceres::AutoDiffCostFunction<ProjectionCost, 3, 3, 3>(new ProjectionCost(laser_cloud, camera_cloud));
      problem.AddResidualBlock(cost_function, new ceres::CauchyLoss(0.5), angle_axis, t);
      laser_cloud.clear();
      camera_cloud.clear();
    }
  }
  ceres::Solver::Options option;
  option.linear_solver_type = ceres::DENSE_SCHUR;
  option.minimizer_progress_to_stdout = false;
  ceres::Solver::Summary summary;
  ceres::Solve(option, &problem, &summary);

#if LCDEBUG
  INFO << summary.BriefReport() << REND;
#endif

  double rotation[9];  // colomn major
  ceres::AngleAxisToRotationMatrix<double>(angle_axis, rotation);
  trans.setIdentity();
  for (std::size_t i = 0; i < 3; ++i)
  {
    for (std::size_t j = 0; j < 3; ++j)
    {
      trans(j, i) = rotation[i * 3 + j];
    }
  }
  trans(0, 3) = t[0];
  trans(1, 3) = t[1];
  trans(2, 3) = t[2];

  return true;
}

bool ArucoCalib::computeTransICP(const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, Eigen::Matrix4d& trans)
{
  // using pcl icp use ransac to solve non-linear optimation
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(source_cloud);
  icp.setInputTarget(target_cloud);
  icp.setTransformationEpsilon(1e-8);
  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);
  if (!icp.hasConverged())
  {
#if LCDEBUG
    WARN << "icp has unconverged!" << REND;
#endif
    return false;
  }

  Eigen::Matrix4f temp_trans = icp.getFinalTransformation();  // colomn major
  trans = temp_trans.cast<double>();
  return true;
}

bool ArucoCalib::computeTransSVD(const pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud,
                                 const pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, Eigen::Matrix4d& trans)
{
  // turn into cv::Point3f
  int corner_size = source_cloud->size();
  std::vector<cv::Point3f> pts1(corner_size), pts2(corner_size);
  for (size_t i = 0; i < corner_size; ++i)
  {
    pcl::PointXYZ cloud_point = source_cloud->points[i], camera_point = target_cloud->points[i];
    pts1[i] = cv::Point3f(camera_point.x, camera_point.y, camera_point.z);
    pts2[i] = cv::Point3f(cloud_point.x, cloud_point.y, cloud_point.z);
  }
  // compute center of mass
  cv::Point3f p1(0, 0, 0), p2(0, 0, 0);
  for (size_t i = 0; i < corner_size; ++i)
  {
    p1 += pts1[i];
    p2 += pts2[i];
  }
  p1 /= corner_size;
  p2 /= corner_size;

  // remove centroid
  std::vector<cv::Point3f> qts1(corner_size), qts2(corner_size);
  for (size_t i = 0; i < corner_size; ++i)
  {
    qts1[i] = pts1[i] - p1;
    qts2[i] = pts2[i] - p2;
  }

  // compute qts1 *  qts2^T
  Eigen::Matrix3d w = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < corner_size; ++i)
  {
    w +=
        Eigen::Vector3d(qts1[i].x, qts1[i].y, qts1[i].z) * Eigen::Vector3d(qts2[i].x, qts2[i].y, qts2[i].z).transpose();
  }

  // svd on w
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(w, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d u = svd.matrixU();
  Eigen::Matrix3d v = svd.matrixV();

  // get r and t
  Eigen::Matrix3d r = u * (v.transpose());
  Eigen::Vector3d t = Eigen::Vector3d(p1.x, p1.y, p1.z) - r * Eigen::Vector3d(p2.x, p2.y, p2.z);

  // convert to cv::Mat
  trans = Eigen::Matrix4d::Identity();
  trans.block<3, 3>(0, 0) = r;
  trans.block<3, 1>(0, 3) = t;
  return true;
}

double ArucoCalib::computeCloudDistance(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr_1,
                                        const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr_2)
{
  if (input_cloud_ptr_1->size() != input_cloud_ptr_1->size())
  {
    return -1.0;
  }

  // compute distance
  double distance_error = 0.0;
  int cloud_size = input_cloud_ptr_1->size();
  for (size_t i = 0; i < cloud_size; ++i)
  {
    pcl::PointXYZ point1 = input_cloud_ptr_1->points[i];
    pcl::PointXYZ point2 = input_cloud_ptr_2->points[i];
    double distance = pcl::euclideanDistance(point1, point2);
    distance_error += distance;
  }
  distance_error /= cloud_size;
  return distance_error;
}

Eigen::Matrix4d ArucoCalib::getLidar2CamTrans()
{
  Eigen::Matrix4d lidar2cam_trans = Eigen::Matrix4d::Zero();
  lidar2cam_trans(0, 1) = -1;
  lidar2cam_trans(1, 2) = -1;
  lidar2cam_trans(2, 0) = 1;
  lidar2cam_trans(3, 3) = 1;
  return lidar2cam_trans;
}

void ArucoCalib::detectDataStream()
{
  int pre_num = 0;
  int num = 0;
  // 数据等待超时提醒
  while (pre_num < frame_num_ || frame_num_ == 0)
  {
    pre_num = frame_num_;
    ++num;
    if (pre_num == 0 && num > 60)  // 1 mimutes
    {
      num = 0;
      WARN << "There is no input data, please play rosbag." << REND;
    }
    boost::this_thread::sleep_for(boost::chrono::seconds(3));
  }

  if (calib_frame_num_ > -1)
  {
    if (valid_num_ == calib_frame_num_)
    {
      return;
    }
    // else
    // {
    //   INFO << "The number of valid frame "<<valid_num_ << " is small than "<< calib_frame_num_<<
    //   " to compute the transform matrix." << REND;
    // }
  }
  else if (calib_frame_num_ == -1)
  {
    if (valid_num_ > 50)
    {
      return;
    }
  }
  else
  {
    WARN << "Should not be there!" << REND;
  }
}

const cv::Mat& ArucoCalib::getMarkerImage()
{
  return marker_image_;
}

const pcl::PointCloud<pcl::PointXYZ>& ArucoCalib::getBoardCloud()
{
  return *board_cloud_ptr_;
}

const pcl::PointCloud<pcl::PointXYZ>& ArucoCalib::getEdgeCloud()
{
  return *edge_cloud_ptr_;
}

const pcl::PointCloud<pcl::PointXYZ>& ArucoCalib::getLidarCornerCloud()
{
  return *lidar_corner_cloud_ptr_;
}

}  // namespace cicv