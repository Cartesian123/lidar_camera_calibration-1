
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
#include <pcl/sample_consensus/sac_model_plane.h>
#include <opencv2/core/persistence.hpp>
#include <pcl/common/intersections.h>
#include <pcl/io/pcd_io.h>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "laser_pattern.h"

namespace cicv
{
struct CornerCost
{
  CornerCost(const pcl::PointCloud<pcl::PointXYZ>& edge_cloud, const int line_index, const double board_length)
    : edge_cloud_(edge_cloud), line_index_(line_index), board_length_(board_length)
  {
  }
  template <typename T>
  bool operator()(const T* const corner_v, T* residual) const
  {
    if (edge_cloud_.empty())
    {
      return false;
    }
    // cost1: 边缘点到直线的距离
    T dist_sum_1 = T(0.0);
    for (size_t i = 0; i < edge_cloud_.size(); ++i)
    {
      pcl::PointXYZ point = edge_cloud_.points[i];
      T point_c[3];
      point_c[0] = T(point.x);
      point_c[1] = T(point.y);
      point_c[2] = T(point.z);
      T a[3], b[3];
      a[0] = corner_v[((line_index_ + 3) % 4) * 3] - corner_v[line_index_ * 3];
      a[1] = corner_v[((line_index_ + 3) % 4) * 3 + 1] - corner_v[line_index_ * 3 + 1];
      a[2] = corner_v[((line_index_ + 3) % 4) * 3 + 2] - corner_v[line_index_ * 3 + 2];
      b[0] = corner_v[line_index_ * 3] - point_c[0];
      b[1] = corner_v[line_index_ * 3 + 1] - point_c[1];
      b[2] = corner_v[line_index_ * 3 + 2] - point_c[2];
      T a_norm = ceres::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
      T b_norm = ceres::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
      T a_dot_b = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
      T dist = b_norm * ceres::sin(ceres::acos(a_dot_b / (a_norm * b_norm)));
      dist_sum_1 += ceres::abs(dist);
    }
    dist_sum_1 /= T(edge_cloud_.size());

    // cost2: 边长都等于75cm, 且斜边等于75根号2
    T dist_sum_2 = T(0.0);
    for (size_t i = 0; i < 4; ++i)
    {
      T a[3], b[3];
      a[0] = corner_v[((i + 1) % 4) * 3] - corner_v[i * 3];
      a[1] = corner_v[((i + 1) % 4) * 3 + 1] - corner_v[i * 3 + 1];
      a[2] = corner_v[((i + 1) % 4) * 3 + 2] - corner_v[i * 3 + 2];
      T a_norm = ceres::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
      dist_sum_2 += ceres::abs(a_norm - T(board_length_));
      if (i < 2)
      {
        b[0] = corner_v[(i + 2) * 3] - corner_v[i * 3];
        b[1] = corner_v[(i + 2) * 3 + 1] - corner_v[i * 3 + 1];
        b[2] = corner_v[(i + 2) * 3 + 2] - corner_v[i * 3 + 2];
        T b_norm = ceres::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
        dist_sum_2 += ceres::abs(b_norm - T(board_length_ * std::sqrt(2)));
      }
    }
    dist_sum_2 /= T(6.0);

    residual[0] = dist_sum_1 + dist_sum_2;
    return true;
  }

  const pcl::PointCloud<pcl::PointXYZ> edge_cloud_;
  const int line_index_;
  const double board_length_;
};

LaserPattern::LaserPattern(const std::string& config_path, const std::string& file_name)
{
  // config
  std::string config_file_name = config_path + "/" + file_name;
  if (!readConfigFile(config_file_name))
  {
    return;
  }
}
LaserPattern::~LaserPattern()
{
}

bool LaserPattern::readConfigFile(const std::string& config_file_name)
{
  cv::FileStorage fs_reader(config_file_name, cv::FileStorage::READ);
  if (!fs_reader.isOpened())
  {
    WARN << config_file_name << " is wrong!" << REND;
    return false;
  }
  std::vector<double> marker_length;
  fs_reader["marker_length"] >> marker_length;
  fs_reader["cloud_roi"] >> cloud_roi_;
  double dist_thres;
  fs_reader["distance_thres"] >> dist_thres;
  fs_reader.release();

  if (marker_length.size() != 2 || marker_length[0] <= 0 || marker_length[1] <= 0)
  {
    WARN << "marker length value is empty!" << REND;
    return false;
  }
  board_length_ = (2 + 0.25) * marker_length[0] + 2 * marker_length[1];

  if (cloud_roi_.size() != 6 || cloud_roi_[0] >= cloud_roi_[1] || cloud_roi_[2] >= cloud_roi_[3] ||
      cloud_roi_[4] >= cloud_roi_[5])
  {
    WARN << "cloud roi value is wrong!" << REND;
    return false;
  }

  INFO << "distance thres: " << dist_thres << REND;
  if (dist_thres <= 0)
  {
    WARN << "distance thres value is wrong!" << REND;
    return false;
  }
  square_dist_thres_ = dist_thres * dist_thres;
  return true;
}

// 利用距离差来提取边缘
bool LaserPattern::extractEdgeCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr board_cloud_ptr,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud_ptr)
{
  board_cloud_ptr->clear();
  edge_cloud_ptr->clear();
  if (!input_cloud_ptr->isOrganized())
  {
    WARN << "input cloud is not organized!" << REND;
    return false;
  }

  // 计算距离差, 初步筛选边缘
  int search_length = 5;
  pcl::PointCloud<pcl::PointXYZ>::Ptr roi_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_edge_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<bool> is_left_flag_v_temp(0);
  pcl::PointXYZI cur_point, pre_point, suc_point;
  float pre_square_dist = 0, suc_square_dist = 0;
  for (size_t i = 0; i < input_cloud_ptr->height; ++i)
  {
    int row_offset = i * input_cloud_ptr->width;
    for (size_t j = 0; j < input_cloud_ptr->width; ++j)
    {
      cur_point = input_cloud_ptr->points[row_offset + j];
      if (pcl::isFinite(cur_point))
      {
        if (cur_point.x > cloud_roi_[0] && cur_point.x < cloud_roi_[1] && cur_point.y > cloud_roi_[2] &&
            cur_point.y < cloud_roi_[3] && cur_point.z > cloud_roi_[4] && cur_point.z < cloud_roi_[5])
        {
          roi_cloud_ptr->points.emplace_back(cur_point.x, cur_point.y, cur_point.z);
          bool is_pre_find = false;
          for (size_t k = 1; k <= search_length; ++k)
          {
            if (row_offset + j - k >= 0)
            {
              pcl::PointXYZI pre_point = input_cloud_ptr->points[row_offset + j - k];
              if (pcl::isFinite(pre_point))
              {
                pre_square_dist = (cur_point.x - pre_point.x) * (cur_point.x - pre_point.x) +
                                  (cur_point.y - pre_point.y) * (cur_point.y - pre_point.y) +
                                  (cur_point.z - pre_point.z) * (cur_point.z - pre_point.z);
                is_pre_find = true;
                break;
              }
            }
          }
          bool is_suc_find = false;
          for (size_t k = 1; k <= search_length; ++k)
          {
            if (row_offset + j + k < input_cloud_ptr->size())
            {
              pcl::PointXYZI suc_point = input_cloud_ptr->points[row_offset + j + k];
              if (pcl::isFinite(suc_point))
              {
                suc_square_dist = (cur_point.x - suc_point.x) * (cur_point.x - suc_point.x) +
                                  (cur_point.y - suc_point.y) * (cur_point.y - suc_point.y) +
                                  (cur_point.z - suc_point.z) * (cur_point.z - suc_point.z);
                is_suc_find = true;
                break;
              }
            }
          }
          // 如果旁边没有点, 或者都是nan点
          if (!is_pre_find || !is_suc_find)
          {
            continue;
          }
          if (pre_square_dist > square_dist_thres_ && suc_square_dist < 0.15 * 0.15)
          {
            temp_edge_cloud_ptr->points.emplace_back(cur_point.x, cur_point.y, cur_point.z);
            is_left_flag_v_temp.emplace_back(true);
          }
          else if (suc_square_dist > square_dist_thres_ && pre_square_dist < 0.15 * 0.15)
          {
            temp_edge_cloud_ptr->points.emplace_back(cur_point.x, cur_point.y, cur_point.z);
            is_left_flag_v_temp.emplace_back(false);
          }
        }
      }
    }
  }
#if RSDEBUG
  INFO << "roi points num: " << roi_cloud_ptr->size() << REND;
#endif
  if (roi_cloud_ptr->size() < 12)
  {
    WARN << "too less point in roi region!" << REND;
    return false;
  }

  pcl::ModelCoefficients::Ptr plane_coeff(new pcl::ModelCoefficients);
  if (input_cloud_ptr->height == 16)
  {
    if (!computePlaneModel(roi_cloud_ptr, board_cloud_ptr, plane_coeff))
    {
      return false;
    }
  }
  else
  {
    if (!computePlaneModelNormal(roi_cloud_ptr, board_cloud_ptr, plane_coeff))
    {
      return false;
    }
  }

  // 筛选边缘点, 剔除掉跳动大的边缘点
  std::vector<bool> is_left_flag_v(0);
  for (size_t i = 0; i < temp_edge_cloud_ptr->size(); ++i)
  {
    pcl::PointXYZ cur_point = temp_edge_cloud_ptr->points[i];
    double dist = pcl::pointToPlaneDistance(cur_point, plane_coeff->values[0], plane_coeff->values[1],
                                            plane_coeff->values[2], plane_coeff->values[3]);
    if (dist < 0.10)  // ignore points which are far form calibration board
    {
      edge_cloud_ptr->points.emplace_back(cur_point);
      is_left_flag_v.emplace_back(is_left_flag_v_temp[i]);
    }
  }

  int edge_cloud_num = edge_cloud_ptr->size();
#if RSDEBUG
  INFO << "edge points num: " << edge_cloud_num << REND;
#endif
  if (edge_cloud_num < 12 || edge_cloud_num > input_cloud_ptr->height * 2)
  {
#if RSDEBUG
    WARN << "edge points num is wrong! " << REND;
#endif
    edge_cloud_ptr->clear();
    return false;
  }

  // 对边缘点进行矫正和补偿
  Eigen::Vector4f board_plane_coeff(plane_coeff->values[0], plane_coeff->values[1], plane_coeff->values[2],
                                    plane_coeff->values[3]);
  float horizon_angle = 0.0, vertical_angle = 0.0;
  for (size_t i = 0; i < edge_cloud_ptr->size(); ++i)
  {
    pcl::PointXYZ& point = edge_cloud_ptr->points[i];
    bool is_left_flag = is_left_flag_v[i];

    if (std::abs(point.x) < 0.1 && std::abs(point.z) < 0.1 || std::abs(point.x) < 0.1 && std::abs(point.y))
    {
      continue;
    }

    // 计算水平角和垂直角
    horizon_angle = std::atan2(point.y, point.x);
    vertical_angle = std::atan2(point.z, point.x);
    if (is_left_flag)
    {
      horizon_angle = horizon_angle + (0.1 / 180.0) * M_PI;  // 补偿0.1°
    }
    else
    {
      horizon_angle = horizon_angle - (0.1 / 180.0) * M_PI;
    }
    // 求取补偿线的两个平面
    Eigen::Matrix3f vertical_rotation_matrix =
        Eigen::AngleAxisf(-vertical_angle, Eigen::Vector3f(0.0, 1.0, 0.0)).matrix();
    Eigen::Matrix3f horizon_rotation_matrix = Eigen::AngleAxisf(horizon_angle, Eigen::Vector3f(0.0, 0.0, 1.0)).matrix();
    Eigen::Vector3f vertical_plane_norm =
        vertical_rotation_matrix * horizon_rotation_matrix * Eigen::Vector3f(0.0, 0.0, 1.0);

    Eigen::Vector3f horizon_plane_norm = horizon_rotation_matrix * Eigen::Vector3f(0.0, 1.0, 0.0);
    Eigen::Vector4f vertical_plane_coeff, horizon_plane_coeff;
    vertical_plane_coeff.head<3>() = vertical_plane_norm;
    vertical_plane_coeff(3) = 0.0;
    horizon_plane_coeff.head<3>() = horizon_plane_norm;
    horizon_plane_coeff(3) = 0.0;
    Eigen::Vector3f intersection_point;
    pcl::threePlanesIntersection(board_plane_coeff, vertical_plane_coeff, horizon_plane_coeff, intersection_point);
    pcl::PointXYZ correct_point = pcl::PointXYZ(intersection_point(0), intersection_point(1), intersection_point(2));
    point = correct_point;
  }
  return true;
}

bool LaserPattern::computeCloudCornersAuto(const pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud_ptr,
                                           pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_corner_cloud_ptr)
{
  lidar_corner_cloud_ptr->clear();
  if (edge_cloud_ptr->empty())
  {
    return false;
  }

  std::vector<Eigen::VectorXf> v_line_coeff;
  std::vector<pcl::PointCloud<pcl::PointXYZ> > v_line_cloud;
  if (!seperateEdgeCloud(edge_cloud_ptr, v_line_coeff, v_line_cloud))
  {
#if RSDEBUG
    WARN << "fail to seperate line!" << REND;
#endif
    return false;
  }
  orderLineCloud(v_line_coeff, v_line_cloud);

  int edge_num = v_line_cloud.size();
  if (edge_num != 4)
  {
    return false;
  }

  // compute init corners
  pcl::PointCloud<pcl::PointXYZ>::Ptr init_corners_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (size_t j = 0; j < 4; ++j)
  {
    pcl::PointXYZ intersect_point;
    if (!computeLinesIntersect(v_line_coeff[j], v_line_coeff[(j + 1) % 4], intersect_point))
    {
#if RSDEBUG
      WARN << "fail to compute line intersection!" << REND;
#endif
      return false;
    }
    init_corners_ptr->points.emplace_back(intersect_point);
#if 0
    std::cout << j << " init corner point: " << intersect_point.x << " " << intersect_point.y << " "
              << intersect_point.z << std::endl;
#endif
  }

  double corners[12] = { init_corners_ptr->points[0].x, init_corners_ptr->points[0].y, init_corners_ptr->points[0].z,
                         init_corners_ptr->points[1].x, init_corners_ptr->points[1].y, init_corners_ptr->points[1].z,
                         init_corners_ptr->points[2].x, init_corners_ptr->points[2].y, init_corners_ptr->points[2].z,
                         init_corners_ptr->points[3].x, init_corners_ptr->points[3].y, init_corners_ptr->points[3].z };
  ceres::Problem problem;
  for (std::size_t i = 0; i < v_line_cloud.size(); ++i)
  {
    ceres::CostFunction* cost_function =
        new ceres::AutoDiffCostFunction<CornerCost, 1, 12>(new CornerCost(v_line_cloud[i], i, board_length_));
    problem.AddResidualBlock(cost_function, nullptr, corners);
  }

  ceres::Solver::Options option;
  option.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(option, &problem, &summary);

  for (size_t i = 0; i < v_line_cloud.size(); ++i)
  {
    pcl::PointXYZ point = pcl::PointXYZ(corners[i * 3], corners[i * 3 + 1], corners[i * 3 + 2]);
    lidar_corner_cloud_ptr->points.emplace_back(point);
  }

#if RSDEBUG
  INFO << "corner points num: " << lidar_corner_cloud_ptr->size() << REND;
#endif
  if (lidar_corner_cloud_ptr->size() != 4)
  {
#if RSDEBUG
    WARN << "corner points num is wrong!" << REND;
#endif
    return false;
  }
  return true;
}

}  // namespace cicv
