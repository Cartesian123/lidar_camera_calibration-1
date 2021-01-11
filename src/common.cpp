
/******************************************************************************
 * Copyright 2017 cicv All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.cicv.ai

 * This software is provided to you directly by cicv and might
 * only be used to access cicv LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are * restricted without cicv's prior consent.

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
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <pcl/filters/filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/intersections.h>

#include <opencv2/core/persistence.hpp>
#include <opencv2/core/eigen.hpp>

#include "common.h"

namespace cicv
{
bool readCamParam(const std::string& file_name, aruco::CameraParameters& cam_param)
{
  cv::FileStorage fs_reader(file_name, cv::FileStorage::READ);
  if (!fs_reader.isOpened())
  {
    WARN << file_name << " is wrong!" << REND;
    return false;
  }
  fs_reader["CameraMat"] >> cam_param.CameraMatrix;
  fs_reader["DistCoeff"] >> cam_param.Distorsion;
  fs_reader["ImageSize"] >> cam_param.CamSize;
  fs_reader.release();
  if (!cam_param.isValid())
  {
    WARN << "CameraMat, DistCoeff or Image_Size is empty, check it!" << REND;
    return false;
  }
  return true;
}

void printCamPara(aruco::CameraParameters& cam_param)
{
  INFO << "ImageSize: " << cam_param.CamSize << REND;
  INFO << "\n"
       << "CameraMat: " << REND << cam_param.CameraMatrix << REND;
  INFO << "DistCoeff: " << REND << cam_param.Distorsion << REND;
}

bool isDirExist(const std::string& path_name)
{
  if (boost::filesystem::exists(path_name) && boost::filesystem::is_directory(path_name))
  {
    return true;
  }
  return false;
}

bool createNewDir(const std::string& path_name)
{
  if (isDirExist(path_name))
  {
    return true;
  }
  return boost::filesystem::create_directories(path_name);
}

bool isFileExist(const std::string& file_name)
{
  if (boost::filesystem::exists(file_name) && boost::filesystem::is_regular_file(file_name))
  {
    return true;
  }
  return false;
}

bool createNewFile(const std::string& file_name)
{
  if (isFileExist(file_name))
  {
    return true;
  }
  boost::filesystem::ofstream file(file_name);
  file.close();
  return isFileExist(file_name);
}

// compute plane model
bool computePlaneModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud_ptr,
                       pcl::ModelCoefficients::Ptr plane_coeff_ptr)
{
  if (input_cloud_ptr->size() < 5)
  {
#if RSDEBUG
    WARN << "too less points to fit a plane!" << REND;
#endif
    return false;
  }
  Eigen::VectorXf v_plane_coeff;
  std::vector<int> inliers;
  pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr plane_model(
      new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(input_cloud_ptr));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(plane_model);
  ransac.setDistanceThreshold(0.04);
  ransac.setMaxIterations(100);
  if (!ransac.computeModel())
  {
#if RSDEBUG
    WARN << "fail to compute plane model!" << REND;
#endif
    return false;
  }
  ransac.getModelCoefficients(v_plane_coeff);
  ransac.getInliers(inliers);

  pcl::copyPointCloud(*input_cloud_ptr, inliers, *inlier_cloud_ptr);
  if (v_plane_coeff[2] < 0.0f)
  {
    for (size_t i = 0; i < v_plane_coeff.size(); ++i)
    {
      v_plane_coeff[i] *= -1;
    }
  }

  plane_coeff_ptr->values.resize(v_plane_coeff.size());
  for (size_t i = 0; i < v_plane_coeff.size(); ++i)
  {
    plane_coeff_ptr->values[i] = v_plane_coeff[i];
  }
  return true;
}

void computeCloudNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                        pcl::PointCloud<pcl::Normal>::Ptr normal_cloud_ptr)
{
  normal_cloud_ptr->clear();
  if (input_cloud_ptr->empty())
  {
    return;
  }
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>::Ptr normal_est_ptr(
      new pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal>);
  normal_est_ptr->setInputCloud(input_cloud_ptr);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr kd_tree_ptr(new pcl::search::KdTree<pcl::PointXYZ>());
  normal_est_ptr->setSearchMethod(kd_tree_ptr);
  normal_est_ptr->setRadiusSearch(0.1);
  normal_est_ptr->compute(*normal_cloud_ptr);
}

bool computePlaneModelNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud_ptr,
                             pcl::ModelCoefficients::Ptr plane_coeff_ptr)
{
  if (input_cloud_ptr->size() < 5)
  {
#if RSDEBUG
    WARN << "too less points to fit a plane!" << REND;
#endif
    return false;
  }
  pcl::PointCloud<pcl::Normal>::Ptr normal_cloud_ptr(new pcl::PointCloud<pcl::Normal>);
  computeCloudNormal(input_cloud_ptr, normal_cloud_ptr);

  Eigen::VectorXf v_plane_coeff;
  std::vector<int> inlier_indice;
  pcl::SampleConsensusModelNormalPlane<pcl::PointXYZ, pcl::Normal>::Ptr plane_model(
      new pcl::SampleConsensusModelNormalPlane<pcl::PointXYZ, pcl::Normal>(input_cloud_ptr));
  plane_model->setNormalDistanceWeight(0.1);
  plane_model->setInputNormals(normal_cloud_ptr);

  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(plane_model);
  ransac.setDistanceThreshold(0.04);
  if (!ransac.computeModel())
  {
#if RSDEBUG
    WARN << "fail to compute plane model!" << REND;
#endif
    return false;
  }
  ransac.getModelCoefficients(v_plane_coeff);
  ransac.getInliers(inlier_indice);

  pcl::copyPointCloud(*input_cloud_ptr, inlier_indice, *inlier_cloud_ptr);

  if (v_plane_coeff[2] < 0)
  {
    for (size_t i = 0; i < v_plane_coeff.size(); ++i)
    {
      v_plane_coeff[i] *= -1;
    }
  }

  plane_coeff_ptr->values.resize(v_plane_coeff.size());
  for (size_t i = 0; i < v_plane_coeff.size(); ++i)
  {
    plane_coeff_ptr->values[i] = v_plane_coeff[i];
  }
  return true;
}

void projectToPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                    const pcl::ModelCoefficients::Ptr plane_coeff_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr)
{
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(input_cloud_ptr);
  proj.setModelCoefficients(plane_coeff_ptr);
  proj.filter(*output_cloud_ptr);
}

// line model
bool computeLineModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, Eigen::VectorXf& line_coeff)
{
  if (input_cloud_ptr->size() < 3)
  {
#if RSDEBUG
    WARN << "too less points to fit a line!" << REND;
#endif
    return false;
  }
  pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr line_model(
      new pcl::SampleConsensusModelLine<pcl::PointXYZ>(input_cloud_ptr));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(line_model);
  ransac.setDistanceThreshold(0.02);
  if (ransac.computeModel())
  {
#if RSDEBUG
    WARN << "fail to compute line model!" << REND;
#endif
    return false;
  }
  ransac.getModelCoefficients(line_coeff);
  return true;
}

bool seperateEdgeCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                       std::vector<Eigen::VectorXf>& v_line_coeff,
                       std::vector<pcl::PointCloud<pcl::PointXYZ>>& v_line_cloud)
{
  v_line_coeff.clear();
  v_line_cloud.clear();
  if (input_cloud_ptr->empty())
  {
    WARN << "input cloud is empty!" << REND;
    return false;
  }

  // extract line segment from edges cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr remain_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr line_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  pcl::ModelCoefficients::Ptr line_coeff(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices);
  pcl::SACSegmentation<pcl::PointXYZ> line_segment;
  line_segment.setModelType(pcl::SACMODEL_LINE);
  line_segment.setDistanceThreshold(0.03);

  pcl::copyPointCloud(*input_cloud_ptr, *remain_cloud);
  pcl::ExtractIndices<pcl::PointXYZ> line_extract;

  while (remain_cloud->size() >= 3 && v_line_cloud.size() < 4)
  {
    line_segment.setInputCloud(remain_cloud);
    line_segment.segment(*line_inliers, *line_coeff);
    if (line_inliers->indices.size() < 3)
    {
#if RSDEBUG
      WARN << "fail to seperate lines!" << REND;
#endif
      return false;
    }
    Eigen::VectorXf temp_coeff(6);
    temp_coeff << line_coeff->values[0], line_coeff->values[1], line_coeff->values[2], line_coeff->values[3],
        line_coeff->values[4], line_coeff->values[5];
    v_line_coeff.emplace_back(temp_coeff);

    line_extract.setInputCloud(remain_cloud);
    line_extract.setIndices(line_inliers);
    line_extract.setNegative(false);
    line_extract.filter(*line_cloud);
    v_line_cloud.emplace_back(*line_cloud);

    // remove inlier from remain cloud
    line_extract.setNegative(true);
    line_extract.filter(*temp_cloud);
    remain_cloud.swap(temp_cloud);
  }
  if (v_line_cloud.size() != 4)
  {
    return false;
  }
  return true;
}

bool cmpZ(const std::pair<Eigen::VectorXf, pcl::PointCloud<pcl::PointXYZ>>& lhs,
          const std::pair<Eigen::VectorXf, pcl::PointCloud<pcl::PointXYZ>>& rhs)
{
  return lhs.first(2) > rhs.first(2);
}

void orderLineCloud(std::vector<Eigen::VectorXf>& v_line_coeff,
                    std::vector<pcl::PointCloud<pcl::PointXYZ>>& v_line_cloud)
{
  int line_num = v_line_coeff.size();
  std::vector<std::pair<Eigen::VectorXf, pcl::PointCloud<pcl::PointXYZ>>> coeff_cloud_v;
  for (size_t i = 0; i < line_num; ++i)
  {
    coeff_cloud_v.emplace_back(std::make_pair(v_line_coeff[i], v_line_cloud[i]));
  }
  std::sort(coeff_cloud_v.begin(), coeff_cloud_v.end(), cmpZ);
  // sort by descending order
  // The six coefficients of the line are given by a point on the line and the direction of the line as:
  // [point_on_line.x point_on_line.y point_on_line.z line_direction.x line_direction.y line_direction.z]
  // line order in clockwise: top-left, top-right, bottom-right, bottom-left
  if (coeff_cloud_v[0].first(1) < coeff_cloud_v[1].first(1))
  {
    std::swap(coeff_cloud_v[0], coeff_cloud_v[1]);
  }
  if (coeff_cloud_v[2].first(1) > coeff_cloud_v[3].first(1))
  {
    std::swap(coeff_cloud_v[2], coeff_cloud_v[3]);
  }

  v_line_coeff.clear();
  v_line_cloud.clear();
  for (size_t i = 0; i < line_num; ++i)
  {
    // std::cout << "coeff: " << coeff_cloud_v[i].first.size() << std::endl;
    // std::cout << "cloud: " << coeff_cloud_v[i].second.size() << std::endl;
    v_line_coeff.emplace_back(coeff_cloud_v[i].first);
    v_line_cloud.emplace_back(coeff_cloud_v[i].second);
  }
}

bool computeLinesIntersect(const Eigen::VectorXf& line_coeff_1, const Eigen::VectorXf& line_coeff_2,
                           pcl::PointXYZ& intersect_point)
{
  Eigen::Vector4f p1, p2, p_intersect;
  pcl::lineToLineSegment(line_coeff_1, line_coeff_2, p1, p2);
  pcl::PointXYZ begin_point(p1(0), p1(1), p1(2));
  pcl::PointXYZ end_point(p2(0), p2(1), p2(2));
  if (pcl::squaredEuclideanDistance(begin_point, end_point) > 0.01f)
  {
    return false;
  }
  for (size_t i = 0; i < 4; ++i)
  {
    p_intersect(i) = (p1(i) + p2(i)) / 2.0f;
  }
  intersect_point = pcl::PointXYZ(p_intersect(0), p_intersect(1), p_intersect(2));
  return true;
}

// transformation and pose
void pose2Transform(const std::vector<double>& pose, Eigen::Matrix4d& transform)
{
  Eigen::AngleAxisd rotation_x(pose[3], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rotation_y(pose[4], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd rotation_z(pose[5], Eigen::Vector3d::UnitZ());
  Eigen::Translation3d translation(pose[0], pose[1], pose[2]);
  transform = (translation * rotation_x * rotation_y * rotation_z).matrix();
}

void transform2Pose(const Eigen::Matrix4d& transform, std::vector<double>& pose)
{
  pose.resize(6);
  Eigen::Vector3d angle = transform.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
  pose[0] = transform(0, 3);
  pose[1] = transform(1, 3);
  pose[2] = transform(2, 3);
  pose[3] = angle(0);
  pose[4] = angle(1);
  pose[5] = angle(2);
}

void projectPoint(const pcl::PointXYZ& point_3d, const cv::Mat& camera_mat, cv::Point& point_2d)
{
  cv::Mat pt_3d(3, 1, CV_64FC1);
  pt_3d.at<double>(0) = point_3d.x;
  pt_3d.at<double>(1) = point_3d.y;
  pt_3d.at<double>(2) = point_3d.z;

  cv::Mat pt_2d = camera_mat * pt_3d;
  point_2d.x = static_cast<int>(pt_2d.at<double>(0) / pt_2d.at<double>(2));
  point_2d.y = static_cast<int>(pt_2d.at<double>(1) / pt_2d.at<double>(2));
}

void printCalibration(const std::vector<double>& pose, const Eigen::Matrix4d& trans)
{
  INFO << "calibration result: " << REND;
  INFO << "lidar2cam transformation matrix:\n" << trans << REND;
  INFO << "lidar2cam pose vector: " << REND;
  for (size_t i = 0; i < pose.size(); ++i)
  {
    INFO << pose[i] << "  ";
  }
  INFO << REND;
}

void saveYamlFile(const std::string file_name, const aruco::CameraParameters& cam_param,
                  const std::vector<double>& pose, const Eigen::Matrix4d& transformation)
{
  cv::FileStorage fs_writer(file_name, cv::FileStorage::WRITE);
  if (fs_writer.isOpened())
  {
    cv::Mat trans_mat;
    cv::eigen2cv(transformation, trans_mat);
    fs_writer << "CameraMat" << cam_param.CameraMatrix;
    fs_writer << "DistCoeff" << cam_param.Distorsion;
    fs_writer << "ImageSize" << cam_param.CamSize;
    fs_writer << "CameraExtrinsicMat" << trans_mat;
    fs_writer << "Pose" << pose;
    fs_writer.release();
    INFO << "Save result file successfully!" << REND;
  }
  else
  {
    WARN << "Fail to open yaml file" << REND;
  }
  fs_writer.release();
}
}  // namespace cicv
