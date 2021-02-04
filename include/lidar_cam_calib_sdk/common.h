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

#ifndef ARUCO_CALIB_COMMON_H
#define ARUCO_CALIB_COMMON_H

#include <aruco/cameraparameters.h>

#include <opencv2/core/mat.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>

#include "prompt.h"

namespace cicv
{
bool readCamParam(const std::string& file_name, aruco::CameraParameters& cam_param);
void printCamParam(const aruco::CameraParameters& cam_param);

bool isDirExist(const std::string& path_name);
bool createNewDir(const std::string& path_name);
bool isFileExist(const std::string& file_name);
bool createNewFile(const std::string& file_name);

bool computePlaneModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud_ptr,
                       pcl::ModelCoefficients::Ptr plane_coeff_ptr);
void computeCloudNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                        pcl::PointCloud<pcl::Normal>::Ptr normal_cloud_ptr);
bool computePlaneModelNormal(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud_ptr,
                             pcl::ModelCoefficients::Ptr plane_coeff_ptr);

void projectToPlane(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                    const pcl::ModelCoefficients::Ptr plane_coeff_ptr,
                    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud_ptr);

bool computeLineModel(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr, Eigen::VectorXf& line_coeff);
bool seperateEdgeCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
                       std::vector<Eigen::VectorXf>& v_line_coeff,
                       std::vector<pcl::PointCloud<pcl::PointXYZ>>& v_line_cloud);
void orderLineCloud(std::vector<Eigen::VectorXf>& v_line_coeff,
                    std::vector<pcl::PointCloud<pcl::PointXYZ>>& v_line_cloud);
bool computeLinesIntersect(const Eigen::VectorXf& line_coeff_1, const Eigen::VectorXf& line_coeff_2,
                           pcl::PointXYZ& intersect_point);

void pose2Transform(const std::vector<double>& pose, Eigen::Matrix4d& transform);
void transform2Pose(const Eigen::Matrix4d& transform, std::vector<double>& pose);

void projectPoint(const pcl::PointXYZ& point_3d, const cv::Mat& camera_mat, cv::Point& point_2d);

void printCalibration(const std::vector<double>& pose, const Eigen::Matrix4d& trans);
void saveYamlFile(const std::string file_path, const aruco::CameraParameters& cam_param,
                  const std::vector<double>& pose, const Eigen::Matrix4d& transformation);
}  // namespace cicv

#endif
