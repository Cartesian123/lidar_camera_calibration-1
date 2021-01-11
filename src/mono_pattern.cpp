
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
#include <boost/filesystem.hpp>

#include <opencv2/core/persistence.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "mono_pattern.h"

namespace cicv
{
MonoPattern::MonoPattern(const std::string& config_path, const std::string& file_name)
{
  // read config file
  if (!readConfigFile(config_path, file_name))
  {
    return;
  }

  aruco::MarkerDetector::Params& detector_params = marker_detector_.getParameters();
  detector_params.detectEnclosedMarkers(true);
  detector_params.setCornerRefinementMethod(aruco::CORNER_LINES);
  marker_detector_.setDetectionMode(aruco::DM_FAST);
  marker_detector_.setDictionary(marker_map_.getDictionary(), 0.3);

  marker_map_tracker_.setParams(cam_param_, marker_map_);
}

MonoPattern::~MonoPattern()
{
}

bool MonoPattern::readConfigFile(const std::string& config_path, const std::string& config_file_name)
{
  std::string file_name = config_path + "/" + config_file_name;
  cv::FileStorage fs_reader(file_name, cv::FileStorage::READ);
  if (!fs_reader.isOpened())
  {
    WARN << file_name << " is wrong!" << REND;
    return false;
  }
  std::string cam_intrinsic_file;
  fs_reader["cam_intrinsic_file"] >> cam_intrinsic_file;
  std::string cam_intrinsic_name = config_path + "/" + cam_intrinsic_file;
  INFO << "camera intrinsic file: " << cam_intrinsic_file << REND;
  fs_reader["marker_length"] >> marker_length_;
  std::string marker_map_file;
  fs_reader["marker_map_file"] >> marker_map_file;
  marker_map_file = config_path + "/" + marker_map_file;
  INFO << "marker map file: " << marker_map_file << REND;
  fs_reader.release();

  // config check
  if (!isFileExist(cam_intrinsic_name))
  {
    WARN << "camera intrinsic file don't exist: " << cam_intrinsic_file << REND;
    return false;
  }
  if (!readCamParam(cam_intrinsic_name, cam_param_))
  {
    return false;
  }
  if (marker_length_.size() != 2 || marker_length_[0] <= 0 || marker_length_[1] <= 0)
  {
    WARN << "marker length value is empty!" << REND;
    return false;
  }
  if (!isFileExist(marker_map_file))
  {
    WARN << "marker map file don't exist!" << REND;
    return false;
  }

  marker_map_.readFromFile(marker_map_file);
  if (marker_map_.isExpressedInPixels())
  {
    marker_map_ = marker_map_.convertToMeters(marker_length_[0]);
  }

  marker_ids_.clear();
  marker_map_.getIdList(marker_ids_);
  INFO << "marker id: ";
  for (auto marker_id : marker_ids_)
  {
    INFO << marker_id << " ";
  }
  INFO << REND;

  return true;
}

bool MonoPattern::checkIds(const std::vector<aruco::Marker>& markers)
{
  if (markers.size() == 0)
  {
    return false;
  }
  int detected_marker_num = 0;
  for (auto& marker : markers)
  {
    for (auto target_id : marker_ids_)
    {
      if (marker.id == target_id)
      {
        ++detected_marker_num;
        break;
      }
    }
  }

  if (detected_marker_num < marker_ids_.size() * 0.4)
  {
    WARN << "detected marker num is: " << detected_marker_num << REND;
    return false;
  }
  return true;
}

bool MonoPattern::computeMarkerBoardPose(cv::Mat& input_image, cv::Mat& transform_matrix)
{
  // detect markers
  std::vector<aruco::Marker> markers = marker_detector_.detect(input_image);

#if RSDEBUG
  INFO << "detected marker num: " << markers.size() << REND;
#endif

#if RSDEBUG
  INFO << "detect marker id: ";
#endif
  for (auto& marker : markers)
  {
#if RSDEBUG
    INFO << marker.id << " ";
#endif
    marker.draw(input_image, cv::Scalar(0, 0, 255), 2);
  }
#if RSDEBUG
  INFO << REND;
#endif

  if (marker_map_tracker_.estimatePose(markers))
  {
    aruco::CvDrawingUtils::draw3dAxis(input_image, cam_param_, marker_map_tracker_.getRvec(),
                                      marker_map_tracker_.getTvec(), marker_length_[0]);
    transform_matrix = marker_map_tracker_.getRTMatrix();  // cv_32F, float
  }

  if (markers.size() < marker_map_.size() * 0.4 || transform_matrix.data == nullptr)
  {
    return false;
  }
  return true;
}

void MonoPattern::computeCornerCloud(const cv::Mat& transform_matrix, pcl::PointCloud<pcl::PointXYZ>::Ptr corner_cloud)
{
  // Eigen::Matrix4d marker_board_corner;
  // original as the first point, all points are in clockwise order.
  // marker_board_corner << -marker_length_[1], -marker_length_[1], 0, 1, -marker_length_[1],
  // 2.25 * marker_length_[0] + marker_length_[1], 0, 1, 2.25 * marker_length_[0] + marker_length_[1],
  // 2.25 * marker_length_[0] + marker_length_[1], 0, 1, 2.25 * marker_length_[0] + marker_length_[1],
  //-marker_length_[1], 0, 1;

  // Eigen::Matrix4d board_corner = t * (marker_board_corner.transpose());

  // original as the first point, all points are in clockwise order.
  cv::Mat board_corners(4, 4, transform_matrix.type(), cv::Scalar::all(1));
  board_corners.at<float>(0, 0) = -marker_length_[1];
  board_corners.at<float>(1, 0) = -marker_length_[1];
  board_corners.at<float>(2, 0) = 0;

  board_corners.at<float>(0, 1) = -marker_length_[1];
  board_corners.at<float>(1, 1) = 2.25 * marker_length_[0] + marker_length_[1];
  board_corners.at<float>(2, 1) = 0;

  board_corners.at<float>(0, 2) = 2.25 * marker_length_[0] + marker_length_[1];
  board_corners.at<float>(1, 2) = 2.25 * marker_length_[0] + marker_length_[1];
  board_corners.at<float>(2, 2) = 0;

  board_corners.at<float>(0, 3) = 2.25 * marker_length_[0] + marker_length_[1];
  board_corners.at<float>(1, 3) = -marker_length_[1];
  board_corners.at<float>(2, 3) = 0;
  cv::Mat cam_corners = transform_matrix * board_corners;
  getOrderedCorner(cam_corners, corner_cloud);
}

void MonoPattern::getOrderedCorner(const cv::Mat& cam_corners, pcl::PointCloud<pcl::PointXYZ>::Ptr corner_cloud)
{
  corner_cloud->clear();
  // camera coordinate: x axis points to right, y axis points to down, z axis points to front which is vertical to x-y
  // plane. So the top point's y is smallest.
  double min_y = cam_corners.at<float>(1, 0);
  int min_y_pos = 0;
  for (int i = 1; i < 4; ++i)
  {
    if (cam_corners.at<float>(1, i) < min_y)
    {
      min_y = cam_corners.at<float>(1, i);
      min_y_pos = i;
    }
  }
  for (int i = 0; i < 4; ++i)
  {
    int cur_pos = (i + min_y_pos) % 4;
    corner_cloud->points.emplace_back(cam_corners.at<float>(0, cur_pos), cam_corners.at<float>(1, cur_pos),
                                      cam_corners.at<float>(2, cur_pos));
  }
}
}  // namespace cicv
