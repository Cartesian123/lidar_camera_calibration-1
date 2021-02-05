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

#ifndef MONO_PATTERN_H
#define MONO_PATTERN_H

#include <aruco/aruco.h>

#include "common.h"

namespace cicv
{
class MonoPattern
{
public:
  MonoPattern(const std::string& config_path, const std::string& file_name);
  ~MonoPattern();

  bool computeMarkerBoardPose(cv::Mat& input_image, cv::Mat& transform_matrix);
  void computeCornerCloud(const cv::Mat& transform_matrix, pcl::PointCloud<pcl::PointXYZ>::Ptr corner_cloud);

  aruco::CameraParameters cam_param_;
  std::vector<double> marker_length_;

private:
  bool checkIds(const std::vector<aruco::Marker>& markers);
  bool readConfigFile(const std::string& config_path, const std::string& config_file_name);
  void getOrderedCorner(const cv::Mat& cam_corners, pcl::PointCloud<pcl::PointXYZ>::Ptr corner_cloud);

  aruco::MarkerMap marker_map_;
  aruco::MarkerMapPoseTracker marker_map_tracker_;
  aruco::MarkerDetector marker_detector_;
  std::vector<int> marker_ids_;
};
}  // namespace cicv

#endif
