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
#ifndef RSLIDAR2CAMCALIB_H
#define RSLIDAR2CAMCALIB_H

#include <memory>

#include <opencv2/core/mat.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "prompt.h"

namespace cicv
{
class ArucoCalib;
class Lidar2CamCalib
{
public:
  Lidar2CamCalib(const std::string& config_path, const std::string& file_name);
  ~Lidar2CamCalib();

  int process(const cv::Mat& input_image, const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr);
  void compute();
  const cv::Mat& getMarkerImage();
  const pcl::PointCloud<pcl::PointXYZ>& getBoardCloud();
  const pcl::PointCloud<pcl::PointXYZ>& getEdgeCloud();
  const pcl::PointCloud<pcl::PointXYZ>& getCornerCloud();

private:
  std::unique_ptr<ArucoCalib> aruco_calib_impl_;
};
}  // namespace cicv

#endif
