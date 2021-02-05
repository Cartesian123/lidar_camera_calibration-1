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

#ifndef LASER_PATTERN
#define LASER_PATTERN

#include "common.h"

namespace cicv
{
class LaserPattern
{
public:
  LaserPattern(const std::string& config_path, const std::string& file_name);
  ~LaserPattern();

  bool extractEdgeCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr board_cloud_ptr,
                        pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud_ptr);
  bool computeCloudCornersAuto(const pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud_ptr,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr corner_cloud_ptr);

  float board_length_;

private:
  bool readConfigFile(const std::string& config_file_name);
  std::vector<double> cloud_roi_;
  double square_dist_thres_;
};
}  // namespace cicv

#endif
