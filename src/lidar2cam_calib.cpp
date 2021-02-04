
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
#include "aruco_calib.h"
#include "lidar2cam_calib.h"

namespace cicv
{
Lidar2CamCalib::Lidar2CamCalib(const std::string& config_path, const std::string& file_name)
{
  aruco_calib_impl_.reset(new ArucoCalib(config_path, file_name));
}

Lidar2CamCalib::~Lidar2CamCalib()
{
}

int Lidar2CamCalib::process(const cv::Mat& input_image, const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud_ptr)
{
  return aruco_calib_impl_->process(input_image, input_cloud_ptr);
}

const cv::Mat& Lidar2CamCalib::getMarkerImage()
{
  return aruco_calib_impl_->getMarkerImage();
}

const pcl::PointCloud<pcl::PointXYZ>& Lidar2CamCalib::getBoardCloud()
{
  return aruco_calib_impl_->getBoardCloud();
}

const pcl::PointCloud<pcl::PointXYZ>& Lidar2CamCalib::getEdgeCloud()
{
  return aruco_calib_impl_->getEdgeCloud();
}

const pcl::PointCloud<pcl::PointXYZ>& Lidar2CamCalib::getCornerCloud()
{
  return aruco_calib_impl_->getLidarCornerCloud();
}

void Lidar2CamCalib::compute()
{
  return aruco_calib_impl_->computeTransform();
}

}  // namespace cicv
