#include <iostream>
#include <string>
#include <stdio.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include "opencv2/imgproc.hpp"

using namespace cv;
using namespace std;

void gengerate_aruco_code(const int& marker_id)
{
  cv::Mat marker_img;
  cv::Ptr<cv::aruco::Dictionary> mdictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
  cv::aruco::drawMarker(mdictionary, marker_id, 1000, marker_img, 1);

  imshow("test", marker_img);
  waitKey();
  std::string image_name = "aruco_marker_" + std::to_string(marker_id) + ".jpg";
  imwrite(image_name, marker_img);
}
void generate_block(const std::string& image_name, const int& marker_x, const int& marker_y, const int& marker_length,
                    const int& marker_separation)
{
  cv::Mat raw_img = imread(image_name, 0);

  int channel = raw_img.channels();
  for (int i = 0; i < marker_y + 1; i++)
  {
    for (int j = 0; j < marker_x + 1; j++)
    {
      for (int row = i * (marker_length + marker_separation);
           row < i * (marker_length + marker_separation) + marker_separation; row++)
      {
        for (int col = j * (marker_length + marker_separation);
             col < j * (marker_length + marker_separation) + marker_separation; col++)
        {
          if (channel == 1)
          {
            raw_img.at<uchar>(row, col) = 255 - raw_img.at<uchar>(row, col);
          }
        }
      }
    }
  }
  // imshow("grid", raw_img);
  // waitKey();
  imwrite(image_name, raw_img);
}

void gengerate_aruco_grid_code(const int& marker_id, const int& marker_x, const int& marker_y, const int& marker_length,
                               const int& marker_separation)
{
  cv::Size imageSize;
  imageSize.width = marker_x * (marker_length + marker_separation) + marker_separation;
  imageSize.height = marker_y * (marker_length + marker_separation) + marker_separation;

  cv::Mat marker_img;
  cv::Ptr<cv::aruco::Dictionary> mdictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(marker_x, marker_y, float(marker_length),
                                                                     float(marker_separation), mdictionary, marker_id);
  board->draw(imageSize, marker_img, marker_separation, 1);

  cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
  detectorParams->minDistanceToBorder = 0;
  vector<int> ids;
  vector<vector<Point2f> > corners, rejected;
  vector<Vec3d> rvecs, tvecs;
  // detect markers and estimate pose
  cv::aruco::detectMarkers(marker_img, mdictionary, corners, ids, detectorParams, rejected);
  for (int i = 0; i < ids.size(); ++i)
  {
    std::cout << "id: " << ids.at(i) << std::endl; /* " x: "<<corners */
  }

  // imshow("grid", marker_img);
  // waitKey();
  std::string image_name = "aruco_grid_marker_" + std::to_string(marker_id) + ".jpg";
  imwrite(image_name, marker_img);
}

int main(int argc, char* argv[])
{
  if (argc < 6)
  {
    std::cerr << "Usage: " << argv[0] << " marker_id marker_num_col maker_num_row marker_size maker_separation"
              << std::endl;
    exit(1);
  }
  int marker_id = std::atoi(argv[1]);
  int marker_x = std::atoi(argv[2]);
  int marker_y = std::atoi(argv[3]);
  int marker_length = std::atoi(argv[4]);
  int marker_separation = std::atoi(argv[5]);
  std::string image_name = "aruco_grid_marker_" + std::to_string(marker_id) + ".jpg";
  gengerate_aruco_grid_code(marker_id, marker_x, marker_y, marker_length, marker_separation);
  generate_block(image_name, marker_x, marker_y, marker_length, marker_separation);

  return 0;
}