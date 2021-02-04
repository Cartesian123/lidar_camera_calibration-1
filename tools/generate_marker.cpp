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
	cv::Mat markerImage;
	cv::Ptr<cv::aruco::Dictionary> mdictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_1000);
	cv::aruco::drawMarker(mdictionary, marker_id, 1000, markerImage, 1);

	imshow("test", markerImage);
	waitKey();
    std::string image_name = "aruco_marker_"+std::to_string(marker_id)+".jpg";
	imwrite(image_name, markerImage);
}

void gengerate_aruco_grid_code(const int& marker_id)
{
    int markersX = 2;
    int markersY = 2;
    int markerLength = 800;
    int markerSeparation = 200;
    int margins = 200;

    cv::Size imageSize;
    imageSize.width = markersX * (markerLength + markerSeparation) - markerSeparation + 2 * margins;
    imageSize.height = markersY * (markerLength + markerSeparation) - markerSeparation + 2 * margins;

	cv::Mat markerImage;
	cv::Ptr<cv::aruco::Dictionary> mdictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(markersX, markersY, float(markerLength), float(markerSeparation), mdictionary,marker_id);
	board->draw(imageSize, markerImage, margins, 1);
    //fill black block
    int height = markerImage.rows;
    int width = markerImage.cols;
    int channel = markerImage.channels();
    // std::cout<<"img height: "<<height<<" width: "<<width<<" channel: "<<channel<<std::endl;
    for(int i = 0; i < markersX+1;i++)
    {
        for(int j = 0; j <markersY+1;j++)
        {
            for (int row = i*(markerLength+markerSeparation); row < i*(markerLength+markerSeparation)+markerSeparation; row++) 
            {
                for (int col = j*(markerLength+markerSeparation); col < j*(markerLength+markerSeparation)+markerSeparation; col++)
                {
                    if (channel == 1)
                    {
                        markerImage.at<uchar>(row, col) = 255 - markerImage.at<uchar>(row, col);
                    }
                }
            }
        }
    }
	imshow("grid", markerImage);
	waitKey();
    std::string image_name = "aruco_grid_marker_"+std::to_string(marker_id)+".jpg";
	imwrite(image_name, markerImage);
}

int main(int argc, char* argv[])
{
    if(argc<2)
    {
     std::cerr<<"Usage: "<< argv[0] <<" marker_id"<<std::endl;
     exit(1);
    }
    int marker_id = std::atoi(argv[1]);
	gengerate_aruco_grid_code(marker_id);
	return 0;
}