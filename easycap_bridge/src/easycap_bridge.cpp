// Project      : easycap_bridge
// File         : easycap_bridge.cpp
// Creation Date: 2014-05-21
// Last modified: 2014-05-21
// Description  : 转发 /dev/video? 到 ros
// 

#include <stdio.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "easycap_bridge");
    ros::NodeHandle n;
    ros::Rate rate(33);
//    std::string package_path =
//        ros::package::getPath("easycap_bridge") + "/";

    cv::VideoCapture cap1 = cv::VideoCapture(0);
    cap1.open("/dev/video1");
    if (!cap1.isOpened())
    {
        printf("Failed to open video1\n");
    }
    cv::Mat frame;
    cv::namedWindow("video", 1);

    while (ros::ok())
    {
        cv::waitKey(20);
        cap1 >> frame;
        if (!frame.data)
        {
            printf("no frame data\n");
            break;
        }
        cv::imshow("video", frame);
    }


    return 0;
}
