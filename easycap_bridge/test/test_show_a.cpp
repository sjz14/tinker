// Project      : easycap_bridge
// File         : test_show_a.cpp
// Creation Date: 2014-05-21
// Last modified: 2014-05-21
// Description  : 
// 

#include <stdio.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <string>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_show_a");
    ros::NodeHandle n;
    ros::Rate rate(33);
    std::string package_path =
        ros::package::getPath("easycap_bridge") + "/";

    cv::FileStorage fs(package_path + "param.yml", cv::FileStorage::READ);

    char video_name_a[64];
    int id_a = (int)fs.getFirstTopLevelNode()["video_a"];
    sprintf(video_name_a, "/dev/video%d", id_a);
    cv::VideoCapture cap1 = cv::VideoCapture(id_a);
    cap1.open(video_name_a);
    if (!cap1.isOpened())
    {
        printf("Failed to open %s\n", video_name_a);
    }

    cv::Mat frame1;
    cv::namedWindow("video_a", 1);

    while (ros::ok())
    {
        cv::waitKey(20);
        cap1 >> frame1;
        if (!frame1.data)
        {
            printf("channel a: no frame data\n");
            break;
        }
        cv::imshow("video_a", frame1);
    }


    return 0;
}
