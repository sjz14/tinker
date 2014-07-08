// Project      : easycap_bridge
// File         : test_show_b.cpp
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
    ros::init(argc, argv, "test_show_b");
    ros::NodeHandle n;
    ros::Rate rate(33);
    std::string package_path =
        ros::package::getPath("easycap_bridge") + "/";

    cv::FileStorage fs(package_path + "param.yml", cv::FileStorage::READ);

    char video_name_b[64];
    int id_b = (int)fs.getFirstTopLevelNode()["video_b"];
    sprintf(video_name_b, "/dev/video%d", id_b);
    cv::VideoCapture cap2 = cv::VideoCapture(id_b);
    cap2.open(video_name_b);
    if (!cap2.isOpened())
    {
        printf("Failed to open %s\n", video_name_b);
    }

    cv::Mat frame2;
    cv::namedWindow("video_b", 1);

    while (ros::ok())
    {
        cv::waitKey(20);
        cap2 >> frame2;
        if (!frame2.data)
        {
            printf("channel b: no frame data\n");
            break;
        }
        cv::imshow("video_b", frame2);
    }


    return 0;
}
