// Project      : easycap_bridge
// File         : test_send_a.cpp
// Creation Date: 2014-05-22
// Last modified: 2014-05-22, 00:49:24
// Description  : 
// 

#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_send_a");
    ros::NodeHandle n;
    ros::Rate rate(33);
    std::string package_path =
        ros::package::getPath("easycap_bridge") + "/";

    image_transport::ImageTransport it(n);
    image_transport::Publisher image_pub =
        it.advertise("/binocular/test_a", 1);

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
    //cv::namedWindow("video_a", 1);

    while (ros::ok())
    {
        cv::waitKey(20);
        cap1 >> frame1;
        if (!frame1.data)
        {
            printf("channel a: no frame data\n");
            break;
        }
        //cv::imshow("video_a", frame1);
        cv_bridge::CvImage img1;
        img1.header.stamp = ros::Time::now();
        img1.header.frame_id = "frame_left";
        img1.encoding = "bgr8";
        cv_bridge::CvImage img_b;
        img1.image = frame1;
        image_pub.publish(img1.toImageMsg());
    }


    return 0;
}
