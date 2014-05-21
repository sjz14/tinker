// Project      : easycap_bridge
// File         : easycap_bridge.cpp
// Creation Date: 2014-05-21
// Last modified: 2014-05-22, 00:55:43
// Description  : 转发 /dev/video? 到 ros
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
    ros::init(argc, argv, "easycap_bridge");
    ros::NodeHandle n;
    ros::Rate rate(33);
    std::string package_path =
        ros::package::getPath("easycap_bridge") + "/";

    image_transport::ImageTransport it(n);
    image_transport::Publisher image_pub_left =
        it.advertise("/binocular/left", 1);
    image_transport::Publisher image_pub_right =
        it.advertise("/binocular/right", 1);

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

    char video_name_b[64];
    int id_b = (int)fs.getFirstTopLevelNode()["video_b"];
    sprintf(video_name_b, "/dev/video%d", id_b);
    cv::VideoCapture cap2 = cv::VideoCapture(id_b);
    cap2.open(video_name_b);
    if (!cap2.isOpened())
    {
        printf("Failed to open %s\n", video_name_b);
    }

    cv::Mat frame1, frame2;
    //cv::namedWindow("videoa", 1);
    //cv::namedWindow("videob", 1);
    bool a_is_left = ((int)fs.getFirstTopLevelNode()["a_is_left"] != 0);

    while (ros::ok())
    {
        cv::waitKey(20);
        cap1 >> frame1;
        cap2 >> frame2;
        if (!frame1.data)
        {
            printf("channel a: no frame data\n");
            break;
        }
        if (!frame2.data)
        {
            printf("channel b: no frame data\n");
            break;
        }
        //cv::imshow("videoa", frame1);
        //cv::imshow("videob", frame2);
        cv_bridge::CvImage img1;
        img1.header.stamp = ros::Time::now();
        img1.header.frame_id = "frame_left";
        // img1.encoding = "mono8"; // 黑白
        img1.encoding = "bgr8"; // 彩色
        img1.image = frame1;

        cv_bridge::CvImage img2;
        img2.header.stamp = ros::Time::now();
        img2.header.frame_id = "frame_right";
        img2.encoding = "bgr8"; // 彩色
        img2.image = frame2;

        if (a_is_left)
        {
            image_pub_left.publish(img1.toImageMsg());
            image_pub_right.publish(img2.toImageMsg());
        }
        else
        {
            image_pub_left.publish(img2.toImageMsg());
            image_pub_right.publish(img1.toImageMsg());
        }
    }

    return 0;
}
