// Created by bss at 2014-04-19
// Last modified: 2014-04-20, 00:33:33

#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include "reading_image.h"
#include "fm_utils.h"
#include "TLD.h"
#include "std_msgs/String.h"
#include <string.h>
#include <stdio.h>
#include "frmsg/bindingBox.h"

ImageConverter* ic_;
ros::NodeHandle* n_;
ros::Rate* rate_;
cv::Rect box_;
cv::Mat last_gray_;
cv::Mat curr_gray_;

bool BindBox();
int main(int argc, char** argv);
void followmeCmdCallback(const std_msgs::String& msg);

bool BindBox()
{
    static const char WINDOW_NAME[] = "Select bind box";
    if (!ic_->ready)
    {
        printf("ImageConverter failed.\n");
        return false;
    }
    cv::Mat first;
    cv::Mat frame;
    int i = 0;
    while (i<50)
    {
        std::cout<<i<<std::endl;
        if (ic_->ready) 
        {   
            ic_->curr_image.copyTo(first);
            cv::imshow(WINDOW_NAME, first);
            cv::waitKey(3);
            i++;
            ic_->ready = false;
            while (!ic_->ready && ros::ok())
            {
                ros::spinOnce();
            }
            rate_->sleep();
        }
        
    }
    MouseBox::BeginDrawingBox();
    cvSetMouseCallback(WINDOW_NAME, MouseBox::MouseHandler, NULL);

    while (!MouseBox::IsGotBox())
    {
        first.copyTo(frame);
        drawText(frame, "Please select bounding box");
        drawBox(frame, MouseBox::GetBox());
        cv::imshow(WINDOW_NAME, frame);
        cv::waitKey(3);
        rate_->sleep();
        if (!ros::ok())
        {
            printf("Terminated by control-C in BindBox.\n");
            return false;
        }
    }
    box_ = MouseBox::GetBox();

    cvSetMouseCallback(WINDOW_NAME, NULL, NULL);

    return true;
}

void followmeCmdCallback(const std_msgs::String::ConstPtr& msg)
{
    if (0 == strcmp(msg->data.c_str(), "select"))
    {
        printf("hahaha\n");
        BindBox();
    }
}

int main(int argc, char** argv)
{
    printf("ahahah\n");
    ros::init(argc, argv, "followme_select");
    n_ = new ros::NodeHandle();
    rate_ = new ros::Rate(33);
    ic_ = new ImageConverter();
    ros::Subscriber subs = n_->subscribe("/followme/select", 1000, followmeCmdCallback);

    ros::Publisher pub_bindBox = n_->advertise<frmsg::bindingBox>(
            "/followme/bindbox", 1000);
    frmsg::bindingBox bindBox;
    while (ros::ok())
    {
        bindBox.x = 0;
        bindBox.y = 0;
        bindBox.width = 100;
        bindBox.height =100;
        pub_bindBox.publish(bindBox);
    }

    return 0;
}
