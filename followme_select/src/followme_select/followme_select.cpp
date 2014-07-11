// Project:     followme_select
// File:        followme_select.cpp
// Created by bss at 2013-12-21
// Last modified: 2014-07-11, 10:47:58
// Description: 

#include "followme/followme.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include "reading_image/reading_image.h"
#include "macro.h"
#include "OpenTLD/TLD.h"
#include "followme/fm_utils.h"
#include "frmsg/box.h"
#include "frmsg/bindingBox.h"

const char FollowmeDN::WINDOW_NAME[] = "followme image, select";

FollowmeDN::FollowmeDN()
    : n_(NULL)
    , rate_(NULL)
    , ic_(NULL)
    , tld_(NULL)
    , box_()
    , last_gray_()
    , curr_gray_()
    , fs_(NULL)
    , pub_()
    , fsmy_(NULL)
    , set_()
    , package_path_("./")
    , cmdstring()
{
}

FollowmeDN::~FollowmeDN()
{
    SAFE_DELETE(rate_);
    SAFE_DELETE(ic_);
    SAFE_DELETE(tld_);
    SAFE_DELETE(n_);
    SAFE_DELETE(fs_);
    SAFE_DELETE(fsmy_);
}

bool FollowmeDN::Init(int argc, char** argv)
{
    ros::init(argc, argv, "followme_select");
    n_ = new ros::NodeHandle;
    rate_ = new ros::Rate(33);
    pub_ = n_->advertise<frmsg::box>("box",10);

    package_path_ = ros::package::getPath("followme") + "/";
    fs_ = new cv::FileStorage(package_path_ + "parameters.yml",
        cv::FileStorage::READ);

    fsmy_ = new cv::FileStorage(package_path_ + "myparam.yml",
        cv::FileStorage::READ);
    set_.show_pts = (int)fsmy_->getFirstTopLevelNode()["show_pts"];
    set_.show_depth = (int)fsmy_->getFirstTopLevelNode()["show_depth"];

    // init Image Converter, convert image from ros to OpenCV
    ic_ = new ImageConverter();

    while (!ic_->ready)
    {
        ros::spinOnce();
        rate_->sleep();
        if (!ros::ok())
        {
            printf("Terminated by control-C in Init.\n");
            return false;
        }
    }
    
    //delay
    ros::Duration(1.0).sleep();
    
    // init Bounding Box
    box_ = cv::Rect();
    
    ros::Publisher pub_bindBox = n_->advertise<frmsg::bindingBox>(
            "/followme/bindbox", 1000);
    frmsg::bindingBox bindBox;
    while (ros::ok())
    {
        if (!BindBox())
        {
            return false;
        }
        if (cv::min(box_.width, box_.height) <
            (int)fs_->getFirstTopLevelNode()["min_win"])
        {
            printf("Bounding box too small.\n");
        }
        else
        {
            // send to followme
            bindBox.x = box_.x;
            bindBox.y = box_.y;
            bindBox.width = box_.width;
            bindBox.height = box_.height;
            pub_bindBox.publish(bindBox);
        }
    }
    
    return true;
}

void FollowmeDN::Run()
{
}

bool FollowmeDN::BindBox()
{
    if (!ic_->ready)
    {
        printf("ImageConverter failed.\n");
        return false;
    }
    cv::Mat first;
    cv::Mat frame;
    int i = 0;
    while (i<1)
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

double FollowmeDN::GetAvgDepth(cv::Rect& pbox)
{
    cv::Mat depth;
    ic_->curr_image_depth.copyTo(depth);
    //cv::imshow("tmp",depth);
    //cv::waitKey(3);
    int count = 1;
    float tmp = 0;
    double avgdep = 0.0;
    if (!depth.data)
    {
        cmdstring.push_back("No depth data");
        return 0.0;
    }
    else
    {
        for ( int i = pbox.x; i < pbox.x + pbox.width; i++)
        {
            for ( int j = pbox.y; j < pbox.y + pbox.height; j++)
            {
                tmp = (float)depth.at<int>(i,j);
                //ROS_INFO("%f",tmp);
                if ( tmp > 1e-7)
                {
                    avgdep += tmp;
                    count++;
                }
            }
        }
        //ROS_INFO("%f",avgdep);
        avgdep /= count;
        return avgdep;
    }
    return 0.0;
}

void FollowmeDN::MakeDecision(bool is_detected, cv::Rect& pbox)
{
    if (is_detected)
    {
        if (pbox.x + pbox.width*0.5 < centerX - 40)
        {
            cmdstring.push_back("Turn left");
        }
        if (pbox.x + pbox.width*0.5 > centerX + 40)
        {
            cmdstring.push_back("Turn right");
        }
        double avg_depth = GetAvgDepth(pbox);
        std::stringstream ss;
        ss << "Average depth = " << avg_depth;
        cmdstring.push_back(ss.str());
    }
}

