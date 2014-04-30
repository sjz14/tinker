// Project:     followme
// File:        followme.cpp
// Created by bss at 2013-12-21
// Last modified: 2014-04-20, 01:51:53
// Description: 

#include "followme.h"
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv2/opencv.hpp>
#include "reading_image.h"
#include "macro.h"
#include "TLD.h"
#include "fm_utils.h"
#include "frmsg/box.h"
#include "std_msgs/String.h"
#include "frmsg/bindingBox.h"

const char FollowmeDN::WINDOW_NAME[] = "followme image";
bool hasReceivedBindBox = false;
cv::Rect* bindBox = NULL;

void bindBoxCallback(const frmsg::bindingBox::ConstPtr& bindingBox)
{
    if (hasReceivedBindBox)
    {
        //return;
    }
    frmsg::bindingBox box = *bindingBox;
    printf("received bind box x=%d y=%d w=%d h=%d\n",
            box.x, box.y, box.width, box.height);
    hasReceivedBindBox = true;
    bindBox = new cv::Rect(box.x, box.y, box.width, box.height);
}

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
    ros::init(argc, argv, "followme_decision");
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
    /*box_ = cv::Rect();
    while (true)
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
            break;
        }
    }*/
    // receive from followme_select
    ros::Subscriber sub_select = n_->subscribe("/followme/bindbox", 1000,
            bindBoxCallback);
    printf("wait for bind box\n");
    rate_->sleep();
    while (!hasReceivedBindBox)
    {
        ros::spinOnce();
    }
    sub_select.shutdown();
    box_ = *bindBox;
    printf("bind box get\n");
    
    // init TLD
    tld_ = new TLD();
    tld_->read(fs_->getFirstTopLevelNode());

    cvtColor(ic_->curr_image, last_gray_, CV_RGB2GRAY);

    FILE* log_file = fopen((package_path_ + "bb_box.log").c_str(), "w");
    tld_->init(last_gray_, box_, log_file);
    fclose(log_file);

    return true;
}

void FollowmeDN::Run()
{
    cv::Mat frame;
    std::vector<cv::Point2f> pts1;
    std::vector<cv::Point2f> pts2;
    BoundingBox pbox;
    frmsg::box pub;    

    bool status = false;
    bool tl = true;
    FILE* log_file = fopen((package_path_ + "bb_box.log").c_str(), "w");
    while (ros::ok())
    {
        cmdstring.clear();
        ic_->curr_image.copyTo(frame);
        cv::cvtColor(frame, curr_gray_, CV_RGB2GRAY);
        if (set_.show_depth)
        {
            ic_->curr_image_depth.copyTo(frame);
        }
        // process frame
        tld_->processFrame(last_gray_, curr_gray_, pts1, pts2, pbox,
            status, tl, log_file);
        // draw
        if (status)
        {
            if (set_.show_pts)
            {
                drawPoints(frame, pts1);
                drawPoints(frame, pts2, cv::Scalar(0x00, 0xff, 0x00));
            }
            drawBox(frame, pbox);
            // determine
            MakeDecision(status, pbox);
            drawText(frame, cmdstring);
        }
        cv::imshow(WINDOW_NAME, frame);
        cv::waitKey(3);
        pub.l_x = pbox.x;
        pub.l_y = pbox.y;
        pub.r_x = pbox.x+pbox.width;
        pub.r_y = pbox.y+pbox.height;
        pub.catched = status;
        pub.avgdep = FollowmeDN::GetAvgDepth(pbox);
        pub_.publish(pub);
        ROS_INFO("%d,%d,%d,%d,avgdep:%f",pub.l_x,pub.l_y,pub.r_x,pub.r_y,pub.avgdep);

        // clean
        cv::swap(last_gray_, curr_gray_);
        pts1.clear();
        pts2.clear();
        ic_->ready = false;
        while (!ic_->ready && ros::ok())
        {
            ros::spinOnce();
        }
        rate_->sleep();
    }

    fclose(log_file);
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

