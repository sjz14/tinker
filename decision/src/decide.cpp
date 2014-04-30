// Project:         decide
// File:            decide.cpp
// Created by gjq at 2014-01-15
// Last modified: 2014-03-12, 21:40:11
// Description:

#include "TLD.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <ros/package.h>
#include "reading_image.h"
#include "decide.h"
#include <sstream>
//#include "frmsg/box.h"

Decide::Decide()
	:n_(NULL)
	,rate_(NULL)
	,dec_()
	,neardep_()
	,ic_(NULL)
	,depth_()
	,msg_()
	,box_()
	,fs_(NULL)
	,package_path_("./")
	,init_(true)
	,origin_height_(0)
	,origin_width_(0)
{
}

Decide::~Decide()
{
//	SAFE_DELETE(rate_);
//	SAFE_DELETE(ic_);
//	SAFE_DELETE(n_);
//	SAFE_DELETE(depth_);
//	SAFE_DELETE(dec_);
}

bool Decide::Init(int argc, char** argv)
{
	ros::init(argc, argv, "decision");
	n_ = new ros::NodeHandle;
	rate_ = new ros::Rate(33);
    dec_ = n_->advertise<frmsg::dec>("dec",10);
    
    package_path_ = ros::package::getPath("decision") + "/";
    fs_ = new cv::FileStorage(package_path_ + "parameters.yml", cv::FileStorage::READ);
    set_.far_dis = (int)fs_->getFirstTopLevelNode()["far_dis"];
    set_.near_dis = (int)fs_->getFirstTopLevelNode()["near_dis"];
    set_.left_dis = (int)fs_->getFirstTopLevelNode()["left_dis"];
    set_.right_dis = (int)fs_->getFirstTopLevelNode()["right_dis"];
    set_.mode = (int)fs_->getFirstTopLevelNode()["mode"];
    set_.speed_ratio = (double)fs_->getFirstTopLevelNode()["speed_ratio"];
	set_.speed_basis = (int)fs_->getFirstTopLevelNode()["speed_basis"];

	ic_ = new ImageConverter();
	
	while ( !ic_->ready )
	{
		ros::spinOnce();
		rate_->sleep();
		if (!ros::ok())
		{
            printf("Terminated by control-C in Init.\n");
            return false;
        }
    }
	
	return true;
}

void Decide::decidecallback(const frmsg::box::ConstPtr& msg)
{
    ROS_INFO("The box I subscirbed is %d,%d,%d,%d",msg->l_x,msg->l_y,msg->r_x,msg->r_y);
    int decision = 0;
    ic_->curr_image_depth.copyTo(depth_);
    frmsg::dec dec;
    neardep_ = msg->avgdep;
    ROS_INFO("the average depth is %f",neardep_);
    if ( msg->catched == true )
    {
        if ( set_.mode == 0 )
        {
            if ( neardep_ != 0 )
            {
                if ( neardep_ > set_.far_dis )
                    decision = 1;
                else
                    decision = 0;
            }
            msg_ = (msg->l_x + msg->r_x) / 2;
            if ( msg_ < set_.left_dis )
                decision = 3;
            else if ( msg_ > set_.right_dis )
                decision = 4;
            if ( init_ )
            {
                origin_width_ = msg->r_x - msg->l_x;
                origin_height_ = msg->r_y - msg->l_y;
                init_ = false;
            }
            else
            {
                if ( (msg->r_x - msg->l_x) * (msg->r_y - msg->l_y ) < origin_width_ * origin_height_ * 0.5 )
                    decision = 5;
                else if ( (msg->r_x - msg->l_x) * (msg->r_y - msg->l_y ) > origin_width_ * origin_height_ * 2 )
                    if (( decision != 3) && (decision != 4) )
                         decision = 0;
            }
        }
        else if ( set_.mode == 1 )
        {
            //direc: 0 is for stop;
            //1 is for forward;
            //2 is for right;
            //3 is for left;
            msg_ = (msg->l_x + msg->r_x) / 2;
            if ( msg_ > set_.right_dis )
            {
                dec.direc = 2;
                dec.speed = set_.speed_ratio * (double)(neardep_ - set_.far_dis + 10) + set_.speed_basis;
                dec.radius = neardep_;
            }
            else if ( msg_ < set_.left_dis )
            {
                dec.direc = 3;
                dec.speed = set_.speed_ratio * (double)(neardep_ - set_.far_dis + 10) + set_.speed_basis;
                dec.radius = neardep_;
            }
            else if ( neardep_ > set_.far_dis )
            {
                dec.direc = 1;
                dec.speed = set_.speed_ratio * (double)(neardep_ - set_.far_dis + 10) + set_.speed_basis;
                dec.radius = 0;
            }
            else
            {
                dec.direc = 0;
                dec.speed = 0;
                dec.radius = 0;
            }
        }
        dec.mode = set_.mode;
   }
    else
    {
        if ( origin_decision_ == 1 )
            decision = 0;
        else
            decision = origin_decision_;
    }
    dec.dec = decision;
    origin_decision_ = decision;
    dec_.publish(dec);
    ROS_INFO("The decision is %d,mode:%d,direc:%d,speed:%f",decision,dec.mode,dec.direc,dec.speed);
    ros::spinOnce();
    rate_->sleep();
    //PublicDecision(decision);
}

void Decide::Run()
{
    box_ = n_->subscribe("box", 10, &Decide::decidecallback,this);
    ros::spin();
}
