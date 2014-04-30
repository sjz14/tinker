// Project:         decide
// File:            decide.h
// Created by gjq at 2014-01-15
// Last modified: 2014-03-11, 16:42:52
// Description: follow me decision node.

#ifndef _DECIDE_H_
#define _DECIDE_H_

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "reading_image.h"
#include "frmsg/box.h"
#include "frmsg/dec.h"

class ImageConverter;

class Decide
{
public:
	Decide();
	~Decide();

	bool Init(int argc, char** argv);
	void Run();
private:
    void decidecallback(const frmsg::box::ConstPtr& msg);

	ros::NodeHandle* n_;
	ros::Rate* rate_;
	ros::Publisher dec_;
	ros::Subscriber box_;

	double neardep_;
	ImageConverter* ic_;
	cv::Mat depth_;
	cv::FileStorage* fs_;
	
	struct Setting
	{
	    int far_dis;
	    int near_dis;
	    int left_dis;
	    int right_dis;
        double speed_ratio;
		int speed_basis;
        int mode;
	};
	Setting set_;

    std::string package_path_;
    
	int msg_;
	int origin_width_;
	int origin_height_;
	bool init_;
	
	int origin_decision_;
};

#endif
