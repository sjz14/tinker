// Project:     followme
// File:        followme.h
// Created by bss at 2013-12-22
// Last modified: 2014-03-11, 15:31:41
// Description: followme decision node.

#ifndef _FOLLOWME_H_
#define _FOLLOWME_H_

#include "ros/ros.h"
#include <opencv2/opencv.hpp>

class ImageConverter;
class TLD;

class FollowmeDN
{
public:
    FollowmeDN();
    ~FollowmeDN();

    bool Init(int argc, char** argv);
    void Run();

private:
    bool BindBox();
    double GetAvgDepth(cv::Rect& pbox);
    void MakeDecision(bool is_detected, cv::Rect& pbox);

public:
    static const char WINDOW_NAME[];
private:
    static const int centerX = 320;
    static const int centerY = 240;

    ros::NodeHandle* n_;
    ros::Rate* rate_;
    ros::Publisher pub_;

    ImageConverter* ic_;
    TLD* tld_;

    cv::Rect box_;
    cv::Mat last_gray_;
    cv::Mat curr_gray_;
    cv::FileStorage* fs_;
    cv::FileStorage* fsmy_;

    struct Setting
    {
        int show_pts;
        int show_depth;
    };
    Setting set_;

    std::string package_path_;
    std::vector<std::string> cmdstring;
};

#endif

