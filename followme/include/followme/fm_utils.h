// Project:     followme
// File:        fm_utils.h
// Created by bss at 2013-12-22
// Last modified: 2013-12-23, 21:49:54
// Description: 

#ifndef _FM_UTILS_H_
#define _FM_UTILS_H_

#include <opencv2/opencv.hpp>

void drawText(cv::Mat& img, const std::vector<std::string>& texts);
void drawText(cv::Mat& img, const std::string& text,
    cv::Point origin = cv::Point(5, 20));

class MouseBox
{
public:
    static void BeginDrawingBox();
    static bool IsGotBox();
    static void MouseHandler(
        int event, int x, int y, int flags, void* param);
    static cv::Rect GetBox();

private:
    static bool is_drawing_box_;
    static bool is_got_box_;
    static cv::Rect box_;
};

#endif

