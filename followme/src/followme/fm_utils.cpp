// Project:     followme
// File:        fm_utils.cpp
// Created by bss at 2013-12-22
// Last modified: 2013-12-23, 21:49:51
// Description: 

#include "fm_utils.h"
#include <opencv2/opencv.hpp>

void drawText(cv::Mat& img, const std::vector<std::string>& texts)
{
    static const int dy = 20;
    cv::Point ori(5, 20);
    for (int i = 0; i < texts.size(); i++)
    {
        drawText(img, texts[i], ori);
        ori.y += dy;
    }
}

void drawText(cv::Mat& img, const std::string& text, cv::Point origin)
{
    putText(img, text, origin, CV_FONT_HERSHEY_COMPLEX,
        0.6, cv::Scalar(0xff, 0xff, 0xff));
}


/* class MouseBox */

bool MouseBox::is_drawing_box_ = false;
bool MouseBox::is_got_box_ = false;
cv::Rect MouseBox::box_ = cv::Rect();

void MouseBox::BeginDrawingBox()
{
    is_drawing_box_ = false;
    is_got_box_ = false;
    box_ = cv::Rect();
}

bool MouseBox::IsGotBox()
{
    return is_got_box_;
}

void MouseBox::MouseHandler(
    int event, int x, int y, int flags, void* param)
{
    switch (event)
    {
    case CV_EVENT_MOUSEMOVE:
        if (is_drawing_box_)
        {
            box_.width = x - box_.x;
            box_.height = y - box_.y;
        }
        break;
    case CV_EVENT_LBUTTONDOWN:
        is_drawing_box_ = true;
        box_ = cv::Rect(x, y, 0, 0);
        break;
    case CV_EVENT_LBUTTONUP:
        is_drawing_box_ = false;
        if (box_.width < 0)
        {
            box_.x += box_.width;
            box_.width = -box_.width;
        }
        if (box_.height < 0)
        {
            box_.x += box_.height;
            box_.height = -box_.height;
        }
        is_got_box_ = true;
        break;
    default:
        break;
    }
}

cv::Rect MouseBox::GetBox()
{
    return box_;
}

