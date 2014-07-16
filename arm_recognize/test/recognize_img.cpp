// Project      : arm_recognize
// File         : recognize_img.cpp
// created at 2014-07-10
// Last modified: 2014-07-11, 14:06:27

#include <stdlib.h>
#include <iostream>
#include <string>
#include <ros/package.h>

//#include <XnCppWrapper.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace std;
using namespace cv;

int r_low, r_high, g_low, g_high, b_low, b_high;

void handRec(Mat I)
{
    Mat img(I);
    vector<Mat> CC;
    split(img, CC);

    Mat mask, mask1;
    int max_BINARY_value = 255;
    int th;

    // R
    th = r_low;
    threshold(CC[2], mask, th, max_BINARY_value, THRESH_BINARY);
    th = r_high;
    threshold(CC[2], mask1, th, max_BINARY_value, THRESH_BINARY_INV);
    bitwise_and(mask, mask1, mask);
    
    // G(上下限)
    th = g_low;
    threshold(CC[1], mask1, th, max_BINARY_value, THRESH_BINARY);
    bitwise_and(mask, mask1, mask);
    th = g_high;
    threshold(CC[1], mask1, th, max_BINARY_value, THRESH_BINARY_INV);
    bitwise_and(mask, mask1, mask);
    
    // B
    th = b_low;
    threshold(CC[0], mask1, th, max_BINARY_value, THRESH_BINARY);
    bitwise_and(mask, mask1, mask);
    th = b_high;
    threshold(CC[0], mask1, th, max_BINARY_value, THRESH_BINARY_INV);
    bitwise_and(mask, mask1, mask);

    namedWindow("1",1);

    Mat bw;
    dilate(mask, bw, Mat());    // 膨胀
    imshow("1", bw);

    vector<vector<Point> > contours;
    findContours(bw, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    
    for (int i = 0; i<(int)contours.size(); i++)
    {
        Moments mom = moments(Mat(contours[i]),TRUE);
        circle(img,Point((int)(mom.m10/mom.m00),(int)(mom.m01/mom.m00)),2,Scalar(1),2); 
    }
 
     namedWindow("2",1);
     imshow("2",img);  
}

int main()
{
    std::string package_path = ros::package::getPath("arm_recognize") + "/";
    cv::FileStorage* fs_ = new cv::FileStorage(package_path + "myparam.yml", cv::FileStorage::READ);

    r_low = (int)fs_->getFirstTopLevelNode()["th_r_low"];
    r_high = (int)fs_->getFirstTopLevelNode()["th_r_high"];
    g_low = (int)fs_->getFirstTopLevelNode()["th_g_low"];
    g_high = (int)fs_->getFirstTopLevelNode()["th_g_high"];
    b_low = (int)fs_->getFirstTopLevelNode()["th_b_low"];
    b_high = (int)fs_->getFirstTopLevelNode()["th_b_high"];

    delete fs_;
    fs_ = NULL;

    for (int i = 101; i<=102; i++)
    {
        char fName[20];
        sprintf(fName, "%simages/RGB%d.jpg", package_path.c_str(),  i);
        Mat img = imread(fName);
        cvNamedWindow("0");
        imshow("0",img);
        handRec(img);
        cvWaitKey(0);
    }
}


