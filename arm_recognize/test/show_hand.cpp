// Project      : arm_recognize
// File         : main2.cpp
// created at 2014-07-10
// Last modified: 2014-07-11, 12:22:06

#include <stdlib.h>
#include <iostream>
#include <string>
#include <ros/ros.h>

//#include <XnCppWrapper.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "reading_image/reading_image.h"

using namespace std;
using namespace cv;

ImageConverter* ic_ = NULL;

void handRec(Mat I)
{
    Mat img(I);
    vector<Mat> CC;
    split(img, CC);

    Mat mask, mask1;
    int th = 230;
    int max_BINARY_value = 255;
    threshold(CC[2], mask, th, max_BINARY_value, THRESH_BINARY);
    
    th = 130;
    threshold(CC[1], mask1, th, max_BINARY_value, THRESH_BINARY);
    bitwise_and(mask, mask1, mask);
    th = 200;
    threshold(CC[1], mask1, th, max_BINARY_value, THRESH_BINARY_INV);
    bitwise_and(mask, mask1, mask);
    
    th = 130;
    threshold(CC[0], mask1, th, max_BINARY_value, THRESH_BINARY);
    bitwise_and(mask, mask1, mask);
    th = 200;
    threshold(CC[0], mask1, th, max_BINARY_value, THRESH_BINARY_INV);
    bitwise_and(mask, mask1, mask);

    namedWindow("1",1);

    Mat bw;
    dilate(mask, bw, Mat());
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
    ros::Rate rate(3);
    ic_ = new ImageConverter();

    // Init
    while (!ic_->ready)
    {
        ros::spinOnce();
        rate.sleep();
        if (!ros::ok())
        {
            printf("Terminated by C-c when init.\n");
            return -1;
        }
    }

    // Loop
    Mat img;
    while (ros::ok())
    {
        ic_->curr_image.copyTo(img);
        cvNamedWindow("0");
        imshow("0",img);
        handRec(img);

        cvWaitKey(0);
        while (!ic_->ready && ros::ok())
        {
            ros::spinOnce();
        }
        rate.sleep();
    }
    delete ic_;
    ic_ = NULL;
}


