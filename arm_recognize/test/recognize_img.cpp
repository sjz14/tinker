// Project      : arm_recognize
// File         : recognize_img.cpp
// created at 2014-07-10
// Last modified: 2014-07-11, 11:55:47

#include <stdlib.h>
#include <iostream>
#include <string>
#include <ros/ros.h>

//#include <XnCppWrapper.h>
#include "opencv/cv.h"
#include "opencv/highgui.h"

using namespace std;
using namespace cv;

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
    for (int i = 101; i<=102; i++)
    {
        char fName[20];
        sprintf(fName, "images/RGB%d.jpg", i);
        Mat img = imread(fName);
        cvNamedWindow("0");
        imshow("0",img);
        handRec(img);
        cvWaitKey(0);
    }
}


