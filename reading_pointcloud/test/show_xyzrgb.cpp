// Project      : reading_pointcloud
// File         : show_xyz.cpp
// Author       : bss
// Creation Date: 2014-07-12
// Last modified: 2014-07-12, 13:03:09
// Description  : show ros-style pointcloud.
// 

#include <stdio.h>
#include <string>
#include <ros/ros.h>
#include <reading_pointcloud/reading_pointcloud.h>
#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_sub_cloud_xyzrgb");
    ros::NodeHandle n;
    ros::Rate rate(33);

    pcl::visualization::PCLVisualizer viewer("PCL Viewer");

    CloudConverter* cc_ = new CloudConverter();

    while (!cc_->ready_xyzrgb_)
    {
        ros::spinOnce();
        rate.sleep();
        if (!ros::ok())
        {
            printf("Terminated by C-c.\n");
            return -1;
        }
    }

    viewer.setCameraPosition(0, 0, -2, 0, -1, 0, 0);

    while (ros::ok())
    {
        viewer.removeAllPointClouds();
        viewer.removeAllShapes();
        pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr msg = cc_->msg_xyzrgb_;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
        printf ("%s, Cloud: width = %d, height = %d\n", cloud->header.frame_id.c_str(), cloud->width, cloud->height);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer.addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "input_cloud");

        rate.sleep();
    }

    delete cc_;
    cc_ = NULL;

    return 0;
}
