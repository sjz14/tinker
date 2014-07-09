// Project      : openni_pcl_grabber
// File         : show_pts.cpp
// Creation Date: 2014-07-09
// Last modified: 2014-07-09, 03:04:49
// Description  : 显示 openni 中提取的 pcl
// 

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

// PCL viewer //
pcl::visualization::PCLVisualizer viewer("PCL Viewer");

// Mutex: //
boost::mutex cloud_mutex;

void cloud_cb_(const PointCloudT::ConstPtr& callback_cloud, PointCloudT::Ptr& cloud, bool* new_cloud_avaliable_flag)
{
    cloud_mutex.lock();     // for not overwriting the cloud from another thread
    *cloud = *callback_cloud;
    *new_cloud_avaliable_flag = true;
    cloud_mutex.unlock();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pub_pcl");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<PointCloudT>("/openni/points2", 1);

    // Read
    PointCloudT::Ptr cloud(new PointCloudT);
    bool new_cloud_avaliable_flag = false;
    pcl::Grabber* interface = new pcl::OpenNIGrabber();
    boost::function<void (const PointCloudT::ConstPtr&)> f = boost::bind(&cloud_cb_, _1, cloud, &new_cloud_avaliable_flag);
    interface->registerCallback(f);
    interface->start();

    // Wait for the first frame
    while (!new_cloud_avaliable_flag)
    {
        boost::this_thread::sleep(boost::posix_time::milliseconds(1));
    }
    cloud_mutex.lock();     // for not overwriting the point cloud

    // Display
    pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
    viewer.addPointCloud<PointT>(cloud, rgb, "input_cloud");
    viewer.setCameraPosition(0, 0, -2, 0, -1, 0, 0);

    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        ros::Time scan_time = ros::Time::now();
        
        // Get & publish new cloud
        if (new_cloud_avaliable_flag && cloud_mutex.try_lock())
        {
            new_cloud_avaliable_flag = false;
            pub.publish(cloud);

            viewer.removeAllPointClouds();
            viewer.removeAllShapes();
            pcl::visualization::PointCloudColorHandlerRGBField<PointT> rgb(cloud);
            viewer.addPointCloud<PointT>(cloud, rgb, "input_cloud");

            cloud_mutex.unlock();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

