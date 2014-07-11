// Project      : openni_pcl_grabber
// File         : send_cloud.cpp
// Creation Date: 2014-07-11
// Last modified: 2014-07-11, 23:44:45
// Description  : 转发 openni 中提取的 pcl 到 ros
// 

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

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
    //cloud_mutex.lock();     // for not overwriting the point cloud

    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        ros::Time scan_time = ros::Time::now();
        
        // Get & publish new cloud
        if (new_cloud_avaliable_flag && cloud_mutex.try_lock())
        {
            new_cloud_avaliable_flag = false;
            pub.publish(cloud);
            cloud_mutex.unlock();
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

