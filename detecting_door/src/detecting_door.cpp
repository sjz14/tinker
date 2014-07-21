#include <reading_pointcloud/reading_pointcloud.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <std_msgs/Int32.h>

int current_signal = 1;

void signalCallback(const std_msgs::Int32::ConstPtr& signal)
{
  if (current_signal == signal->data)
    return;
  current_signal = signal->data;
}

int main ( int argc, char** argv)
{

  ros::init(argc, argv, "detecting_door");
  ros::NodeHandle nh;
  ros::Rate rate(13);
  printf("detecting door launching...\n");

    ros::Subscriber door_signal = nh.subscribe("door_signal", 100, &signalCallback);
  ros::Publisher door_status = nh.advertise<std_msgs::Int32>("door_status", 100);
  std_msgs::Int32 door_status_pub;

  CloudConverter* cc_ = new CloudConverter();

  printf("here\n");
  while (!cc_->ready_xyzrgb_)
  {
    printf("Waiting for cloud converter\n");
    ros::spinOnce();
    rate.sleep();
    if (!ros::ok())
    {
      printf("Terminated by control-c\n");
      return -1;
    }
  }
  printf("there\n");

  pcl::visualization::PCLVisualizer viewer("PCL Viewer");          // viewer initialization
    viewer.setCameraPosition(0,0,-2,0,-1,0,0);

  while((ros::ok()) && (!viewer.wasStopped()))
  {
    if ( current_signal == 1 )
    {
      printf("detecting...\n");
      if ( cc_->ready_xyzrgb_)
        {
          pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud = cc_->msg_xyzrgb_;
            cc_->ready_xyzrgb_ = false;
            viewer.removeAllPointClouds();
            viewer.removeAllShapes();
              pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
            viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "input_cloud");
            double avg_dep = 0;
            int n = 0;
            for ( int i = cloud->width / 3; i < cloud->width / 3 * 2; i++ )
            {
              for ( int j = cloud->height / 3; j < cloud->height / 3 * 2; j++ )
              {
                if ( cloud->points[ i * j ].z > 0)
                {
                  avg_dep += cloud->points[ i * j ].z;
                  n++;
                }
              }
            }
            avg_dep /= n;
            printf("The avg_dep is %lf .\n", avg_dep);
            if ( avg_dep > 0.9)
            {
              door_status_pub.data = 1;
            }
            else
            {
              door_status_pub.data = 0;
            }
          door_status.publish(door_status_pub);
        }
        ros::spinOnce();
        viewer.spinOnce();
  } else if (current_signal == 0) {
      printf("not started...\n");
        ros::spinOnce();
        viewer.spinOnce();
    } else {
        break;
    }
  }
  printf("detecting finished.\n");
}
