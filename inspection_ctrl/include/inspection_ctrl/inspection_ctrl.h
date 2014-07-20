#ifndef INSPECTION_CTRL_H
#define INSPECTION_CTRL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

class InspectionCtrl{

public:
    InspectionCtrl(ros::NodeHandle nh);
    ~InspectionCtrl();

private:
    void nodeInit();
    void navigationInit();
    void doorDetectorCallback(const std_msgs::Int32::ConstPtr &p);
    void walk();

    double tar_x_[3];
    double tar_y_[3];
    double tar_z_[3];
    double tar_oz_[3];
    double tar_ow_[3];

    MoveBaseClient ac_;

    ros::Subscriber door_subscriber_;
    ros::Publisher door_sighal_publisher_;

    ros::NodeHandle nh_;
}