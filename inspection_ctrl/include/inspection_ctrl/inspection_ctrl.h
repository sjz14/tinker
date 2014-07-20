#ifndef INSPECTION_CTRL_H
#define INSPECTION_CTRL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
class InspectionCtrl{

public:
    InspectionCtrl(ros::NodeHandle nh);
    ~InspectionCtrl();

private:
    void nodeInit();
    void navigationInit();
    void doorDetectorCallback(const std_msgs::Int32::ConstPtr &p);
    void walk();

    void set_target() {

     tar_x_[0] = 5.971, tar_x_[1] = 8.672, tar_x_[2] = 15.005;
     tar_y_[0] = -6.850, tar_y_[1] = -1.227, tar_y_[2] = -8.904;
     tar_z_[0] = 0.00, tar_z_[1] = 0.00, tar_z_[2] = 0.00;
     tar_oz_[0] = 0.666, tar_oz_[1] = 0.729, tar_oz_[2] = -0.667;
     tar_ow_[0] = 0.746, tar_ow_[1] = 0.685, tar_ow_[2] = 0.745;

    }

    double tar_x_[3], tar_y_[3], tar_z_[3], tar_oz_[3], tar_ow_[3];

    MoveBaseClient ac_;

    ros::Subscriber door_subscriber_;
    ros::Publisher door_signal_publisher_;

    ros::NodeHandle nh_;

    int open_count;
    int is_moving;
};

const int OPEN_THRES = 10;

#endif
