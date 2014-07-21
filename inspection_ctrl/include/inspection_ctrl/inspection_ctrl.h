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
    void pathInit();
    void nodeInit();
    void navigationInit();
    void doorDetectorCallback(const std_msgs::Int32::ConstPtr &p);
    void walk();

    double tar_x_[11], tar_y_[11], tar_oz_[11], tar_ow_[11];

    MoveBaseClient ac_;

    ros::Subscriber door_subscriber_;
    ros::Publisher door_signal_publisher_;

    ros::NodeHandle nh_;

    int open_count;
    int is_moving;
    int path_len;
};

const int OPEN_THRES = 10;

#endif
