#ifndef INSPECTION_CTRL_H
#define INSPECTION_CTRL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Twist.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
class WdysCtrl{

public:
    WdysCtrl(ros::NodeHandle nh);
    ~WdysCtrl();

private:
    void nodeInit();
    void doorDetectorCallback(const std_msgs::Int32::ConstPtr &p);
    void answerFinishedCallback(const std_msgs::Int32::ConstPtr &p);
    void speak(char *s);
    void walk();

    double tar_x_[11], tar_y_[11], tar_oz_[11], tar_ow_[11];

    ros::Subscriber door_subscriber_;
    ros::Subscriber answer_finished_subscriber_;
    ros::Publisher door_signal_publisher_;
    ros::Publisher say_publisher_;
    ros::Publisher answer_init_publisher_;
    ros::Publisher cmd_vel_publisher_;
    ros::NodeHandle nh_;

    int is_moving;
    int is_backing;
    int open_count;

};

const int OPEN_THRES = 10;

#endif
