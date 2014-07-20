#include <inspection_ctrl/inspection_ctrl.h>
#include <ros/time.h>
#include <ros/duration.h>

InspectionCtrl::InspectionCtrl(ros::NodeHandle nh):
    nh_(nh),
    ac_("move_base", true),
    is_moving(0),
    open_count(0)
{
    nodeInit();
    navigationInit();

    std_msgs::Int32 start_msg;
    start_msg.data = 1;
    for (int i = 0; i < 5; i++)
        door_signal_publisher_.publish(start_msg);
}

InspectionCtrl::~InspectionCtrl()
{
    // nothing
}

void InspectionCtrl::nodeInit()
{
    printf("Inspection Controller set up\n");

    door_signal_publisher_ = nh_.advertise< std_msgs:: Int32 >(
        "door_signal", 5);

    door_subscriber_ = nh_.subscribe("door_status", 5, &InspectionCtrl::doorDetectionCallback, this);
}

void InspectionCtrl::navigationInit()
{
    while (!ac_.waitForServer(ros::Duration(5.0))) {
        printf("waiting for ac\n");
    }
}

void InspectionCtrl::doorDetectionCallback(const std_msgs::Int32::ConstPtr &p)
{
    if (is_moving)
        return;
    if (p->data == 1) {
        open_count += 1;
        printf("open\n");
        if (open_count < OPEN_THRES)
            return;
        printf("move move!\n");
        is_moving = 1;
        std_msgs::Int32 stop_msg;
        stop_msg.data = 2;
        for (int i = 0; i < 5; i++)
            door_signal_publisher_.publish(stop_msg);
        walk();
    } else {
        open_count = 0;
    }
}

void InspectionCtrl::walk()
{
    ros::Duration start_space(2.0);
    d.sleep();

    for (int i = 0; i < 3; i++) {
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = tar_x_[i];
        goal.target_pose.pose.position.y = tar_y_[i];
        goal.target_pose.pose.position.z = tar_z_[i];

        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = oz;
        goal.target_pose.pose.orientation.w = ow;

        printf("Sending goal");
        ac_.sendGoal(goal);

        ac_.waitForResult();
        if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            printf("Hooray, the base moved to the target");
        else
            printf("The base failed to move to the target");

    }
}