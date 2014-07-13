#include <followme_ctrl/followme_ctrl.h>

FollowmeCtrl::FollowmeCtrl(ros::NodeHandle nh):
    nh_(nh),
    current_state_(followme::NOTSTART),
    ac_("move_base", true)
{
    nodeInit();
    navigationInit();
    people_stack_.clear();
}

void FollowmeCtrl::nodeInit()
{
    ROS_INFO("Followme Controller set up");

    people_subscriber_ = nh_.subscribe(
        "followme_people", 1, &FollowmeCtrl::peopleCallback, this);

    state_publisher_ = nh_.advertise<frmsg::followme_state>(
        "followme_state", 5);
}

void FollowmeCtrl::navigationInit()
{
    while (!ac_.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
}

void FollowmeCtrl::peopleCallback(frmsg::people::ConstPtr &p)
{
    // convert p to some target
    // not sure about the usage of move_base_msgs::MoveBaseGoal
    double x = p-> ; // bala
    double y = p-> ; // bala

    if (people_stack_.size() > 10)
        people_stack_.pop_front();
    people_stack_.push_back(std::makepair(x, y));
    decide();
}

void decide()
{
    // to be done
}

void sendTarget(double x, double w)
{
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.orientation.w = w;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    // not scientific: target should be refreshed continuously
    // to be changed!

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, the base moved 1 meter forward");
    else
        ROS_INFO("The base failed to move forward 1 meter for some reason");

    return 0;
}