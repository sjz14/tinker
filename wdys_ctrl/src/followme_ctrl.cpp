#include <followme_ctrl/followme_ctrl.h>
#include <ros/time.h>
#include <ros/duration.h>

FollowmeCtrl::FollowmeCtrl(ros::NodeHandle nh):
    nh_(nh),
    current_state_(frmsg::followme_state::NOTSTART),
    ac_("move_base", true)
{
    nodeInit();
    navigationInit();
    people_stack_.clear();
}

FollowmeCtrl::~FollowmeCtrl()
{
    // nothing
}

void FollowmeCtrl::nodeInit()
{
    printf("Followme Controller set up\n");

    people_subscriber_ = nh_.subscribe(
        "followme_people", 1, &FollowmeCtrl::peopleCallback, this);

    state_publisher_ = nh_.advertise<frmsg::followme_state>(
        "followme_state", 5);

    starter_subscriber_ = nh_.subscribe(
        "starter/cmd", 1, &FollowmeCtrl::starterCallback, this);

    people_pos_publisher_ = nh_.advertise<geometry_msgs::PoseArray>(
        "followme_people_pos", 5);
}

void FollowmeCtrl::navigationInit()
{
    while (!ac_.waitForServer(ros::Duration(5.0))){
        printf("Waiting for the move_base action server to come up\n");
    }

    // goal_publisher_ = nh_.advertise<geometry_msgs::Pose>(
    //     "/move_base/current_goal", 5);
}

void FollowmeCtrl::starterCallback(const frmsg::starter_state::ConstPtr &p)
{
    if (current_state_ != frmsg::followme_state::NOTSTART)
        return;
    if (p->state == frmsg::starter_state::FOLLOWME) {
        current_state_ = frmsg::followme_state::RUNNING;
        printf("Followme now start!\n");
        printf("state: %d\n", current_state_);
        ros::Duration d(2.0);
        d.sleep();

        ros::Duration delta(0.02);
        frmsg::followme_state ns;
        for (int i = 0; i < 5; i++) {
            ns.header.stamp = ros::Time::now();
            ns.state = current_state_;
            state_publisher_.publish(ns);
            delta.sleep();
        }
    }
}

void FollowmeCtrl::peopleCallback(const frmsg::people::ConstPtr &p)
{
    using namespace std;

    // if (people_stack_.size() > 10)
    //     people_stack_.pop_front();
    // people_stack_.push_back(std::makepair(x, y));
    // decide();

    decide(p);
}

void FollowmeCtrl::decide(const frmsg::people::ConstPtr &p)
{
    paintPeople(p);
    if (p->id < 0) {
        printf("Alas? Where is the people\n");
        return; // do nothing
    } else {
        double people_x = p->depth[p->id];
        double people_y = -p->x[p->id];
        double people_z = -p->y[p->id];
        double ow = 1;
        double oz = 0;

        people_x *= 0.5;
        people_y *= 0.5;

        double theta = atan2(people_y, people_x);
        oz = sin(theta * 0.5);
        ow = cos(theta * 0.5);

        sendTarget(people_x, people_y, people_z, oz, ow);
        printf("Yeah I see you\n");
    }
}

void FollowmeCtrl::paintPeople(const frmsg::people::ConstPtr &p)
{
    geometry_msgs::PoseArray::Ptr pose_array_msg;
    pose_array_msg = boost::make_shared<geometry_msgs::PoseArray>();
    for (int i = 0; i < p->x.size(); i++) {
        geometry_msgs::Pose pose_msg;
        pose_msg.position.x = p->depth[i];
        pose_msg.position.y = -p->x[i];
        pose_msg.position.z = -p->y[i];
        pose_msg.orientation.x = 0;
        pose_msg.orientation.y = 0;
        pose_msg.orientation.z = 0;
        pose_msg.orientation.w = 1;
        pose_array_msg->poses.push_back(pose_msg);
    }
    pose_array_msg->header.frame_id = "base_link";
    pose_array_msg->header.stamp = ros::Time::now();
    people_pos_publisher_.publish(*pose_array_msg);
}

void FollowmeCtrl::sendTarget(double x, double y, double z, double oz, double ow)
{
    printf("target ---- x %lf y %lf z %lf oz %lf ow %lf\n", x, y, z, oz, ow);
    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    goal.target_pose.pose.position.z = z;

    goal.target_pose.pose.orientation.x = 0;
    goal.target_pose.pose.orientation.y = 0;
    goal.target_pose.pose.orientation.z = oz;
    goal.target_pose.pose.orientation.w = ow;

    ROS_INFO("Sending goal");
    // goal_publisher_.publish(goal);
    ac_.sendGoal(goal);

    // not scientific: target should be refreshed continuously
    // to be changed!

    // ac_.waitForResult();

    // if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //     ROS_INFO("Hooray, the base moved 1 meter forward");
    // else
    //     ROS_INFO("The base failed to move forward 1 meter for some reason");
    return;
}
