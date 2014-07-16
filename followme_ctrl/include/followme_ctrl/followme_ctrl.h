#ifndef TINKER_FOLLOWME_CTRL_H
#define TINKER_FOLLOWME_CTRL_H

#include <ros/ros.h>
#include <ros/package.h>
#include <frmsg/people.h>
#include <frmsg/followme_state.h>
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <followme_ctrl/followme_state.h>

#include <deque>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class FollowmeCtrl {

public:
    FollowmeCtrl();
    ~FollowmeCtrl();

private:
    ros::NodeHandle nh_;

    ros::Subscriber people_subscriber_;
    ros::Subscriber starter_subscriber_;

    ros::Publisher state_publisher_;
    ros::Publisher people_pos_publisher_;

    MoveBaseClient ac_;

    void nodeInit();
    void navigationInit();

    void peopleCallback(frmsg::people::ConstPtr &p);
    void starterCallback(frmsg::starter_state::ConstPtr &p);
    
    void decide();

    void paintPeople(frmsg::people::ConstPtr &p);
    void sendTarget(double x, double y, double z, double w);

    FollowmeState current_state_;
    std::deque< std::pair< double, double> > people_stack_;
};

#endif