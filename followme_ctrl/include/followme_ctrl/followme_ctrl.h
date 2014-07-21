#ifndef TINKER_FOLLOWME_CTRL_H
#define TINKER_FOLLOWME_CTRL_H

#include <ros/ros.h>
#include <ros/package.h>
#include "frmsg/people.h"
#include "frmsg/starter_state.h"
#include "frmsg/followme_state.h"
#include <geometry_msgs/PoseArray.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include <deque>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class FollowmeCtrl {

public:
    FollowmeCtrl(ros::NodeHandle nh);
    ~FollowmeCtrl();

private:
    ros::NodeHandle nh_;

    ros::Subscriber people_subscriber_;
    ros::Subscriber starter_subscriber_;

    ros::Publisher state_publisher_;
    ros::Publisher people_pos_publisher_;
    ros::Publisher goal_publisher_;

    MoveBaseClient ac_;

    void nodeInit();
    void navigationInit();

    void peopleCallback(const frmsg::people::ConstPtr &p);
    void starterCallback(const frmsg::starter_state::ConstPtr &p);

    void decide(const frmsg::people::ConstPtr &p);

    void paintPeople(const frmsg::people::ConstPtr &p);
    void sendTarget(double x, double y, double z, double oz, double ow);

    int current_state_;
    std::deque< std::pair< double, double> > people_stack_;
};

#endif
