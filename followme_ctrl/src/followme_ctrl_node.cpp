#include <ros/ros.h>
#include <ros/package.h>
#include "frmsg/people.h"
#include "frmsg/followme_state.h"

void peopleCallback(const frmsg::people::ConstPtr &people)
{
    // send to navigation
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "followme_ctrl");
    ros::NodeHandle nh;
    state_pub = nh.advertise<frmsg::followme_state>("followme_state", 10);
    people_sub = nh.subscribe("followme_people", 10, &peopleCallback);
    ros::spin();
    return 0;
}