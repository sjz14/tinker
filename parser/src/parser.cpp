#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <string>
#include <math.h>
#include "frmsg/dec.h"
#define PI 3.1415926535
ros::Publisher * p= NULL;
void parserCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("get geometry");
    std::stringstream ss;
    float x = msg->linear.x*1000;
    float y = msg->linear.y*1000;
    float th = msg->angular.z/PI*180;
    float dis = sqrt(x*x+y*y);
    float angle = atan2(y,x);
    if (angle<0) angle+=2*PI;
    char c = '0';
    if (th<0) c = '-';
    th = abs(th);
    ss<<"sfg"<<std::setfill('0')<<std::setw(3)<<(int)(dis)<<"g"<<std::setw(3)<<int(angle/PI*180)<<"g"<<c<<std::setfill('0')<<std::setw(2)<<(int)(th)<<"g000e";
    std_msgs::String mesg;
    mesg.data = ss.str();
    ROS_INFO("sending:%s", ss.str().c_str());
    if (p) p->publish(mesg);
    return;
}
int main(int argc, char ** argv)
{
        ros::init(argc, argv, "parser_rm");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("/cmd_vel",1000,parserCallback);
        //ros::Subscriber sub = n.subscribe("dec",10,parserCallback);
        ros::Publisher  pub = n.advertise<std_msgs::String>("order", 1000);
        p = &pub;
        ros::spin();
        return 0;
}
