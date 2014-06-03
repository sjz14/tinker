#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <string>
#include <math.h>
#include "frmsg/dec.h"
ros::Publisher * p= NULL;
void parserCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("get geometry");
    std::stringstream ss;

    /*//added
    std::string angle;
    float speed;
    float sspeed = 0;
    if ( msg->dec == 1 )
    {
        angle = "g000g";
        speed = 300;
    }
    else if ( msg->dec == 2 )
    {
        speed = 300;
        angle = "g180g";
    }
    else if ( msg->dec == 3 )
    {
        sspeed = -45;
    }
    else if ( msg->dec == 4 )
    {
        sspeed = 45;
    }*/
    std::string angle((msg->linear.x > 0)?"g000g":"g180g");

    float speed = (msg->linear.x>0)?msg->linear.x:-msg->linear.x;
    if ((speed+(fabs(msg->angular.z)))<0.50) speed = 0;
    speed*=200;
    float sspeed = -msg->angular.z;
    char c = (sspeed>0)?'0':'-';
    sspeed = (sspeed>0)?sspeed*20:sspeed*(-20);
    if (speed==0) sspeed =0;
    ss<<"sfg"<<std::setfill('0')<<std::setw(3)<<(int)(speed)<<angle<<c<<std::setfill('0')<<std::setw(2)<<(int)sspeed<<"g000e";
    std_msgs::String mesg;
    mesg.data = ss.str();
    if (p) p->publish(mesg);
    return;
}
int main(int argc, char ** argv)
{
        ros::init(argc, argv, "parser_rm");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("/tr/cmd_vel",1000,parserCallback);
        //ros::Subscriber sub = n.subscribe("dec",10,parserCallback);
        ros::Publisher  pub = n.advertise<std_msgs::String>("order", 1000);
        p = &pub;
        ros::spin();
        return 0;
}
