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
    float x = msg->linear.x;
    float y = msg->linear.y;
    float th = msg->angular.z;
    float dis = sqrt(x*x+y*y);
    float angle = atan2(y,x);
    if (angle<0) angle+=2*PI;
    char c = '0';
    if (th<0) c = '-';
    ss<<"sfg"<<std::setfill('0')<<std::setw(3)<<(int)(dis*100)<<"g"<<std::setw(3)<<int(angle/PI*180)<<"g"<<c<<std::setfill('0')<<std::setw(2)<<(int)(th*10)<<"g000e";
    std_msgs::String mesg;
    mesg.data = ss.str();
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
