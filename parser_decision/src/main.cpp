#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <string>
#include <math.h>
#include "frmsg/dec.h"
#include <opencv2/opencv.hpp>
#include <string.h>
#include <ros/package.h>
ros::Publisher * p= NULL;
using namespace std;
void parserCallback(const frmsg::dec::ConstPtr& msg)
{
    int turn = 25;
    string angle_right = "g070g";
    string angle_left = "g290g";
    int tmpspeed = 300;
    int tmp_turn_speed = 250;
    ROS_INFO("get geometry");
    std::string de;
    if ( msg->mode == 0)
    {
        if ( msg->dec == 1 )
        {
            de = "sfg300g000g000e";
        }
        else if ( msg->dec == 2 )
        {
            de = "sfg000g000g000e";
        }
        else if ( msg->dec == 3 )
        {
            de = "sfg200g000g-15e";
        }
        else if ( msg->dec == 4 )
        {
            de = "sfg200g000g015e";
        }
        else if ( msg->dec == 0 )
        {
            de = "sfg000g000g000e";
        }
        else if ( msg->dec == 5 )
        {
            de = "sfg500g000g000e";
        }
    }
    else if ( msg->mode == 1 )
    {
		std::stringstream ss;
		float speed;
		float sspeed;
		char c;
		std::string angle;//((msg->linear.x > 0)?"g000g":"g180g");
		if (msg->direc == 0 )
		{
			speed = 0.0;
			sspeed = 0.0;
			angle = "g000g";
			c = '0';
		}
		else
        {
            if ( msg->direc == 1)
	       	{
	    	    sspeed = 0.0;
                angle = "g000g";
                c = '0';
                speed = msg->speed;
                speed = tmpspeed;
	    	}
            else if ( msg->direc == 2) // turn right
            {
                sspeed = turn;
                c = '0';
                angle = angle_left;
                speed = msg->speed / 2;
                speed = tmp_turn_speed;
            }
            else if ( msg->direc == 3) // turn left
            {
                sspeed = turn;
                angle = angle_right;
                c = '-';
                speed = msg->speed / 2;
                speed = tmp_turn_speed;
            }
        }
		ss<<"sfg"<<std::setfill('0')<<std::setw(3)<<(int)(speed)<<angle<<c<<std::setfill('0')<<std::setw(2)<<(int)sspeed<<"e";
		de = ss.str();
		ROS_INFO("mode:%d,direc:%d,speed:%f",msg->mode,msg->direc,speed);
    }
    std_msgs::String mesg;
    mesg.data = de;
    if (p) p->publish(mesg);
   // ros::Duration(0.3).sleep();
   // de = "sfg000g000g000e";
   // mesg.data = de;
   // if (p) p->publish(mesg);
   // ros::Duration(2.0).sleep();
    return;
}
int main(int argc, char ** argv)
{
        ros::init(argc, argv, "parser");
        ros::NodeHandle n;
       // ros::Subscriber sub = n.subscribe("/tr/cmd_vel",1000,parserCallback);
        ros::Subscriber sub = n.subscribe("dec",10,parserCallback);
        ros::Publisher  pub = n.advertise<std_msgs::String>("order", 1000);
        p = &pub;
        ros::spin();
        return 0;
}
