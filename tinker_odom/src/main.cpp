#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include "std_msgs/String.h"
#include <sstream>
#include <iostream>
#define PI 3.14159
#define LENLEG	187
#define OPTSCALE 198.4252
#define OFFSETY	60.0
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/UInt8MultiArray.h"
using namespace std;
ros::Time current_time, last_time;
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;


void Opt_Deltamove(const char opt[8],float* dx,float* dy,float* dth)
{
float vx,vy,tx,ty,fx1,fx2,fy1,fy2;
int x1,x2,y2,y1;
//for (int i = 0;i<8;i++)
//cout<<(int)opt[i]<<' ';
//cout<<endl;

x1=(((unsigned char)opt[1])<<8)+(unsigned char)opt[0];
y1=(((unsigned char)opt[3])<<8)+(unsigned char)opt[2];
x2=(((unsigned char)opt[5])<<8)+(unsigned char)opt[4];
y2=(((unsigned char)opt[7])<<8)+(unsigned char)opt[6];
//cout<<(int)opt[0]<<' '<<(int)opt[1]<<' '<<(int)opt[2]<<' '<<(int)opt[3]<<' '<<(int)opt[4]<<' '<<(int)opt[5]<<' '<<(int)opt[6]<<' '<<(int)opt[7]<<endl;
//cout<<x1<<' '<<x2<<' '<<y1<<' '<<y2<<endl;
if(x1&0x8000)
	x1=-((x1^0xffff)+1);
if(x2&0x8000)
	x2=-((x2^0xffff)+1);
if(y1&0x8000)
	y1=-((y1^0xffff)+1);
if(y2&0x8000)
	y2=-((y2^0xffff)+1);

fx1=-(float)x1/OPTSCALE;
fx2=-(float)x2/OPTSCALE;
fy1=-(float)y1/OPTSCALE;
fy2=-(float)y2/OPTSCALE;

ty=(fy1-fy2)/2;
vy=(fy1+fy2)/2;
vx=(fx1+fx2)/2;

*dth=(-atan(ty/LENLEG));
*dx=vx+OFFSETY*sin(*dth);
*dy=vy+OFFSETY*(1-cos(*dth));

}

void opticCallback(const std_msgs::UInt8MultiArray::ConstPtr& msg)
{
    float dx,dy,dth;
    current_time = ros::Time::now();
    char buf[8];
    for (int i=0;i<8;i++)
  	buf[i] = msg->data[i];
    Opt_Deltamove(buf,&dx,&dy,&dth);
    cout<<"dx"<<dx<<"dy"<<dy<<"dth"<<dth<<endl;
    double ddx=(1.414/2)*(dx-dy)/1000.0;
    double ddy=(1.414/2)*(dy+dx)/1000.0;
    th-=dth;
    double dy2=ddx*cos(th) + ddy*sin(th);
    double dx2= - ddx*cos(th) + ddy*cos(th);
    x+=dx2;
    y+=dy2;
    double dt=(current_time - last_time).toSec();
    vy = dx2/dt;
    vx = dy2/dt;
    vth = dth/dt;
    last_time = current_time;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;
  ros::Subscriber sub = n.subscribe("opticflow",1000,opticCallback);
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(20);
  while(n.ok()){

    ros::spinOnce();               // check for incoming messages
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time + ros::Duration(0.2);
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time + ros::Duration(0.2);
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    r.sleep();
  }
  return 0;
}
