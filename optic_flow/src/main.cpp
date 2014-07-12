#include <math.h>
#include "ros/ros.h"
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
ros::Publisher * pub;
using namespace std;
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
    char buf[8];
    for (int i=0;i<8;i++)
  	buf[i] = msg->data[i];
    Opt_Deltamove(buf,&dx,&dy,&dth);
    cout<<"dx"<<dx<<"dy"<<dy<<"dth"<<dth<<endl;
}
int main(int argc, char ** argv)
{ 
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("opticflow",1000,opticCallback);
    
    ros::spin();
    return 0;
}

