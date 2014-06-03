
// Project:     serial
// File:        main.cpp
// Created by bss at 2014-01-09
// Last modified: 2014-03-11, 14:46:18
// Description:
///edited by xf 2014-1-18
//

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <SerialStream.h>
#include "std_msgs/String.h"
#include <QSettings>
#include <QList>
#include <geometry_msgs/Twist.h>
using std::cout;
using std::endl;
using std::string;
using namespace LibSerial;

SerialStream my_serial_stream ;

void orderCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    unsigned char *order1 = new unsigned char[10];
    unsigned char *order2 = new unsigned char[10];
    order1[0] = 0xff;
    order1[1] = 0xff;
    order1[2] = 0xfe;//id
    order1[3] = 0x05;
    order1[4] = 0x03;
    order1[5] = 0x1E;
    order1[6] = 0x3ff*150*msg->linear.x/300;
    order1[7] = (0x3ff*150*msg->linear.x/300)/256;
    order1[8] = order1[2]+order1[3]+order1[4]+order1[5]+order1[6]+order1[7];
    order1[8] = ~order1[8];
    order2[0] = 0xff;
    order2[1] = 0xff;
    order2[2] = 0xfe;
    order2[3] = 0x02;
    order2[4] = 0x05;
    order2[5] = order2[2]+order2[3]+order2[4];
    order2[5] = ~order2[5];
    for (int i = 0; i<9;i++)
    if (my_serial_stream) my_serial_stream << order1[i];
    //if (my_serial_stream) my_serial_stream << order2;
    for (int i = 0; i<9; i++)
        cout<<std::hex<<int(order1[i])<<' ';
    cout<<endl;
    for (int i=0; i<6; i++)
        cout<<std::hex<<int(order2[i])<<' ';
    cout<<endl;
    delete [] order1;
    delete [] order2;


}

string parseCommand()
{
    string ins;
    return ins;
}



int main(int argc, char** argv)
{
    /* Init */
    cout<<"serial"<<endl;
    ros::init(argc, argv, "serialport_driver");
    ros::NodeHandle n;
    ros::Rate rate(10);
    std::string package_path = ros::package::getPath("roboarm") + "/";

    QSettings settings((package_path + "furoc.ini").c_str(),QSettings::IniFormat);
    int pos = settings.value("pos", 1).toInt();
    printf("pos=%d\n", pos);
    QString dev = settings.value("dev",1).toString();
    printf("dev = %s\n",dev.toStdString().c_str());
    my_serial_stream.Open( dev.toStdString().c_str() ) ;
    if (!my_serial_stream){
        std::cout<<"cannot open serial"<<std::endl;
        return -1;
    }
   my_serial_stream.SetBaudRate( SerialStreamBuf::BAUD_9600) ;

    ros::Subscriber sub = n.subscribe("/tr/cmd_vel",1000,orderCallback);
    ros::spin();


    my_serial_stream.Close();
    cout<<"bye"<<endl;
    return 0;
}
