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
using std::cout;
using std::endl;
using std::string;
using namespace LibSerial;

SerialStream my_serial_stream ;

void orderCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]",msg->data.c_str());
    if (my_serial_stream) my_serial_stream << msg->data.c_str();
}

int main(int argc, char** argv)
{
    /* Init */
    cout<<"serial"<<endl;
    ros::init(argc, argv, "serialport_driver");
    ros::NodeHandle n;
    ros::Rate rate(10);
    
    
    std::string package_path = ros::package::getPath("serial") + "/";

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
    

    ros::Subscriber sub = n.subscribe("order",1000,orderCallback);
    ros::spin();


    my_serial_stream.Close();
    cout<<"bye"<<endl;
    return 0;
}
