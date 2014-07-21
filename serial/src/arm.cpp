// Project:     serial
// File:        arm.cpp
// Created by bss at 2014-07-21
// Last modified: 2014-07-21, 22:40:52
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
#include <vector>
#include <SerialPort.h>
#include <SerialStream.h>
#include "std_msgs/String.h"
#include <QSettings>
#include <QList>
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"

using std::cout;
using std::endl;
using std::string;
using namespace LibSerial;
using namespace std;

SerialStream my_serial_stream ;
char cmd[20];
//SerialPort * my_serial_port = NULL;

void orderCallback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]",msg->data.c_str());
    if (my_serial_stream) my_serial_stream<<msg->data.c_str();
    strcpy(cmd,msg->data.c_str());
}

int main(int argc, char** argv)
{
    /* Init */
    cout<<"serial_arm"<<endl;
    ros::init(argc, argv, "serialport_arm_driver");
    ros::NodeHandle n;
    ros::Rate rate(40);
    std::string package_path = ros::package::getPath("serial") + "/";
    QSettings settings((package_path + "arm.ini").c_str(),QSettings::IniFormat);
    int pos = settings.value("pos", 1).toInt();
    printf("pos=%d\n", pos);
    QString dev = settings.value("dev",1).toString();
    printf("preconfig dev = %s\n",dev.toStdString().c_str());
    //my_serial_stream.Open( dev.toStdString().c_str() ) ;

    QString password = settings.value("sudopass",1).toString();
    QString chipset = settings.value("chipset",1).toString();
    printf("selected chipset is %s\n",chipset.toStdString().c_str());
    char command[1024] = "dmesg | grep ";
    strcat(command, chipset.toStdString().c_str());
    char buffer[100] = {0};
    FILE* fp = popen(command, "r");
    char devnum[10] = "NULL";
    while (fgets(buffer,sizeof(buffer),fp)!=NULL)
    {
        int pos;
        char * substr = strstr(buffer,"ttyUSB");
        if (substr!=0) strncpy(devnum,substr,7);
    }
    pclose(fp);
    printf("%s\n",devnum);
    char sudocommand[100] = "echo ";
    std::stringstream ss;

    if (strcmp(devnum,"NULL") != 0)
    {
        strcat(sudocommand, password.toStdString().c_str());
        strcat(sudocommand, " | sudo -S chmod 777 /dev/");
        strcat(sudocommand, devnum);
        //printf("%s\n",sudocommand);
        system(sudocommand);
        char devnum2[20] = "/dev/";
        strcat(devnum2,devnum);
        my_serial_stream.Open( devnum2 ) ;
        ss<<devnum2;
    }
    else
    {
        strcat(sudocommand, password.toStdString().c_str());
        strcat(sudocommand, " | sudo -S chmod 777 /dev/");
        strcat(sudocommand, dev.toStdString().c_str());
        //printf("%s\n",sudocommand);
        system(sudocommand);
        my_serial_stream.Open( dev.toStdString().c_str() ) ;
        ss<<dev.toStdString().c_str();

    }
    //cout<<"serialport:"<<ss.str()<<endl;
    //my_serial_port = new SerialPort(ss.str());
    //std::cout<<my_serial_port->IsOpen()<<std::endl;
    //my_serial_port->Open();

    //std::cout<<my_serial_port->IsOpen()<<std::endl;
    //my_serial_port->SetBaudRate(SerialPort::BAUD_9600);

    if (!my_serial_stream)
    {
        std::cout<<"cannot open serial"<<std::endl;
        return -1;
    }

    my_serial_stream.SetBaudRate( SerialStreamBuf::BAUD_57600) ;
    std_msgs::String msg2;
    ros::Subscriber sub = n.subscribe("order_arm",1000,orderCallback);

    std_msgs::UInt8MultiArray m;
    
    
    // read from file
    int recmove[3][1000];
	FILE *f1;
	f1=fopen((package_path + "data/wave2.txt").c_str(), "r");
	for(int j=0;j<3;j++)
	{
	    for(int i=0;i<316;i++)
	    {
	        fscanf(f1,"%d",&(recmove[j][i]));
	    }
	}
	fclose(f1);

    unsigned char data[16];
    ros::Rate loop_rate(1);
    int speed, sum;
    int lastpos[3];
    int repflag = 0;
    while(ros::ok())
    {
        for (int rcvack = 1; rcvack <= 3; rcvack++)
        {
            data[0]=0xff;
            data[1]=0xff;
            data[2]=rcvack;
            data[3]=0x07;
            data[4]=0x03;
            data[5]=0x1e;		
            data[6]=(unsigned char)(recmove[rcvack-1][repflag]%256);			
            data[7]=(unsigned char)(recmove[rcvack-1][repflag]/256);
            speed=(abs(recmove[rcvack-1][repflag]-lastpos[rcvack-1])/2)+1;
            lastpos[rcvack-1]=recmove[rcvack-1][repflag];		
            data[8]=(unsigned char)(speed%256);
            data[9]=(unsigned char)(speed/256);
            sum=0;
            for(int i=2;i<10;i++)
            {
	            sum+=data[i];
            }
            data[10]=sum & 0xff;
            data[10]=~(data[10]);


            //comm1.put_Output(COleVariant(data));
            for (int i = 0; i <= 10; i++)
            {
                my_serial_stream << data[i];
            }
            repflag++;
            
            loop_rate.sleep();
        }
        
        
        rate.sleep();
        //msg2.data = ss.str();
        //ROS_INFO("%s",buffer_opt);
        //pub.publish(msg2);
        ros::spinOnce();
    }

    //my_serial_port->Close();
    my_serial_stream.Close();
    cout<<"bye"<<endl;
    return 0;
}
