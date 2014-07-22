#include <wdys_ctrl/wdys_ctrl.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <string>

WdysCtrl::WdysCtrl(ros::NodeHandle nh):
    nh_(nh),
    is_moving(0),
    is_backing(0),
    open_count(0)
{
    nodeInit();

    std_msgs::Int32 start_msg;
    start_msg.data = 1;
    for (int i = 0; i < 5; i++)
        door_signal_publisher_.publish(start_msg);

    printf("WdysCtrl Ready!\n");
}

WdysCtrl::~WdysCtrl()
{
    // nothing
}

void WdysCtrl::nodeInit()
{
    door_signal_publisher_ = nh_.advertise< std_msgs::Int32 >(
        "door_signal", 5);

    say_publisher_ = nh_.advertise< std_msgs::String >(
        "say", 5);

    door_subscriber_ = nh_.subscribe("door_status", 5, &WdysCtrl::doorDetectorCallback, this);

    answer_finished_subscriber_ = nh_.subscribe(
            "/answer/finished", 5, &WdysCtrl::answerFinishedCallback, this);

    answer_init_publisher_ = nh_.advertise< std_msgs::Int32 >(
        "/answer/init", 5);

    cmd_vel_publisher_ = nh_.advertise< geometry_msgs::Twist > ("/cmd_vel", 5);
}

void WdysCtrl::answerFinishedCallback(const std_msgs::Int32::ConstPtr &p)
{
    if (p->data != 1) return;
    if (is_backing) return;
    is_backing = 1;
    // say goodbye
    geometry_msgs::Twist bak;
    bak.linear.x = -1;
    cmd_vel_publisher_.publish(bak);
    ros::Duration(3.0).sleep();
    bak.linear.x = 0;
    cmd_vel_publisher_.publish(bak);
}

void WdysCtrl::doorDetectorCallback(const std_msgs::Int32::ConstPtr &p)
{
    if (is_moving)
        return;
    if (p->data == 1) {
        open_count += 1;
        printf("open\n");
        if (open_count < OPEN_THRES)
            return;
        printf("move move!\n");
        is_moving = 1;
        std_msgs::Int32 stop_msg;
        stop_msg.data = 2;
        for (int i = 0; i < 5; i++)
            door_signal_publisher_.publish(stop_msg);
        walk();
    } else {
        open_count = 0;
    }
}

void WdysCtrl::speak(char *s)
{
    std_msgs::String msg;
    msg.data = std::string(s);
    say_publisher_.publish(msg);
}

void WdysCtrl::walk()
{
    ros::Duration start_space(4.0);
    start_space.sleep();

    geometry_msgs::Twist msg;
    msg.linear.x = 0.3;
    cmd_vel_publisher_.publish(msg);
    ros::Duration moving(3.0);
    moving.sleep();
    msg.linear.x = 0;
    cmd_vel_publisher_.publish(msg);

    ros::Duration waitit(2.0);
    waitit.sleep();

    std_msgs::Int32 ok;
    ok.data = 1;
    answer_init_publisher_.publish(ok);
}
