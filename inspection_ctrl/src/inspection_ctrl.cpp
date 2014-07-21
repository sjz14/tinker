#include <inspection_ctrl/inspection_ctrl.h>
#include <ros/time.h>
#include <ros/duration.h>
#include <string>

InspectionCtrl::InspectionCtrl(ros::NodeHandle nh):
    nh_(nh),
    ac_("move_base", true),
    is_moving(0),
    open_count(0)
{
    pathInit();
    nodeInit();
    navigationInit();

    std_msgs::Int32 start_msg;
    start_msg.data = 1;
    for (int i = 0; i < 5; i++)
        door_signal_publisher_.publish(start_msg);

    printf("InspectionCtrl Ready!\n");
}

InspectionCtrl::~InspectionCtrl()
{
    // nothing
}

void InspectionCtrl::pathInit()
{
    path_len = 11;
    tar_x_[0] = 6.097;
    tar_y_[0] = -7.992;
    tar_oz_[0] = 0.698;
    tar_ow_[0] = 0.716;

    tar_x_[1] = 6.663;
    tar_y_[1] = -6.288;
    tar_oz_[1] = 0.221;
    tar_ow_[1] = 0.975;

    tar_x_[2] = 7.508;
    tar_y_[2] = -5.172;
    tar_oz_[2] = 0.621;
    tar_ow_[2] = 0.784;

    tar_x_[3] = 8.233;
    tar_y_[3] = -3.033;
    tar_oz_[3] = 0.571;
    tar_ow_[3] = 0.821;

    tar_x_[4] = 9.080;
    tar_y_[4] = -0.481;
    tar_oz_[4] = 0.089;
    tar_ow_[4] = 0.996;

    tar_x_[5] = 11.751;
    tar_y_[5] = 0.113;
    tar_oz_[5] = -0.001;
    tar_ow_[5] = 0.100;

    tar_x_[6] = 12.928;
    tar_y_[6] = -0.336;
    tar_oz_[6] = -0.735;
    tar_ow_[6] = 0.678;

    tar_x_[7] = 13.391;
    tar_y_[7] = -3.404;
    tar_oz_[7] = -0.669;
    tar_ow_[7] = 0.743;

    tar_x_[8] = 13.801;
    tar_y_[8] = -5.317;
    tar_oz_[8] = -0.331;
    tar_ow_[8] = 0.944;

    tar_x_[9] = 14.806;
    tar_y_[9] = -6.462;
    tar_oz_[9] = -0.581;
    tar_ow_[9] = 0.814;

    tar_x_[10] = 14.954;
    tar_y_[10] = -7.980;
    tar_oz_[10] = -0.796;
    tar_ow_[10] = 0.605;
}

void InspectionCtrl::nodeInit()
{
    door_signal_publisher_ = nh_.advertise< std_msgs::Int32 >(
        "door_signal", 5);

    say_publisher_ = nh.advertise< std_msgs::String >(
        "say", 5);

    door_subscriber_ = nh_.subscribe("door_status", 5, &InspectionCtrl::doorDetectorCallback, this);
}

void InspectionCtrl::navigationInit()
{
    while (!ac_.waitForServer(ros::Duration(5.0))) {
        printf("waiting for ac\n");
    }
}

void InspectionCtrl::doorDetectorCallback(const std_msgs::Int32::ConstPtr &p)
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

void InspectionCtrl::speak()
{
    std::string sentence("Hello, I am tinker\n");
    std_msgs::String msg;
    msg.data = sentence;
    say_publisher_.publish(msg);
}

void InspectionCtrl::walk()
{
    ros::Duration start_space(6.0);
    start_space.sleep();

    for (int i = 1; i < path_len; i++) {
        move_base_msgs::MoveBaseGoal goal;

        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = tar_x_[i];
        goal.target_pose.pose.position.y = tar_y_[i];
        goal.target_pose.pose.position.z = 0;

        goal.target_pose.pose.orientation.x = 0;
        goal.target_pose.pose.orientation.y = 0;
        goal.target_pose.pose.orientation.z = tar_oz_[i];
        goal.target_pose.pose.orientation.w = tar_ow_[i];

        printf("Sending goal (%lf %lf) (%lf %lf)\n", tar_x_[i], tar_y_[i], tar_oz_[i], tar_ow_[i]);
        ac_.sendGoal(goal);

        ac_.waitForResult();
        if(ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            printf("Hooray, the base moved to the target");
        else
            printf("The base failed to move to the target");

        if (i == 5) {
            speak();
        }

    }
}
