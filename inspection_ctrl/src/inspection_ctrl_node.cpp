#include <inspection_ctrl/inspection_ctrl.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "inspection_ctrl");
    ros::NodeHandle nh;
    InspectionCtrl ctrl(nh);
    ros::spin();
    return 0;
}