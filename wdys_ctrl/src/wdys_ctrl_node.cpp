#include <wdys_ctrl/wdys_ctrl.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wdys_ctrl");
    ros::NodeHandle nh;
    WdysCtrl ctrl(nh);
    ros::spin();
    return 0;
}
