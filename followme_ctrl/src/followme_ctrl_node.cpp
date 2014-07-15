#include <followme_ctrl/followme_ctrl.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "followme_ctrl");
    ros::NodeHandle nh;
    FollowmeCtrl ctrl(nh);
    ros::spin();
    return 0;
}
