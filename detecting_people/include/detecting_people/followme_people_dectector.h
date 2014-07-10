#ifndef FOLLOWME_PEOPLE_DETECTOR
#define FOLLOWME_PEOPLE_DETECTOR

#include <opencv2/opencv.hpp>  
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>    
#include <pcl/io/openni_grabber.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/people/ground_based_people_detection_app.h>
#include <pcl/common/time.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <math.h>
#include <ros/ros.h>
#include <ros/package.h>

namespace followme{

class FollowmePeopleDetector {

public:
	FollowmePeopleDetector();
	~FollowmePeopleDetector();

private:
	void follomeStateCallback(followme_msgs::followme_state::ConstPtr &state_msg);

};

} // namespace followme

#endif