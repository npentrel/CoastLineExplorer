#include <stdlib.h>
#include <string.h>
#include <limits>

//ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>


#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../include/reef_explorer/distance_finder.h"
#include "../include/reef_explorer/explore_algorithm.h"


// current appraoch
std::string odometryTopic = "/dataNavigator_G500RAUVI";
std::string rangeImageTopic = "/girona500_RAUVI/rangecamera";
std::string octomapTopic = "/octomap_full";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reef_explorer");
    ExploreAlgorithm exploreAlgorithm(odometryTopic, octomapTopic);
    DistanceFinder distanceFinder(rangeImageTopic, -1.0, 0.25);
    distanceFinder.setMinimumDistanceValuePointer(exploreAlgorithm.getMinimumDistanceValuePointer());
	ros::Rate r(25);
	ros::spinOnce();
	r.sleep();
	ros::spinOnce();
	while (ros::ok())
    {
        exploreAlgorithm.runExploreAlgorithm();
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
