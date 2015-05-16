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
std::string jointTopic = "/girona500_RAUVI/joints_in";
std::string panName = "pan";
std::string tiltName = "tilt";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reef_explorer");
    ExploreAlgorithm exploreAlgorithm(odometryTopic);
    DistanceFinder distanceFinder(rangeImageTopic, jointTopic, panName, tiltName , 0.25, 0.0);
    distanceFinder.setMinimumDistanceValuePointer(exploreAlgorithm.getMinimumDistanceValuePointer());

	ros::Rate r(25);
	while (ros::ok())
    {
        exploreAlgorithm.runExploreAlgorithm();
		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
