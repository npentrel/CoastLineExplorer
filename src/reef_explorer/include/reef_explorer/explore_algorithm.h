#ifndef EXPLORE_ALGORITHM_H_
#define EXPLORE_ALGORITHM_H_

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

class ExploreAlgorithm
{
    
public:
    ExploreAlgorithm(const std::string& odometryTopic);
    ~ExploreAlgorithm();
    void runExploreAlgorithm();
    double* getMinimumDistanceValuePointer();

protected:
private:
    void controlDistanceToCliff();
    void setErrorValues();
    void followCliff();
    void initOdom();

    int state;
    double minimumDistanceValue;
    double scanDistance;
    double scanDistanceDownMovementOffset;
    ros::NodeHandle nh;
    ros::Publisher position_pub;
    nav_msgs::Odometry odom;
    std::string odometryTopic;

    double error;
    double total_error_I;
    double derivative_error_D;
    double last_error;
    double current_fix;
};

#endif
