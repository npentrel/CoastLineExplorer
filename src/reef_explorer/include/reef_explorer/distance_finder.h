#ifndef DISTANCE_FINDER_H_
#define DISTANCE_FINDER_H_

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

class DistanceFinder
{

public:

    DistanceFinder(const std::string& rangeImageTopic, double splitFactorHeight, double splitFactorWidth);
    ~DistanceFinder();

    void setMinimumDistanceValuePointer(double* minimumDistanceValue);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);

protected:
private:
    double* minimumDistanceValue;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    std::string rangeImageTopic;
    double splitFactorHeight;
    double splitFactorWidth;
};

#endif
