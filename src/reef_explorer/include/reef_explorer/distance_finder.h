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
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

class DistanceFinder
{

public:
    DistanceFinder(const std::string& rangeImageTopic, const std::string& jointTopic, const std::string& panTopic, const std::string& tiltTopic, double splitFactorHeight, double splitFactorWidth, volatile int* syncState);
    ~DistanceFinder();

    void setMinimumDistanceValuePointer(double* minimumDistanceValue);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);    
    void cloudCb(const sensor_msgs::PointCloud2ConstPtr& input);

protected:
private:
    double* minimumDistanceValue;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber cloud_sub;
    std::string rangeImageTopic;
    std::string panTopic;
    std::string tiltTopic;
    std::string jointTopic;
    double splitFactorHeight;
    double splitFactorWidth;
    ros::Publisher pub;
    volatile int state;
    volatile bool moveSensor;
    volatile int* syncState;
    volatile bool movingSensorAtm;

};

#endif
