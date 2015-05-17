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
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <limits>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf_conversions/tf_eigen.h>
#include <pcl/range_image/range_image.h>
#include <nav_msgs/OccupancyGrid.h>


#define HEIGHT 100



class ExploreAlgorithm
{

public:
    ExploreAlgorithm(const std::string& odometryTopic);
    ~ExploreAlgorithm();
    void runExploreAlgorithm();
    double* getMinimumDistanceValuePointer();
    volatile int* getSyncState();
  //  void pclCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

protected:
private:
    void controlDistanceToCliff();
    void setErrorValues();
    void followCliff();
    void initOdom();
    void getTF();
    void checkStateConditions();
    void moveRight();

    int state;
    double minimumDistanceValue;
    double scanDistance;
    double scanDistanceDownMovementOffset;
    ros::NodeHandle nh;
    ros::Publisher position_pub;
    nav_msgs::Odometry odom;
    std::string odometryTopic;

    tf::TransformListener listener;
    tf::StampedTransform transform;
//  tf::StampedTransform transformPointCloud;
//  tf::TransformBroadcaster transformBroadcaster;
    std::string tfName;
    std::string tfBaseName;
//  std::string tfPointCloudName;
    ros::Subscriber pclOctoMapSub;
    double moveRightPointCoordinateY;
    bool upwardMovement;
    bool rightMovement;
    double zSpeed;
    double ySpeed;
    double xSpeed;
    double rightMovementConstant;


    double heightLimitTop;
    double heightLimitBottom;

    // pid things
    double error;
    double total_error_I;
    double derivative_error_D;
    double last_error;
    double current_fix;
    volatile int syncState;


  //  image_transport::ImageTransport it;
  //  image_transport::Publisher pub;


};

#endif
