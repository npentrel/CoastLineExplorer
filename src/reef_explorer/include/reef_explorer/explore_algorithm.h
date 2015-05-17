#ifndef EXPLORE_ALGORITHM_H_
#define EXPLORE_ALGORITHM_H_

#include <stdlib.h>
#include <string.h>
#include <limits>
#include <vector>

//ROS
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <tf/transform_listener.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <tf_conversions/tf_eigen.h>
#include <pcl/range_image/range_image.h>
#include <nav_msgs/OccupancyGrid.h>

#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

class ExploreAlgorithm
{

public:
    ExploreAlgorithm(const std::string& odometryTopic = "/dataNavigator_G500RAUVI", const std::string& octomapTopic = "/octomap_full", const std::string& tfRobotName = "/girona500_RAUVI/base_link", const std::string& tfBaseName = "/world",
                     double heightLimitTop = -9.0, double heightLimitBottom = -50.0, double rightMovementConstant = 2.0, double xSpeed = 0.2, double ySpeed = 0.4, double zSpeed = 0.4,
                     double scanDistance = 5.0, double scanDistanceDownMovementOffset = 0.1, double rotateTimerCount = 10.0);
    ~ExploreAlgorithm();
    void runExploreAlgorithm();
    double* getMinimumDistanceValuePointer();

protected:
private:
    void controlDistanceToCliff();
    void setErrorValues();
    void followCliff();
    void initOdom();
    void getTF();
    void checkStateConditions();
    void moveRight();
    void rotateAround();
    octomap::point3d calculateRollPitchYaw(const octomap::point3d& direction,const octomap::point3d& robotDirection);

    void octomapCb(const octomap_msgs::Octomap::ConstPtr& msg);
    void callbackRotateTimer(const ros::TimerEvent& event);

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
    std::string tfName;
    std::string tfBaseName;
    double moveRightPointCoordinateY;
    bool upwardMovement;
    bool rightMovement;
    double zSpeed;
    double ySpeed;
    double xSpeed;
    double rightMovementConstant;


    double heightLimitTop;
    double heightLimitBottom;

    std::string octomapTopic;
    ros::Subscriber octomapSub;
    std::pair<std::vector<octomap::point3d>,octomap::point3d> missingRayVec;

    double lastYaw;
    bool changeStateNextIter;
    double counter;
    double lastRotateTimer;
    double rotateTimerCount;

};

#endif
