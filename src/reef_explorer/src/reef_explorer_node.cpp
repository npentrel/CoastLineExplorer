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


// ugly global variable


// current appraoch
std::string odometryTopic = "/dataNavigator_G500RAUVI";
std::string rangeImageTopic = "/girona500_RAUVI/rangecamera";

class DistanceFinder
{
public:
    DistanceFinder() : it_(nh_)
    {
        image_sub_ = it_.subscribe(rangeImageTopic, 1, &DistanceFinder::imageCb, this);
    }

    ~DistanceFinder()
    {

    }

    void setMinimumDistanceValuePointer(double* minimumDistanceValue)
    {
        this->minimumDistanceValue = minimumDistanceValue;
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
        }

        double min = std::numeric_limits<double>::max();
        for(int i = 0; i < cv_ptr->image.rows; i++)
        {
            for(int j = 0; j < cv_ptr->image.cols; j++)
            {
                if(cv_ptr->image.at<float>(i,j) < min)
                {
                    min = cv_ptr->image.at<float>(i,j);
                }
            }
        }
        *minimumDistanceValue = min;
    }

protected:
private:
    double* minimumDistanceValue;
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
};

class ExploreAlgorithm
{
public:
    ExploreAlgorithm()
    {
        this->minimumDistanceValue = std::numeric_limits<double>::max();
        this->state = 0;
        this->initOdom();
        this->position_pub = nh.advertise<nav_msgs::Odometry>(odometryTopic,1);
        this->scanDistance = 5.0;
        this->scanDistanceDownMovementOffset = 0.1;
    }

    ~ExploreAlgorithm()
    {

    }

    void runExploreAlgorithm()
    {
        std::cout << this->minimumDistanceValue << std::endl;
        switch(this->state)
        {
            case 0:
                this->moveTowardsCliff();
                break;
            case 1:
                this->followCliff();
                break;
            default:
            {

                break;
            }
        }
    }

    double* getMinimumDistanceValuePointer()
    {
        return &minimumDistanceValue;
    }

protected:
private:
    void moveTowardsCliff()
    {
        if(this->minimumDistanceValue > this->scanDistance)
        {
            this->odom.twist.twist.linear.x = 0.5;
        }
        else if(this->minimumDistanceValue < this->scanDistance - this->scanDistanceDownMovementOffset)
        {
            this->odom.twist.twist.linear.x = -0.2;
        }
        else
        {
            this->odom.twist.twist.linear.x = 0.0;
            this->state = 1;
        }
        this->position_pub.publish(this->odom);
    }

    void followCliff()
    {
        if(this->minimumDistanceValue > this->scanDistance)
        {
            this->odom.twist.twist.linear.x = 0.1;
        }
        else
        {
            this->odom.twist.twist.linear.x = -0.1;
        }

        if(this->minimumDistanceValue > this->scanDistanceDownMovementOffset + this->scanDistance ||
           this->minimumDistanceValue < this->scanDistanceDownMovementOffset - this->scanDistance)
        {
            this->odom.twist.twist.linear.z = 0.0;
        }
        else
        {
            this->odom.twist.twist.linear.z = 0.1;  
        }
        this->position_pub.publish(this->odom);
    }


    void initOdom()
    {
        this->odom.pose.pose.position.x = 0.0;
        this->odom.pose.pose.position.y = 0.0;
        this->odom.pose.pose.position.z = 0.0;
        this->odom.pose.pose.orientation.x = 0.0;
        this->odom.pose.pose.orientation.y = 0.0;
        this->odom.pose.pose.orientation.z = 0.0;
        this->odom.pose.pose.orientation.w = 1;
        this->odom.twist.twist.linear.x = 0;
        this->odom.twist.twist.linear.y = 0;
        this->odom.twist.twist.linear.z = 0;
        this->odom.twist.twist.angular.x = 0;
        this->odom.twist.twist.angular.y = 0;
        this->odom.twist.twist.angular.z = 0;
        for (int i=0; i<36; i++)
        {
            this->odom.twist.covariance[i] = 0;
            this->odom.pose.covariance[i] = 0;
        }        
    }

    int state;
    double minimumDistanceValue;
    double scanDistance;
    double scanDistanceDownMovementOffset;
    ros::NodeHandle nh;
    ros::Publisher position_pub;
    nav_msgs::Odometry odom;
};




int main(int argc, char **argv)
{
    ros::init(argc, argv, "reef_explorer");
    ExploreAlgorithm exploreAlgorithm;
    DistanceFinder distanceFinder;
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
