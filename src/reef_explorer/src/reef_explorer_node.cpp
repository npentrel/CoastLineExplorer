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
double minimumValue;

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
        minimumValue = minimumValue;
        std::cout << "Cur min: " << min << "\n";
    }

protected:
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reef_explorer");
    DistanceFinder distanceFinder;
	ros::NodeHandle nh;
	ros::Publisher position_pub;
	position_pub=nh.advertise<nav_msgs::Odometry>(odometryTopic,1);

	//ros::NodeHandle nh2;
	//ros::Subscriber sub = nh2.subscribe<PointCloud>("/girona500_RAUVI/points", 1, callback);

	ros::Rate r(25);
	while (ros::ok())
    {
		nav_msgs::Odometry odom;
		odom.pose.pose.position.x=0.0;
		odom.pose.pose.position.y=0.0;
		odom.pose.pose.position.z=0.0;
		odom.pose.pose.orientation.x=0.0;
		odom.pose.pose.orientation.y=0.0;
		odom.pose.pose.orientation.z=0.0;
		odom.pose.pose.orientation.w=1;

		odom.twist.twist.linear.x=0;
		odom.twist.twist.linear.y=0;
		odom.twist.twist.linear.z=0.5;
		odom.twist.twist.angular.x=0;
		odom.twist.twist.angular.y=0;
		odom.twist.twist.angular.z=0;
		for (int i=0; i<36; i++) {
			odom.twist.covariance[i]=0;
			odom.pose.covariance[i]=0;
		}
		position_pub.publish(odom);

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
