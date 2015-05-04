#include "../include/reef_explorer/distance_finder.h"


DistanceFinder::DistanceFinder(const std::string& rangeImageTopic) : it_(nh_)
{
    this->rangeImageTopic = rangeImageTopic;
    this->image_sub_ = it_.subscribe(rangeImageTopic, 1, &DistanceFinder::imageCb, this);
}

DistanceFinder::~DistanceFinder()
{

}

void DistanceFinder::setMinimumDistanceValuePointer(double* minimumDistanceValue)
{
    this->minimumDistanceValue = minimumDistanceValue;
}

void DistanceFinder::imageCb(const sensor_msgs::ImageConstPtr& msg)
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
