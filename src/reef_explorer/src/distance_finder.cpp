#include "../include/reef_explorer/distance_finder.h"


DistanceFinder::DistanceFinder(const std::string& rangeImageTopic, double splitFactorHeight, double splitFactorWidth) : it_(nh_)
{
    this->rangeImageTopic = rangeImageTopic;
    this->image_sub_ = it_.subscribe(rangeImageTopic, 1, &DistanceFinder::imageCb, this);
    this->splitFactorHeight = splitFactorHeight;
    this->splitFactorWidth = splitFactorWidth;
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
    int widthMin = 0;
    int widthMax = cv_ptr->image.rows;
    if(0 < this->splitFactorWidth || this->splitFactorWidth < 0.5)
    {
        widthMin = this->splitFactorWidth * cv_ptr->image.rows;
        widthMax = (1.0 - this->splitFactorWidth) * cv_ptr->image.rows;
    }
    int heightMin = 0;
    int heightMax = cv_ptr->image.rows;
    if(0 < this->splitFactorHeight || this->splitFactorHeight < 0.5)
    {
        heightMin = this->splitFactorHeight * cv_ptr->image.cols;
        heightMax = (1.0 - this->splitFactorHeight) * cv_ptr->image.cols;
    }
    for(int i = widthMin; i < widthMax; i++)
    {
        for(int j = heightMin; j < heightMax; j++)
        {
            if(cv_ptr->image.at<float>(i,j) < min)
            {
                min = cv_ptr->image.at<float>(i,j);
            }
        }
    }
    *minimumDistanceValue = min;
}
