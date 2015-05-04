#include "../include/reef_explorer/explore_algorithm.h"

ExploreAlgorithm::ExploreAlgorithm(const std::string& odometryTopic)
{
    this->minimumDistanceValue = std::numeric_limits<double>::max();
    this->state = 0;
    this->initOdom();
    this->position_pub = nh.advertise<nav_msgs::Odometry>(odometryTopic,1);
    this->scanDistance = 5.0;
    this->scanDistanceDownMovementOffset = 0.1;
    this->odometryTopic = odometryTopic;
}

ExploreAlgorithm::~ExploreAlgorithm()
{

}

void ExploreAlgorithm::runExploreAlgorithm()
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
double* ExploreAlgorithm::getMinimumDistanceValuePointer()
{
    return &minimumDistanceValue;
}

void ExploreAlgorithm::moveTowardsCliff()
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

void ExploreAlgorithm::followCliff()
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

void ExploreAlgorithm::initOdom()
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
