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
    this->error = 0;
    this->total_error_I = 0;
    this->derivative_error_D = 0;
    this->current_fix = 1;
    this->last_error = 0;
    this->tfName = "girona500_RAUVI/base_link";
    this->tfBaseName = "world";
    this->upwardMovement = false;
    this->rightMovement = false;
    this->rightMovementConstant = 3.0;
    this->zSpeed = 0.4;
    this->ySpeed = 0.4;
    this->xSpeed = 0.2;
    this->heightLimitTop = -9.0;
    this->heightLimitBottom = -50.0;
    this->getTF();
}

ExploreAlgorithm::~ExploreAlgorithm() {

}

void ExploreAlgorithm::runExploreAlgorithm()
{
    setErrorValues();
    this->initOdom();
    this->getTF();
    std::cout << "Distance: " << this->minimumDistanceValue << "\n";
    std::cout << "Pos Z: " << this->transform.getOrigin().z() << "\n";
    this->checkStateConditions();

    switch(this->state)
    {
        case 0:
            this->controlDistanceToCliff();
            break;
        case 1:
            this->followCliff();
            break;
        case 2:
            this->moveRight();
            break;
        default:
            break;
    }

}

double* ExploreAlgorithm::getMinimumDistanceValuePointer()
{
    return &minimumDistanceValue;
}

void ExploreAlgorithm::controlDistanceToCliff() {
    if(this->minimumDistanceValue > this->scanDistance) {
        this->odom.twist.twist.linear.x = this->xSpeed * 5;
    } else if(this->minimumDistanceValue < this->scanDistance - this->scanDistanceDownMovementOffset) {
        this->odom.twist.twist.linear.x = -this->xSpeed * 2;
    } else {
        this->odom.twist.twist.linear.x = 0.0;
        this->state = 1;
    }

    this->position_pub.publish(this->odom);
}

void ExploreAlgorithm::setErrorValues()
{
    this->error = this->minimumDistanceValue - this->scanDistance;
    if (this->error < 100)
    {
        this->total_error_I = this->total_error_I + this->error;
        this->derivative_error_D = this->last_error-this->error;

        this->current_fix = 1*(this->error) + 0.001*(this->total_error_I) + 0.5*(this->derivative_error_D);
        this->last_error = this->error; 
    }

    std::cout << "ERROR: " << this->error << std::endl;
    std::cout << "TOTAL ERROR: " << this->total_error_I << std::endl;
    std::cout << "DERIVATIVE ERROR: " << this->derivative_error_D << std::endl;
    std::cout << "FIX ERROR: " << this->current_fix << std::endl;       
}

/*
From PID keep for now
    if (this->minimumDistanceValue > this->scanDistance) {
        this->odom.twist.twist.linear.x = this->current_fix;
*/
void ExploreAlgorithm::followCliff()
{
    if(this->upwardMovement)
    {
        if(this->minimumDistanceValue > this->scanDistance)
        {
            this->odom.twist.twist.linear.x = this->xSpeed;
        }
        else
        {
            this->odom.twist.twist.linear.x = -this->xSpeed;
        }
        if(this->minimumDistanceValue > this->scanDistanceDownMovementOffset + this->scanDistance ||
           this->minimumDistanceValue < this->scanDistanceDownMovementOffset - this->scanDistance)
        {
            this->odom.twist.twist.linear.z = 0.0;
        }
        else
        {
            this->odom.twist.twist.linear.z = -this->zSpeed;  
        }      
    }
    else
    {
        if(this->minimumDistanceValue > this->scanDistance)
        {
            this->odom.twist.twist.linear.x = this->xSpeed;
        }
        else
        {
            this->odom.twist.twist.linear.x = -this->xSpeed;
        }
        if(this->minimumDistanceValue > this->scanDistanceDownMovementOffset + this->scanDistance ||
           this->minimumDistanceValue < this->scanDistanceDownMovementOffset - this->scanDistance)
        {
            this->odom.twist.twist.linear.z = 0.0;
        }
        else
        {
            this->odom.twist.twist.linear.z = this->zSpeed;  
        }       
    }
    this->position_pub.publish(this->odom);
}

void ExploreAlgorithm::initOdom() {
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

void ExploreAlgorithm::getTF()
{
    try
    {
        this->listener.lookupTransform(this->tfBaseName, this->tfName, ros::Time(0), this->transform);
    }
    catch (tf::TransformException &ex) 
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
}

void ExploreAlgorithm::checkStateConditions()
{
    if(this->transform.getOrigin().z() > this->heightLimitTop && this->rightMovement == false)
    {
        this->upwardMovement = false;
        this->rightMovement = true;
        this->state = 2;
        this->moveRightPointCoordinateY = this->transform.getOrigin().y();
    }
    else if(this->transform.getOrigin().z() < this->heightLimitBottom && this->rightMovement == false)
    {
        this->upwardMovement = true; 
        this->rightMovement = true;
        this->state = 2;
        this->moveRightPointCoordinateY = this->transform.getOrigin().y();
    }
}

void ExploreAlgorithm::moveRight()
{
    if(this->moveRightPointCoordinateY + this->rightMovementConstant < this->transform.getOrigin().y() &&
       this->transform.getOrigin().z() >= this->heightLimitBottom &&
       this->transform.getOrigin().z() <= this->heightLimitTop)
    {
        this->rightMovement = false;
        this->state = 1;
    }
    else
    {
        if(this->minimumDistanceValue > this->scanDistance)
        {
            this->odom.twist.twist.linear.x = this->xSpeed;
        }
        else
        {
            this->odom.twist.twist.linear.x = -this->xSpeed;
        }
        this->odom.twist.twist.linear.y = this->ySpeed;
        if(this->upwardMovement && this->transform.getOrigin().z() < this->heightLimitBottom)
        {
            this->odom.twist.twist.linear.z = -this->zSpeed;
        }
        else if(!this->upwardMovement && this->transform.getOrigin().z() > this->heightLimitTop)
        {
            this->odom.twist.twist.linear.z = this->zSpeed;
        }
    }
    this->position_pub.publish(this->odom);
}

