#include "../include/reef_explorer/explore_algorithm.h"


ExploreAlgorithm::ExploreAlgorithm(const std::string& odometryTopic, const std::string& octomapTopic, const std::string& tfRobotName, const std::string& tfBaseName,
                                   double heightLimitTop, double heightLimitBottom, double rightMovementConstant, double xSpeed, double ySpeed, double zSpeed,
                                   double scanDistance, double scanDistanceDownMovementOffset, double rotateTimerCount)
{
    this->minimumDistanceValue = std::numeric_limits<double>::max();
    this->scanDistance = scanDistance;
    this->scanDistanceDownMovementOffset = scanDistanceDownMovementOffset;
    this->odometryTopic = odometryTopic;
    this->tfName = tfRobotName;
    this->tfBaseName = tfBaseName;
    this->octomapTopic = octomapTopic;
    this->upwardMovement = false;
    this->rightMovement = false;
    this->rightMovementConstant = rightMovementConstant;
    this->xSpeed = xSpeed;
    this->ySpeed = ySpeed;
    this->zSpeed = zSpeed;
    this->heightLimitTop = heightLimitTop;
    this->heightLimitBottom = heightLimitBottom;
    this->rotateTimerCount = rotateTimerCount;
    this->initOdom();
    this->position_pub = nh.advertise<nav_msgs::Odometry>(this->odometryTopic,1);
    this->octomapSub = this->nh.subscribe(this->octomapTopic, 1, &ExploreAlgorithm::octomapCb, this);
    this->getTF();
    this->state = 0;
    this->lastRotateTimer = ros::Time::now().toSec() - 2 * this->rotateTimerCount;
}

ExploreAlgorithm::~ExploreAlgorithm()
{

}

void ExploreAlgorithm::runExploreAlgorithm()
{
    if((this->lastRotateTimer + this->rotateTimerCount) < ros::Time::now().toSec() && this->state != 3 && this->state != 2)
    {
        this->state = 3;
        tf::Quaternion q = this->transform.getRotation();
        tf::Matrix3x3 m(q);
        double roll;
        double pitch;
        double yaw;
        m.getRPY(roll, pitch, yaw);
        this->lastYaw = yaw + M_PI;
        this->counter = 0.0;
    }

    this->initOdom();
    this->getTF();
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
        case 3:
            this->rotateAround();
            break;
        default:
            this->state = 0;
            break;
    }
}

double* ExploreAlgorithm::getMinimumDistanceValuePointer()
{
    return &minimumDistanceValue;
}

void ExploreAlgorithm::controlDistanceToCliff()
{
    if(this->minimumDistanceValue > this->scanDistance)
    {
        this->odom.twist.twist.linear.x = this->xSpeed * 5;
    }
    else if(this->minimumDistanceValue < this->scanDistance - this->scanDistanceDownMovementOffset)
    {
        this->odom.twist.twist.linear.x = -this->xSpeed * 2;
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

void ExploreAlgorithm::initOdom()
{
    this->odom.pose.pose.position.x = 0.0;
    this->odom.pose.pose.position.y = 0.0;
    this->odom.pose.pose.position.z = 0.0;
    this->odom.pose.pose.orientation.x = 0.0;
    this->odom.pose.pose.orientation.y = 0.0;
    this->odom.pose.pose.orientation.z = 0.0;
    this->odom.pose.pose.orientation.w = 1.0;
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
        this->listener.waitForTransform(this->tfBaseName, this->tfName, ros::Time(0), ros::Duration(1.0));
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

void ExploreAlgorithm::rotateAround()
{
    tf::Quaternion q = this->transform.getRotation();
    tf::Matrix3x3 m(q);
    double roll;
    double pitch;
    double yaw;
    m.getRPY(roll, pitch, yaw);
    this->odom.twist.twist.angular.y = 0;
    this->odom.twist.twist.angular.z = 0;
    this->odom.twist.twist.angular.z = 1;
    yaw = yaw + M_PI;

    double step = (this->lastYaw-yaw);
    if(step < 0.0)
    {
        step = -step;
    }
    this->counter += step;
    if(this->counter >= 2 * M_PI)
    {
        this->odom.twist.twist.angular.z = -1;
        this->counter = 0.0;
        this->state = 0;
        this->lastRotateTimer = ros::Time::now().toSec();
    }
    this->position_pub.publish(this->odom);
    this->lastYaw = yaw;
}

void ExploreAlgorithm::octomapCb(const octomap_msgs::Octomap::ConstPtr& msg)
{
    /*
    octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);    
    octomap::point3d origin(this->transform.getOrigin().x(), this->transform.getOrigin().y(), this->transform.getOrigin().z());
    octomap::point3d end(0.0,0.0,0.0);

    this->missingRayVec.first.clear();
    this->missingRayVec.second = origin;
    double maxRange = 9.0;
    double stepSize = 1.0;
    for(double i = -1.0; i <= 1.0; i += stepSize)
    {
        for(double j = -1.0; j <= 1.0; j += stepSize)
        {
            for(double k = -1.0; k <= 1.0; k += stepSize)
            {
                if(i != 0.0 || j != 0.0 || k != 0.0)
                {
                    octomap::point3d direction(i,j,k);     
                    if(!octree->castRay(origin, direction, end, true, maxRange))
                    {
                        this->missingRayVec.first.push_back(end);
                    }  
                }
            }        
        }        
    }
    if(!this->missingRayVec.first.empty() && this->state != 3)
    {
        std::cout << "gere" << std::endl;
        tf::Quaternion q = this->transform.getRotation();
        tf::Matrix3x3 m(q);
        m.getRPY(this->resetRoll, this->resetPitch, this->resetYaw);
        this->resetRoll += M_PI;
        this->resetPitch += M_PI;
        this->resetYaw += M_PI;
        this->lastRoll = this->resetRoll;
        this->lastPitch = this->resetPitch;
        this->lastYaw = this->resetYaw;
        this->rotateFlag = 0;
        this->yawFlag = true;
        this->state = 3;
    }
    if(tree)
    {
        delete(tree);
    }    
    */
}
