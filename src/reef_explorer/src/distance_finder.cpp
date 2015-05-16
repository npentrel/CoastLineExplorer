#include "../include/reef_explorer/distance_finder.h"


DistanceFinder::DistanceFinder(const std::string& rangeImageTopic, const std::string& jointTopic, const std::string& panTopic, const std::string& tiltTopic, double splitFactorHeight, double splitFactorWidth) : it_(nh_)
{
    this->rangeImageTopic = rangeImageTopic;
    this->panTopic = panTopic;
    this->tiltTopic = tiltTopic;
    this->jointTopic = jointTopic;
    this->image_sub_ = it_.subscribe(this->rangeImageTopic, 1, &DistanceFinder::imageCb, this);
    this->splitFactorHeight = splitFactorHeight;
    this->splitFactorWidth = splitFactorWidth;
    this->pub = this->nh_.advertise<sensor_msgs::JointState>(this->jointTopic, 1000);
    this->state = 0;
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
    float sleepFreq = 30;
    std::cout << this->state << std::endl;
    switch(this->state)
    {
        case 0:
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
                this->state = 1;
            }
            break;
        case 1:
            {
                double positions[2] = {1.0, 0.0};
                std::vector<double> vec(positions, positions + sizeof(positions)/ sizeof(double));
                sensor_msgs::JointState msg;
                msg.header.stamp = ros::Time::now();
                msg.name.resize(2);
                msg.name[0] = this->panTopic;
                msg.name[1] = this->tiltTopic;
                msg.position = vec;
                pub.publish(msg);
                // spin once to let tf publish
                ros::Rate r(sleepFreq);
                r.sleep();
                ros::spinOnce();
                this->state = 2;
            }
            break;
        case 2:
            {
                double positions[2] = {-1.0, 0.0};
                std::vector<double> vec(positions, positions + sizeof(positions)/ sizeof(double));
                sensor_msgs::JointState msg;
                msg.header.stamp = ros::Time::now();
                msg.name.resize(2);
                msg.name[0] = this->panTopic;
                msg.name[1] = this->tiltTopic;
                msg.position = vec;
                pub.publish(msg);
                // spin once to let tf publish
                ros::Rate r(sleepFreq);
                r.sleep();     
                ros::spinOnce();
                this->state = 3;
            }
            break;
        case 3:
            {
                double positions[2] = {0.0, 1.0};
                std::vector<double> vec(positions, positions + sizeof(positions)/ sizeof(double));
                sensor_msgs::JointState msg;
                msg.header.stamp = ros::Time::now();
                msg.name.resize(2);
                msg.name[0] = this->panTopic;
                msg.name[1] = this->tiltTopic;
                msg.position = vec;
                pub.publish(msg);
                // spin once to let tf publish
                ros::Rate r(sleepFreq);
                r.sleep();
                ros::spinOnce();
                this->state = 4;
            }
            break;
        case 4:
            {
                double positions[2] = {0.0, -1.0};
                std::vector<double> vec(positions, positions + sizeof(positions)/ sizeof(double));
                sensor_msgs::JointState msg;
                msg.header.stamp = ros::Time::now();
                msg.name.resize(2);
                msg.name[0] = this->panTopic;
                msg.name[1] = this->tiltTopic;
                msg.position = vec;
                pub.publish(msg);
                // spin once to let tf publish
                ros::Rate r(sleepFreq);
                r.sleep();
                ros::spinOnce();
                this->state = 0;
                {
                double positions[2] = {0.0, 0.0};
                std::vector<double> vec(positions, positions + sizeof(positions)/ sizeof(double));
                sensor_msgs::JointState msgJoint;
                msgJoint.header.stamp = ros::Time::now();
                msgJoint.name.resize(2);
                msgJoint.name[0] = this->panTopic;
                msgJoint.name[1] = this->tiltTopic;
                msgJoint.position = vec;
                pub.publish(msgJoint);
                // spin once to let tf publish               
                ros::Rate r(sleepFreq);
                r.sleep();
                ros::spinOnce();
                }
            }
            break;
        default:
            this->state = 0;
    }
}
