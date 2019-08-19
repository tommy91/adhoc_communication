/*
 * PositionSubscriber.cpp
 *
 *  Created on: Dec 11, 2013
 *      Author: cwioro
 */

#include "PositionSubscriber.h"
#include <cmath>
#include <string>

PositionSubscriber::PositionSubscriber(std::string robot_name, uint32_t robot_number)
{
	robot_name_ = robot_name;
	robot_number_ = robot_number;
    initialized = false;
    x_pos_ = 0;
    y_pos_ = 0;
    callback_count = 0;
    callback_refresh = 1; // every 'callback_refresh' calls of the Subscribe function, the position will be updated.
}

PositionSubscriber::~PositionSubscriber()
{
    // TODO Auto-generated destructor stub
}

void PositionSubscriber::Subscribe(const nav_msgs::Odometry::ConstPtr& position)
{
    callback_count++;

    if (callback_count % callback_refresh == 0)
    {       
        initialized = true;
        this->x_pos_ = position->pose.pose.position.x;
        this->y_pos_ = position->pose.pose.position.y;
        ROS_INFO("Received new position for %s: (%f, %f)", this->robot_name_.c_str(), this->x_pos_, this->y_pos_);

        //	x_pos_ = 7.3f;
    }
}

double PositionSubscriber::calcDistance(PositionSubscriber* other)
{ 
    //ROS_DEBUG("P1: %f / %f",this->x_pos_,this->y_pos_);
    //ROS_DEBUG("P2: %f / %f",other->x_pos_,other->y_pos_);
   // ROS_ERROR("me [%s] %u other [%s] %u", this->robot_name_.c_str(), initialized ,other->robot_name_.c_str() ,other->initialized);
    
    if (initialized && other->initialized)
        return sqrt(pow(x_pos_ - other->x_pos_, 2) + pow(y_pos_ - other->y_pos_, 2));
    else
        return -1;
}


