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
    callback_count = 0;
    callback_refresh = 1; // every 'callback_refresh' calls of the Subscribe function, the position will be updated.
}

PositionSubscriber::~PositionSubscriber()
{
    // TODO Auto-generated destructor stub
}

void PositionSubscriber::Subscribe(const sensor_msgs::NavSatFix& position)
{
    callback_count++;

    if (callback_count % callback_refresh == 0)
    {       
        initialized = true;
        position_ = position;
        ROS_INFO("Received new position for %s: (%f, %f, %f)", this->robot_name_.c_str(), position.latitude, position.longitude, position.altitude);
    }
}

double PositionSubscriber::getLatitude()
{
	return position_.latitude;
}

double PositionSubscriber::getLongitude()
{
	return position_.longitude;
}

double PositionSubscriber::getAltitude()
{
	return position_.altitude;
}

double PositionSubscriber::calcDistance(PositionSubscriber* other)
{ 
    // ROS_ERROR("P1: (%f,%f,%f)", getLatitude(), getLongitude(), getAltitude());
    // ROS_ERROR("P2: (%f,%f,%f)", other->getLatitude(), other->getLongitude(), other->getAltitude());
	// ROS_ERROR("me [%s] %u other [%s] %u", this->robot_name_.c_str(), initialized, other->robot_name_.c_str(), other->initialized);
    
    if (initialized && other->initialized)
        return sqrt(pow(getLatitude() - other->getLatitude(), 2) + pow(getLongitude() - other->getLongitude(), 2) + pow(getAltitude() - other->getAltitude(), 2));
    else
        return -1;
}


