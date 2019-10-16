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
	this->robot_name = robot_name;
	this->robot_number = robot_number;
    initialized = false;
    callback_count = 0;
    callback_refresh = 10; // every 'callback_refresh' calls of the Subscribe function, the position will be updated.

	pi = 3.14159265358979323846;

	/* WGS-84 constants */
	a = 6378137;  			// semi-major axis or equatorial radius (m)
	b = 6356752.31425;  	// semi-minor axis or polar radius (m)
	f = 1 / 298.257223563;	// flattening [ f=(a-b)/a ]
	oe = acos(b/a);
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

        global_position = position;

        double rad_lat = position.latitude * pi / 180;
		double rad_lon = position.longitude * pi / 180;

		double n = a / sqrt(1 - pow(sin(rad_lat) * sin(oe), 2));

		x = (n + position.altitude) * cos(rad_lat) * cos(rad_lon);
		y = (n + position.altitude) * cos(rad_lat) * sin(rad_lon);
		z = ((pow(cos(oe),2) * n) + position.altitude) * sin(rad_lat);

        ROS_INFO("Received new position for %s: (%f, %f, %f)", this->robot_name.c_str(), position.latitude, position.longitude, position.altitude);
    }
}

std::string PositionSubscriber::getRobotName()
{
	return robot_name;
}

uint32_t PositionSubscriber::getRobotNumber()
{
	return robot_number;
}

bool PositionSubscriber::isInitialized()
{
	return initialized;
}

double PositionSubscriber::getLatitude()
{
	return global_position.latitude;
}

double PositionSubscriber::getLongitude()
{
	return global_position.longitude;
}

double PositionSubscriber::getAltitude()
{
	return global_position.altitude;
}

double PositionSubscriber::getX()
{
	return x;
}

double PositionSubscriber::getY()
{
	return y;
}

double PositionSubscriber::getZ()
{
	return z;
}

double PositionSubscriber::calcDistance(PositionSubscriber* other)
{
	// ROS_ERROR("me [%s] %u other [%s] %u", robot_name.c_str(), initialized, other->getRobotName().c_str(), other->isInitialized());

    if (initialized && other->isInitialized())
    {
		double d = sqrt(pow(x - other->getX(), 2) + pow(y - other->getY(), 2) + pow(z - other->getZ(), 2));

//		ROS_ERROR("LLA P1: (%f,%f,%f)", getLatitude(), getLongitude(), getAltitude());
//		ROS_ERROR("LLA P2: (%f,%f,%f)", other->getLatitude(), other->getLongitude(), other->getAltitude());
//		ROS_ERROR("xyz P1: (%f,%f,%f)", x, y, z);
//		ROS_ERROR("xyz P2: (%f,%f,%f)", other->getX(), other->getY(), other->getZ());
//		ROS_ERROR("distance: %f", d);

		return d;
    }
    else
        return -1;
}


