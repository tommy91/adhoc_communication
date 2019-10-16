/*!
 * @file PositionSubscriber.h
 *
 * \date 11.11.2013
 * \author: Günther Cwioro, Torsten Andre
 */

#ifndef POSITIONSUBSCRIBER_H_
#define POSITIONSUBSCRIBER_H_

/*!
\class PositionSubscriber
\brief Subscripes to the position of the stage simulation

The PositionSubscripe class implements all function to build up a channel model in the stage simulation
*/
class PositionSubscriber {
public:

	PositionSubscriber(std::string robot_name, uint32_t robot_number) ;
	virtual ~PositionSubscriber();

	std::string getRobotName();
	uint32_t getRobotNumber();
	bool isInitialized();

	double getLatitude();
	double getLongitude();
	double getAltitude();

	double getX();
	double getY();
	double getZ();

	double calcDistance(PositionSubscriber* other);

	void Subscribe(const sensor_msgs::NavSatFix& position);

private:
	uint16_t callback_count, callback_refresh;
	bool initialized; 						///< Defines if the robot position has been initialized
	std::string robot_name; 				///< Name of the robot in stage. e.g: "robot_0"
	uint32_t robot_number; 					///< Number of the robot in stage. e.g: number of "robot_0" would be "0"

	sensor_msgs::NavSatFix global_position; ///< Latest position of the specific robot (LLA)
	double x, y, z;							///< Local position (xyz)

	double pi, a, b, f, oe;

};

#endif /* POSITIONSUBSCRIBER_H_ */
