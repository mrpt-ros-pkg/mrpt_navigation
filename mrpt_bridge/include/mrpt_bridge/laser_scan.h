#ifndef MRPT_LASER_SCAN_H
#define MRPT_LASER_SCAN_H

#include <sensor_msgs/LaserScan.h>
#include <mrpt/slam/CObservation2DRangeScan.h>

namespace mrpt_bridge {

/** This class contains methods to convert between ROS msgs and MRPT objects for laser-scan datatypes.
  * \note Methods are not static, so you will need to instantiate an object of this converter class. This is done on purpose so options and switches can be added to the class in the future.
  */
class LaserScan
{
public:
	/**
	  * \return true on sucessful conversion, false on any error.
	  * \sa mrpt2ros
	  */
	bool ros2mrpt(
		const sensor_msgs::LaserScan &msg,
		mrpt::slam::CObservation2DRangeScan  &obj
		);

	/**
	  * \return true on sucessful conversion, false on any error.
	  * \sa ros2mrpt
	  */
	bool mrpt2ros(
		const mrpt::slam::CObservation2DRangeScan  &obj,
		const std_msgs::Header &msg_header,
		sensor_msgs::LaserScan &msg
		);
};

} //namespace mrpt_bridge

#endif
