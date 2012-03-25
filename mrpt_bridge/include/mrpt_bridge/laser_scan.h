#ifndef MRPT_LASER_SCAN_H
#define MRPT_LASER_SCAN_H

#include <sensor_msgs/LaserScan.h>
#include <mrpt/slam/CObservation2DRangeScan.h>

namespace mrpt_bridge {

/** Methods to convert between ROS msgs and MRPT objects for laser-scan datatypes.
  */
namespace laser_scan
{
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
