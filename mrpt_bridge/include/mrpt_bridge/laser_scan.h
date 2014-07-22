#ifndef MRPT_BRIDGE_LASER_SCAN_H
#define MRPT_BRIDGE_LASER_SCAN_H

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
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
	bool ros2mrpt(const sensor_msgs::LaserScan &_msg, const mrpt::poses::CPose3D &_pose, mrpt::slam::CObservation2DRangeScan &_obj);

    /**
      * \return true on sucessful conversion, false on any error.
      * \sa ros2mrpt
      */
    bool mrpt2ros(const mrpt::slam::CObservation2DRangeScan &_obj, sensor_msgs::LaserScan &_msg);
    
    /**
      * \return true on sucessful conversion, false on any error.
      * \sa ros2mrpt
      */
    bool mrpt2ros(const mrpt::slam::CObservation2DRangeScan &_obj, sensor_msgs::LaserScan &_msg, geometry_msgs::Pose &_pose);
};

} //namespace mrpt_bridge

#endif //MRPT_BRIDGE_LASER_SCAN_H
