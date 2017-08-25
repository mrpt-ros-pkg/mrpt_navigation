/* +------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)            |
   |                          http://www.mrpt.org/                          |
   |                                                                        |
   | Copyright (c) 2005-2017, Individual contributors, see AUTHORS file     |
   | See: http://www.mrpt.org/Authors - All rights reserved.                |
   | Released under BSD License. See details in http://www.mrpt.org/License |
   +------------------------------------------------------------------------+ */

/*---------------------------------------------------------------
	APPLICATION: mrpt_ros bridge
	FILE: imu.h
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/
#ifndef MRPT_BRIDGE_IMU_H
#define MRPT_BRIDGE_IMU_H

#include <cstring> // size_t
#include <sensor_msgs/Imu.h>
#include <mrpt/obs/CObservationIMU.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>


using namespace mrpt::obs;


/// ROS message:    http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
/// MRPT message:   https://github.com/MRPT/mrpt/blob/master/libs/obs/include/mrpt/obs/CObservationIMU.h

namespace mrpt_bridge
{
    namespace imu
    {

        /** Convert sensor_msgs/Imu -> mrpt::obs::CObservationIMU
          * // STILL NEED TO WRITE CODE FOR COVARIANCE
          * \return true on sucessful conversion, false on any error.
          */
        bool ros2mrpt(const sensor_msgs::Imu &msg, CObservationIMU obj);

        /** Convert mrpt::obs::CObservationIMU -> sensor_msgs/Imu
          *  The user must supply the "msg_header" field to be copied into the output message object, since that part does not appear in MRPT classes.
          *
          *  Since COnservationIMU does not contain covariance terms NEED TO fix those.
          * \return true on sucessful conversion, false on any error.
          */
        bool mrpt2ros(const CObservationIMU &obj, const std_msgs::Header &msg_header, sensor_msgs::Imu &msg);

    }
}



#endif //MRPT_BRIDGE_IMU_H



