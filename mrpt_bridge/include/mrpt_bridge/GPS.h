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
	FILE: GPS.h
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#ifndef MRPT_BRIDGE_GPS_H
#define MRPT_BRIDGE_GPS_H


#include <sensor_msgs/NavSatFix.h>
#include <mrpt/obs/CObservationGPS.h>

using namespace mrpt::obs;

/// ROS message:    http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
/// MRPT message:   https://github.com/MRPT/mrpt/blob/master/libs/obs/include/mrpt/obs/CObservationGPS.h

namespace mrpt_bridge
{
    namespace GPS
    {
        /** Convert sensor_msgs/NavSatFix -> mrpt::obs::CObservationGPS
          *
          * \return true on sucessful conversion, false on any error.
          */
        bool ros2mrpt(const sensor_msgs::NavSatFix &msg,
                      CObservationGPS &obj);


        /** Convert mrpt::obs::CObservationGPS -> sensor_msgs/NavSatFix
          *  The user must supply the "msg_header" field to be copied into the output message object, since that part does not appear in MRPT classes.
          *
          *  Since COnservationGPS does not contain "position_covariance" and "position_covariance_type"
          *  sensor_msgs::NavSatFix::position_covariance_type and sensor_msgs::NavSatFix::position_covariance will be empty.
          * \return true on sucessful conversion, false on any error.
          */
        bool mrpt2ros(const CObservationGPS &obj,
                      const std_msgs::Header &msg_header,
                      sensor_msgs::NavSatFix &msg);
    }
}


#endif //MRPT_BRIDGE_GPS_H
