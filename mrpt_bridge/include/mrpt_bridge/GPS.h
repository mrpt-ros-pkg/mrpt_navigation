//
// Created by raghavender on 12/08/17.
//

#ifndef MRPT_BRIDGE_GPS_H
#define MRPT_BRIDGE_GPS_H


#include <sensor_msgs/NavSatFix.h>
#include <mrpt/obs/CObservationGPS.h>

using namespace mrpt::obs;

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


        /** Convert mrpt::obs::CObservation -> sensor_msgs/NavSatFix
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
