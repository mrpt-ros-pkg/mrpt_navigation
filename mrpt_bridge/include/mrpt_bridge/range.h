//
// Created by raghavender on 12/08/17.
//

#ifndef MRPT_BRIDGE_RANGE_H
#define MRPT_BRIDGE_RANGE_H

#include <sensor_msgs/Range.h>
#include <mrpt/obs/CObservationRange.h>

using namespace mrpt::obs;


/// ROS message :   http://docs.ros.org/api/sensor_msgs/html/msg/Range.html
/// MRPT message:   https://github.com/MRPT/mrpt/blob/master/libs/obs/include/mrpt/obs/CObservationRange.h

namespace mrpt_bridge
{
    namespace range
    {
        /** Convert sensor_msgs/Range -> mrpt::obs::CObservationRange
          *
          * \return true on sucessful conversion, false on any error.
          */
        bool ros2mrpt(const sensor_msgs::Range &msg,
                      CObservationRange &obj);

        /** Convert mrpt::obs::CObservationRange -> sensor_msgs/Range
          *  The user must supply the "msg_header" field to be copied into the output message object, since that part does not appear in MRPT classes.
          *
          *  Since COnservation does not contain "radiation_type", sensor_msgs::Range::radiation_type will be empty.
          * \return true on sucessful conversion, false on any error.
          */
        bool mrpt2ros(const CObservationRange &obj,
                      const std_msgs::Header &msg_header,
                      sensor_msgs::Range &msg);
    }
}

#endif //MRPT_BRIDGE_RANGE_H
