//
// Created by raghavender on 12/08/17.
//

#ifndef MRPT_BRIDGE_RANGE_H
#define MRPT_BRIDGE_RANGE_H

#include <sensor_msgs/Range.h>
#include <mrpt/obs/CObservationRange.h>

using namespace mrpt::obs;

namespace mrpt_bridge
{
    namespace range
    {
        bool ros2mrpt(const sensor_msgs::Range &msg,
                      CObservationRange &obj);

        bool mrpt2ros(const CObservationRange &obj,
                      const std_msgs::Header &msg_header,
                      sensor_msgs::Range &msg);
    }
}

#endif //MRPT_BRIDGE_RANGE_H
