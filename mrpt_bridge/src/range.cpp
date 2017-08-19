//
// Created by raghavender on 12/08/17.
//

#include "mrpt_bridge/range.h"

namespace mrpt_bridge
{
    namespace range
    {
        /************************************************************************
        *						ros2mrpt    							        *
        ************************************************************************/
        bool ros2mrpt(const sensor_msgs::Range &msg,
                      CObservationRange &obj) {
            obj.minSensorDistance = msg.min_range;
            obj.maxSensorDistance = msg.max_range;
            obj.sensorConeApperture = msg.field_of_view;

            /// again this is amibiguous as can't be certain of number of measurement from corresponding ROS message
            obj.sensedData.at(0).sensedDistance = msg.range;
        }

        /************************************************************************
        *						mrpt2ros    							        *
        ************************************************************************/
        bool mrpt2ros(const CObservationRange &obj,
                      const std_msgs::Header &msg_header,
                      sensor_msgs::Range &msg) {
            // 1) sensor_msgs::Range:: header
            msg.header = msg_header;

            // 2) sensor_msg::Range parameters
            msg.max_range       = obj.maxSensorDistance;
            msg.min_range       = obj.minSensorDistance;
            msg.field_of_view   = obj.sensorConeApperture;

            /// following part needs to be double checked, it looks incorrect
            /// ROS has single number float for range, MRPT has a list of sensedDistances
            int i;
            for (i = 0; i < obj.sensedData.size(); i++) {
                msg.range += obj.sensedData.at(i).sensedDistance;
            }
            msg.range = msg.range / i;

            /// currently the following are not available in MRPT for corresponding range ROS message
            /// NO corresponding value for MRPT radiation_type at http://mrpt.ual.es/reference/devel/_c_observation_range_8h_source.html
            //msg.radiation_type
        }
    }
}

/// Range ROS message
/*
uint8 ULTRASOUND=0
uint8 INFRARED=1
std_msgs/Header header
uint8 radiation_type
float32 field_of_view
float32 min_range
float32 max_range
float32 range
*/