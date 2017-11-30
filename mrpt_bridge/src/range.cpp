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
	FILE: range.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/
#include "mrpt_bridge/range.h"
#include <iostream>
using namespace std;


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
            return true;
        }

        /************************************************************************
        *						mrpt2ros    							        *
        ************************************************************************/
        bool mrpt2ros(const CObservationRange &obj,
                      const std_msgs::Header &msg_header,
                      sensor_msgs::Range *msg)
        {
            long num_range = obj.sensedData.size();

            // 1) sensor_msgs::Range:: header
            for(int i=0 ; i<num_range ; i++)
                msg[i].header = msg_header;

            // 2) sensor_msg::Range parameters
            for(int i=0 ; i<num_range ; i++)
            {
                msg[i].max_range = obj.maxSensorDistance;
                msg[i].min_range = obj.minSensorDistance;
                msg[i].field_of_view = obj.sensorConeApperture;
            }

            /// following part needs to be double checked, it looks incorrect
            /// ROS has single number float for range, MRPT has a list of sensedDistances
            for (int i = 0; i < num_range; i++)
                msg[i].range = obj.sensedData.at(i).sensedDistance;

            /// currently the following are not available in MRPT for corresponding range ROS message
            /// NO corresponding value for MRPT radiation_type at http://mrpt.ual.es/reference/devel/_c_observation_range_8h_source.html
            //msg.radiation_type
            return true;
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
