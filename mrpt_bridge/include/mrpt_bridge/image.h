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
	FILE: image.h
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#ifndef MRPT_BRIDGE_IMAGE_H
#define MRPT_BRIDGE_IMAGE_H

#include <cstring> // size_t
#include <sensor_msgs/Image.h>
#include <mrpt/obs/CObservationImage.h>

using namespace mrpt::obs;


namespace mrpt_bridge
{
    namespace image
    {
        bool ros2mrpt(const sensor_msgs::Image &msg,  CObservationImage &obj);
        bool mrpt2ros(const CObservationImage &obj,const std_msgs::Header &msg_header, sensor_msgs::Image &msg);
    }
}


#endif //MRPT_BRIDGE_IMAGE_H
