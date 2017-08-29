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
	FILE: stereo_image.h
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#ifndef MRPT_BRIDGE_STEREO_IMAGE_H
#define MRPT_BRIDGE_STEREO_IMAGE_H

#include <cstring> // size_t
#include <sensor_msgs/Image.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <stereo_msgs/DisparityImage.h>

using namespace mrpt::obs;


namespace mrpt_bridge
{
    namespace stereo_image
    {
        bool mrpt2ros(const CObservationStereoImages &obj,const std_msgs::Header &msg_header,
                      sensor_msgs::Image &left, sensor_msgs::Image &right,
                      stereo_msgs::DisparityImage &disparity);
    }
}


#endif //MRPT_BRIDGE_STEREO_IMAGE_H
