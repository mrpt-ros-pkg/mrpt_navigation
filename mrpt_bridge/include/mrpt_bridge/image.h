//
// Created by raghavender on 12/08/17.
//

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
