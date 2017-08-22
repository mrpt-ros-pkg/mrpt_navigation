//
// Created by raghavender on 21/08/17.
//

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
