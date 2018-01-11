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
	FILE: image.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include "ros/ros.h"
#include "mrpt_bridge/image.h"
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <mrpt/version.h>
#if MRPT_VERSION>=0x199
using namespace mrpt::img;
#else
using namespace mrpt::utils;
#endif

using namespace ros;
using namespace sensor_msgs;
using namespace cv;
using namespace cv_bridge;


namespace mrpt_bridge
{
    namespace image
    {
        bool ros2mrpt(const sensor_msgs::Image &msg,  CObservationImage &obj)
        {
            CvImage *frame1 = cv_bridge::toCvCopy(msg, "bgr8").get(); //CvShare(msg,"bgr8").image;
            if (!frame1) return false;
            IplImage ipl = frame1->image;
            obj.image.loadFromIplImage(&ipl);

            return true;
        }

        /************************************************************************
        *						mrpt2ros    							        *
        ************************************************************************/
        bool mrpt2ros(const CObservationImage &obj, const std_msgs::Header &msg_header, sensor_msgs::Image &msg)
        {
            CImage temp_img = obj.image;
            Mat cvImg = cv::cvarrToMat(temp_img.getAs<IplImage>());

            cv_bridge::CvImage img_bridge;
            sensor_msgs::Image img_msg;

            img_bridge = CvImage(msg.header, sensor_msgs::image_encodings::BGR8, cvImg);
            img_bridge.toImageMsg(msg);

            msg.encoding = "bgr8";
            msg.header = msg_header;
            msg.height = (int) obj.image.getHeight();
            msg.width = (int) obj.image.getWidth();

            return true;
        }
    }
}

//
/*
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
 */
