//
// Created by raghavender on 12/08/17.
//

#include "ros/ros.h"
#include "mrpt_bridge/image.h"
#include <mrpt/utils.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

using namespace ros;
using namespace sensor_msgs;
using namespace cv;
using namespace mrpt::utils;
using namespace cv_bridge;


namespace mrpt_bridge
{
    namespace imu
    {
        bool ros2mrpt(const sensor_msgs::Image &msg,  CObservationImage &obj)
        {
            CvImage *frame1 = cv_bridge::toCvCopy(msg, "bgr8").get(); //CvShare(msg,"bgr8").image;

        }

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