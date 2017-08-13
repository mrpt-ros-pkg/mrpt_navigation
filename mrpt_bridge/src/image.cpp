//
// Created by raghavender on 12/08/17.
//

#include "mrpt_bridge/image.h"
#include <mrpt/utils.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include "ros/ros.h"

using namespace ros;
using namespace sensor_msgs;
using namespace cv;
using namespace mrpt::utils;

namespace mrpt_bridge
{
    namespace imu
    {
        bool mrpt2ros(CObservationImage &obj, sensor_msgs::Image &msg)
        {
            CImage temp_img = obj.image;
            Mat cvImg = cv::cvarrToMat(temp_img.getAs<IplImage>());


            //if(obj.image.getChannelCount() ==3 )

            //sensor_msgs::ImagePtr msg11 = cv_bridge::CvImage(std_msgs::Header(),"bgr8", cvImg).toImageMsg();
            //msg = *msg11.get();

            msg.height = (int) obj.image.getHeight();
            msg.width = (int) obj.image.getWidth();

            return true;
        }
    }
}
