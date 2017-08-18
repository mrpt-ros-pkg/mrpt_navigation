//
// Created by raghavender on 17/08/17.
//

#include "mrpt_bridge/GPS.h"
#include "mrpt_bridge/range.h"
#include "mrpt_bridge/imu.h"
#include "mrpt_bridge/image.h"

#include "ros/ros.h"

/// mrpt imports
#include <mrpt/obs/CObservation.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs.h>
#include <mrpt/obs/CObservationImage.h>
#include <mrpt/obs/CObservationStereoImages.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/obs/CObservationRange.h>


///c++ standard stuff
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <sstream>
#include <fstream>
#include <iostream>
#include <boost/bind.hpp>

/// opencv includes
#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Imu.h>


using namespace sensor_msgs;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt::utils;
using namespace mrpt;

using namespace std;
using namespace cv;


int rawlog_type = 0;




int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    Imu     message1 ;
    sensor_msgs::Range   message2;

    CObservationGPS     msg1;
    CObservationIMU     msg2;
    CObservationRange   msg3;

    msg3.maxSensorDistance = 212;
    msg3.minSensorDistance = 212;
    msg3.sensorConeApperture =33;

    std_msgs::Header header;
    header.frame_id = 23;

    //mrpt_bridge::range::mrpt2ros(msg3, header, message2);
    //mrpt_bridge::imu::mrpt2ros(msg2, header, message1);

    ROS_INFO("conversion successful");

    cout << message2.field_of_view;



    cout << " Hello" << endl;
    return 0;
}


