//
// Created by raghavender on 17/08/17.
//

#include "mrpt_bridge/GPS.h"
#include "mrpt_bridge/range.h"
#include "mrpt_bridge/imu.h"
#include "mrpt_bridge/image.h"

#include "../GPS.cpp"
#include "../range.cpp"
#include "../imu.cpp"
#include "../image.cpp"

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
#include <cmath>
#include <dirent.h>

/// opencv includes
#include "opencv2/core.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

/// ros includes
#include "ros/ros.h"
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "testing_MRPT_bridge");
    ros::NodeHandle n;

    ros::Publisher imu_pub          = n.advertise<sensor_msgs::Imu>("imu_publisher", 1000);
    ros::Publisher range_pub        = n.advertise<sensor_msgs::Range>("range_publisher", 1000);
    ros::Publisher navSatFix_pub    = n.advertise<sensor_msgs::NavSatFix>("navSatFix_publisher", 1000);
    ros::Publisher image_pub    = n.advertise<sensor_msgs::Image>("image_publisher", 1000);

    sensor_msgs::Imu        ros_Imu;
    sensor_msgs::Range      ros_Range;
    sensor_msgs::NavSatFix  ros_GPS;
    sensor_msgs::Image      ros_Image;

    CObservationGPS     mrpt_GPS;
    CObservationIMU     mrpt_IMU;
    CObservationRange   mrpt_Range;
    CObservationImage   mrpt_Image;

    int i=0;
    std_msgs::Header header;
    header.frame_id = "map";

    /// Testing Range mrpt --> ros
    mrpt_Range.maxSensorDistance      = 500;
    mrpt_Range.minSensorDistance      = 60;
    mrpt_Range.sensorConeApperture    = (float) M_PI/3;

    /// Testing Imu mrpt--> ros
    mrpt_IMU.rawMeasurements.at(IMU_ORI_QUAT_X)  = 1;
    mrpt_IMU.rawMeasurements.at(IMU_ORI_QUAT_Y)  = 1;
    mrpt_IMU.rawMeasurements.at(IMU_ORI_QUAT_Z)  = 1;
    mrpt_IMU.rawMeasurements.at(IMU_ORI_QUAT_W)  = 1;

    mrpt_IMU.rawMeasurements.at(IMU_X_ACC_GLOBAL) = 1;
    mrpt_IMU.rawMeasurements.at(IMU_Y_ACC_GLOBAL) = 1;
    mrpt_IMU.rawMeasurements.at(IMU_Z_ACC_GLOBAL) = 1;

    mrpt_IMU.rawMeasurements.at(IMU_X_VEL)   = 1;
    mrpt_IMU.rawMeasurements.at(IMU_Y_VEL)   = 1;
    mrpt_IMU.rawMeasurements.at(IMU_Z_VEL)   = 1;

    /// Testing Image mrpt-->ros
    CObservationImage image;
    DIR *dir;
    dirent *pdir;
    vector<string> files;
    vector<string> files_fullpath;

    /// Need to change this path to be the path having the images by the user
    string file_path1 = "/home/raghavender/catkin_ws/src/mrpt_navigation/mrpt_bridge/src/images";

    dir = opendir(file_path1.c_str());
    while(pdir = readdir(dir))
    {
        char *temp_filepath  = pdir->d_name;
        files.push_back(pdir->d_name);
    }
    for(int i=0,j=0 ; i<files.size() ; i++)
    {
        if(files.at(i).size() > 4) // this removes the . and .. in linux as all files will have size more than 4 .png .jpg etc.
        {
            files_fullpath.push_back(file_path1 + "/" + files.at(i));
            //cout << files_fullpath.at(j) << endl;
            j++;
        }
    } // end of for
    sort(files_fullpath.begin(),files_fullpath.end());

    ROS_INFO("Range Message Conversion STARTED..!!");


    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        /// Commpn header for all ROS messages
        header.seq = i;
        header.stamp = ros::Time::now();
        i++;

        /// Publishing sensor_msgs::Image ROS message
        mrpt_Image.image.loadFromFile(files_fullpath.at(i%(files_fullpath.size()-1)));
        mrpt_bridge::image::mrpt2ros(mrpt_Image, header, ros_Image);
        image_pub.publish(ros_Image);


        /// Publishing sensor_msgs::Imu ROS message
        mrpt_IMU.rawMeasurements.at(IMU_ORI_QUAT_X)  = i*0.1;
        mrpt_IMU.rawMeasurements.at(IMU_ORI_QUAT_Y)  = i*0.1;
        mrpt_IMU.rawMeasurements.at(IMU_ORI_QUAT_Z)  = i*0.1;
        mrpt_IMU.rawMeasurements.at(IMU_ORI_QUAT_W)  = i*0.1;
        mrpt_IMU.rawMeasurements.at(IMU_X_ACC_GLOBAL) = i * 0.1;
        mrpt_IMU.rawMeasurements.at(IMU_Y_ACC_GLOBAL) = i*0.1;
        mrpt_IMU.rawMeasurements.at(IMU_Z_ACC_GLOBAL) = 9.8+ i*0.003;
        mrpt_IMU.rawMeasurements.at(IMU_X_VEL)   = i*0.1 * i*0.2;
        mrpt_IMU.rawMeasurements.at(IMU_Y_VEL)   = i*0.1 * i*0.2;
        mrpt_IMU.rawMeasurements.at(IMU_Z_VEL)   = 0;
        mrpt_bridge::imu::mrpt2ros(mrpt_IMU, header, ros_Imu);
        imu_pub.publish(ros_Imu);


        /// Publishing sensor_msgs::NavSatFix ROS message
        mrpt::obs::gnss::Message_NMEA_GGA gga;
        gga.fields.altitude_meters      = 2;
        gga.fields.latitude_degrees     = i*0.001;
        gga.fields.longitude_degrees    = i*0.03 + 43;
        gga.fields.fix_quality          = 1;
        mrpt_GPS.setMsg(gga);
        mrpt_bridge::GPS::mrpt2ros(mrpt_GPS, header, ros_GPS);
        navSatFix_pub.publish(ros_GPS);


        /// Publishing sensor_msgs::Range ROS message
        //mrpt_Range.sensedData.at(0).sensedDistance =  (float) i*0.1;
        ros_Range.range = i*0.1;
        mrpt_bridge::range::mrpt2ros(mrpt_Range, header, ros_Range);
        ROS_INFO(" published %d",i);
        range_pub.publish(ros_Range);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


