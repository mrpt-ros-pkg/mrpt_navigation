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
	FILE: test_Bridge.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/
#include "mrpt_bridge/GPS.h"
#include "mrpt_bridge/range.h"
#include "mrpt_bridge/imu.h"
#include "mrpt_bridge/image.h"
#include "mrpt_bridge/stereo_image.h"

#include "../GPS.cpp"
#include "../range.cpp"
#include "../imu.cpp"
#include "../image.cpp"
#include "../stereo_image.cpp"

/// mrpt imports
#include <mrpt/obs/CRawlog.h>
#include <mrpt/system/filesystem.h>

///c++ standard stuff
#include <dirent.h>


using namespace sensor_msgs;
using namespace mrpt::obs;
using namespace mrpt::system;
using namespace mrpt;
using namespace std;
using namespace cv;

/**
 * main method to test the MRPT_ROS bridge to test converisons between MRPT messages and ROS messages
 * the results of the topics published values can be viewed in RViz
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char **argv)
{
    ros::init(argc, argv, "testing_MRPT_bridge");
    ros::NodeHandle n;

    ros::Publisher imu_pub          = n.advertise<sensor_msgs::Imu>("imu_publisher", 1000);
    ros::Publisher navSatFix_pub    = n.advertise<sensor_msgs::NavSatFix>("navSatFix_publisher", 1000);
    ros::Publisher image_pub        = n.advertise<sensor_msgs::Image>("image_publisher", 1000);
    ros::Publisher image_pub_left   = n.advertise<sensor_msgs::Image>("left_image_publisher", 1000);
    ros::Publisher image_pub_right  = n.advertise<sensor_msgs::Image>("right_image_publisher", 1000);

    int num_ranges  = 10;
    ros::Publisher range_pub[num_ranges];
    for(int i=0 ; i<num_ranges ; i++)
    {
        stringstream ss;
        ss << "range_publisher" << i;
        string topic_name = ss.str();
        range_pub[i] = n.advertise<sensor_msgs::Range>(topic_name, 1000);
    }
    sensor_msgs::Imu        ros_Imu;
    sensor_msgs::Range      *ros_Range;
    sensor_msgs::NavSatFix  ros_GPS;
    sensor_msgs::Image      ros_Image;
    sensor_msgs::Image      ros_Image_left;
    sensor_msgs::Image      ros_Image_right;

    CObservationGPS     mrpt_GPS;
    CObservationIMU     mrpt_IMU;
    CObservationRange   mrpt_Range;
    CObservationImage   mrpt_Image;
    CObservationImage   mrpt_Image_left;
    CObservationImage   mrpt_Image_right;

    int i=0;
    std_msgs::Header header;
    header.frame_id = "map";

    /// Testing Range mrpt --> ros
    mrpt_Range.maxSensorDistance      = 500;
    mrpt_Range.minSensorDistance      = 60;
    mrpt_Range.sensorConeApperture    = (float) M_PI/3;
    //initializing the measurement of each range value message in MRPT
    for(int i=0 ; i<num_ranges ; i++)
    {
        CObservationRange::TMeasurement measurement;
        measurement.sensedDistance = i*1.0;
        mrpt_Range.sensedData.push_back(measurement);
    }
    /// initializing num_ranges of the ros Range messages
    ros_Range = new sensor_msgs::Range[num_ranges];


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
    while((pdir = readdir(dir)))
        files.push_back(pdir->d_name);
    for(unsigned int i=0,j=0 ; i<files.size() ; i++)
    {
        if(files.at(i).size() > 4) // this removes the . and .. in linux as all files will have size more than 4 .png .jpg etc.
        {
            files_fullpath.push_back(file_path1 + "/" + files.at(i));
            //cout << files_fullpath.at(j) << endl;
            j++;
        }
    } // end of for
    sort(files_fullpath.begin(),files_fullpath.end());



    ///Testing stereo_image mrpt-->ROS
    /// Testing Image mrpt-->ros
    CObservationImage image1, image2;
    DIR *dir1, *dir2;
    dirent *pdir1, *pdir2;
    vector<string> files1, files2;
    vector<string> files_fullpath1, files_fullpath2;

    /// Need to change this path to be the path having the images by the user
    string file_path_left = "/home/raghavender/catkin_ws/src/mrpt_navigation/mrpt_bridge/src/stereo_images/left";
    string file_path_right = "/home/raghavender/catkin_ws/src/mrpt_navigation/mrpt_bridge/src/stereo_images/right";

    dir1 = opendir(file_path_left.c_str());
    while((pdir1 = readdir(dir1)))
        files1.push_back(pdir1->d_name);
    for(unsigned int i=0,j=0 ; i<files1.size() ; i++)
    {
        if(files1.at(i).size() > 4) // this removes the . and .. in linux as all files will have size more than 4 .png .jpg etc.
        {
            files_fullpath1.push_back(file_path_left + "/" + files1.at(i));
            j++;
        }
    } // end of for
    sort(files_fullpath1.begin(),files_fullpath1.end());

    ///right images
    dir2 = opendir(file_path_right.c_str());
    while((pdir2 = readdir(dir2)))
        files2.push_back(pdir2->d_name);
    for(unsigned int i=0,j=0 ; i<files2.size() ; i++)
    {
        if(files2.at(i).size() > 4) // this removes the . and .. in linux as all files will have size more than 4 .png .jpg etc.
        {
            files_fullpath2.push_back(file_path_right + "/" + files2.at(i));
            j++;
        }
    } // end of for
    sort(files_fullpath2.begin(),files_fullpath2.end());

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

        /// Publishing Stereo Images
        mrpt_Image_left.image.loadFromFile(files_fullpath1.at(i%(files_fullpath1.size()-1)));
        mrpt_bridge::image::mrpt2ros(mrpt_Image_left, header, ros_Image_left);
        image_pub_left.publish(ros_Image_left);

        mrpt_Image_right.image.loadFromFile(files_fullpath2.at(i%(files_fullpath2.size()-1)));
        mrpt_bridge::image::mrpt2ros(mrpt_Image_left, header, ros_Image_right);
        image_pub_right.publish(ros_Image_right);


        /// Publishing sensor_msgs::Imu ROS message
        mrpt_IMU.rawMeasurements.at(IMU_ORI_QUAT_X)  = i*0.1;
        mrpt_IMU.rawMeasurements.at(IMU_ORI_QUAT_Y)  = i*0.1;
        mrpt_IMU.rawMeasurements.at(IMU_ORI_QUAT_Z)  = i*0.1;
        mrpt_IMU.rawMeasurements.at(IMU_ORI_QUAT_W)  = i*-0.1;
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
        mrpt_bridge::range::mrpt2ros(mrpt_Range, header, ros_Range);
        ROS_INFO(" published %d",i);
        // publishing the ranges over num_ranges umber of topics
        for(int i=0 ; i<num_ranges ; i++)
            range_pub[i].publish(ros_Range[i]);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}


