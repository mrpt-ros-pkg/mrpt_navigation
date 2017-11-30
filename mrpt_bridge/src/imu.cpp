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
	FILE: imu.cpp
	AUTHOR: Raghavender Sahdev <raghavendersahdev@gmail.com>
  ---------------------------------------------------------------*/

#include "mrpt_bridge/imu.h"

using namespace std;

namespace mrpt_bridge
{
    /************************************************************************
    *						ros2mrpt    							        *
    ************************************************************************/
    bool imu::ros2mrpt(const sensor_msgs::Imu &msg, CObservationIMU obj)
    {
        obj.rawMeasurements.at(IMU_ORI_QUAT_X)  = msg.orientation.x;
        obj.rawMeasurements.at(IMU_ORI_QUAT_Y)  = msg.orientation.y;
        obj.rawMeasurements.at(IMU_ORI_QUAT_Z)  = msg.orientation.z;
        obj.rawMeasurements.at(IMU_ORI_QUAT_W)  = msg.orientation.w;

        obj.rawMeasurements.at(IMU_X_ACC_GLOBAL) = msg.linear_acceleration.x;
        obj.rawMeasurements.at(IMU_Y_ACC_GLOBAL) = msg.linear_acceleration.y;
        obj.rawMeasurements.at(IMU_Z_ACC_GLOBAL) = msg.linear_acceleration.z;

        obj.rawMeasurements.at(IMU_X_VEL)   = msg.angular_velocity.x;
        obj.rawMeasurements.at(IMU_Y_VEL)   = msg.angular_velocity.y;
        obj.rawMeasurements.at(IMU_Z_VEL)   = msg.angular_velocity.z;

        // NEED TO WRITE CODE FOR COVARIANCE
        return true;
    }

    /************************************************************************
    *						mrpt2ros    							        *
    ************************************************************************/
    bool imu::mrpt2ros(const CObservationIMU &obj, const std_msgs::Header &msg_header, sensor_msgs::Imu &msg)
    {
        msg.header = msg_header;

        vector<double> measurements = obj.rawMeasurements;
        msg.orientation.x   = measurements.at(IMU_ORI_QUAT_X);
        msg.orientation.y   = measurements.at(IMU_ORI_QUAT_Y);
        msg.orientation.z   = measurements.at(IMU_ORI_QUAT_Z);
        msg.orientation.w   = measurements.at(IMU_ORI_QUAT_W);


        /// computing acceleration in global navigation frame not in local vehicle frame, might be the other way round
        msg.linear_acceleration.x   = measurements.at(IMU_X_ACC_GLOBAL);
        msg.linear_acceleration.y   = measurements.at(IMU_Y_ACC_GLOBAL);
        msg.linear_acceleration.z   = measurements.at(IMU_Z_ACC_GLOBAL);

        msg.angular_velocity.x      = measurements.at(IMU_X_VEL);
        msg.angular_velocity.y      = measurements.at(IMU_Y_VEL);
        msg.angular_velocity.z      = measurements.at(IMU_Z_VEL);

        // msg.angular_velocity_covariance
        return true;
    }

}


/*
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
 */
