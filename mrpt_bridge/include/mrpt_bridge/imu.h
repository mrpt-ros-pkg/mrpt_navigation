//
// Created by raghavender on 12/08/17.
//

#ifndef MRPT_BRIDGE_IMU_H
#define MRPT_BRIDGE_IMU_H

#include <cstring> // size_t
#include <sensor_msgs/Imu.h>
#include <mrpt/obs/CObservationIMU.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>


using namespace mrpt::obs;


/// ROS message:    http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html
/// MRPT message:   https://github.com/MRPT/mrpt/blob/master/libs/obs/include/mrpt/obs/CObservationIMU.h

namespace mrpt_bridge
{
    namespace imu
    {
        enum TIMUDataIndex
        {
            /** x-axis acceleration (local/vehicle frame) (m/sec<sup>2</sup>) */
                    IMU_X_ACC = 0,
            /** y-axis acceleration (local/vehicle frame) (m/sec<sup>2</sup>) */
                    IMU_Y_ACC,
            /** z-axis acceleration (local/vehicle frame) (m/sec<sup>2</sup>) */
                    IMU_Z_ACC,
            /** yaw angular velocity (local/vehicle frame) (rad/sec) */
                    IMU_YAW_VEL,
            /** pitch angular velocity (local/vehicle frame) (rad/sec) */
                    IMU_PITCH_VEL,
            /** roll angular velocity (local/vehicle frame) (rad/sec) */
                    IMU_ROLL_VEL,
            /** x-axis velocity (global/navigation frame) (m/sec) */
                    IMU_X_VEL,
            /** y-axis velocity (global/navigation  frame) (m/sec) */
                    IMU_Y_VEL,
            /** z-axis velocity (global/navigation  frame) (m/sec) */
                    IMU_Z_VEL,
            /** orientation yaw absolute value (global/navigation frame) (rad) */
                    IMU_YAW,
            /** orientation pitch absolute value (global/navigation frame) (rad) */
                    IMU_PITCH,
            /** orientation roll absolute value (global/navigation frame) (rad) */
                    IMU_ROLL,
            /** x absolute value (global/navigation frame) (meters) */
                    IMU_X,
            /** y absolute value (global/navigation frame) (meters) */
                    IMU_Y,
            /** z absolute value (global/navigation frame) (meters) */
                    IMU_Z,
            /** x magnetic field value (local/vehicle frame) (gauss) */
                    IMU_MAG_X,
            /** y magnetic field value (local/vehicle frame) (gauss) */
                    IMU_MAG_Y,
            /** z magnetic field value (local/vehicle frame) (gauss) */
                    IMU_MAG_Z,
            /** air pressure (Pascals) */
                    IMU_PRESSURE,
            /** altitude from an altimeter (meters) */
                    IMU_ALTITUDE,
            /** temperature (degrees Celsius) */
                    IMU_TEMPERATURE,
            /** Orientation Quaternion X (global/navigation frame) */
                    IMU_ORI_QUAT_X,
            /** Orientation Quaternion Y (global/navigation frame) */
                    IMU_ORI_QUAT_Y,
            /** Orientation Quaternion Z (global/navigation frame) */
                    IMU_ORI_QUAT_Z,
            /** Orientation Quaternion W (global/navigation frame) */
                    IMU_ORI_QUAT_W,
            /** yaw angular velocity (global/navigation frame) (rad/sec) */
                    IMU_YAW_VEL_GLOBAL,
            /** pitch angular velocity (global/navigation frame) (rad/sec) */
                    IMU_PITCH_VEL_GLOBAL,
            /** roll angular velocity (global/navigation frame) (rad/sec) */
                    IMU_ROLL_VEL_GLOBAL,
            /** x-axis acceleration (global/navigation frame) (m/sec<sup>2</sup>) */
                    IMU_X_ACC_GLOBAL,
            /** y-axis acceleration (global/navigation frame) (m/sec<sup>2</sup>) */
                    IMU_Y_ACC_GLOBAL,
            /** z-axis acceleration (global/navigation frame) (m/sec<sup>2</sup>) */
                    IMU_Z_ACC_GLOBAL,

            // Always leave this last value to reflect the number of enum values
                    COUNT_IMU_DATA_FIELDS
        };

        /** Convert sensor_msgs/Imu -> mrpt::obs::CObservationIMU
          * // STILL NEED TO WRITE CODE FOR COVARIANCE
          * \return true on sucessful conversion, false on any error.
          */
        bool ros2mrpt(const sensor_msgs::Imu &msg, CObservationIMU obj);

        /** Convert mrpt::obs::CObservationIMU -> sensor_msgs/Imu
          *  The user must supply the "msg_header" field to be copied into the output message object, since that part does not appear in MRPT classes.
          *
          *  Since COnservationIMU does not contain covariance terms NEED TO fix those.
          * \return true on sucessful conversion, false on any error.
          */
        bool mrpt2ros(const CObservationIMU &obj, const std_msgs::Header &msg_header, sensor_msgs::Imu &msg);

    }
}



#endif //MRPT_BRIDGE_IMU_H



