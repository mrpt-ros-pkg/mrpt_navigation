/**
 * @file   marker_msgs.cpp
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @date   September, 2017
 * @brief  Funtions to convert marker_msgs to mrpt msgs
 **/


#include "mrpt_bridge/marker_msgs.h"
#include "mrpt_bridge/time.h"
#include <marker_msgs/MarkerDetection.h>
#include <mrpt/obs/CObservationBearingRange.h>



#include <iostream>
using namespace std;


namespace mrpt_bridge
{
bool convert(const marker_msgs::MarkerDetection& src, const mrpt::poses::CPose3D& pose, mrpt::obs::CObservationBearingRange& des) {

    convert(src.header.stamp, des.timestamp);
    des.setSensorPose(pose);
    des.minSensorDistance = src.distance_min;
    des.maxSensorDistance = src.distance_max;

    des.sensedData.resize(src.markers.size());
    for(size_t i = 0; i < src.markers.size(); i++) {
        const marker_msgs::Marker &marker = src.markers[i];
        mrpt::obs::CObservationBearingRange::TMeasurement &measurment = des.sensedData[i];
        measurment.range = sqrt(marker.pose.position.x * marker.pose.position.x + marker.pose.position.y * marker.pose.position.y);
        measurment.yaw = atan2(marker.pose.position.y, marker.pose.position.x);
        measurment.pitch = 0.0;
        if(marker.ids.size() > 0) {
            measurment.landmarkID = marker.ids[0];
        } else {
            measurment.landmarkID = -1;
        }
    }
	return true;
};
bool convert(const marker_msgs::MarkerDetection& src, const mrpt::poses::CPose3D& pose, mrpt::obs::CObservationBeaconRanges& des) {

    convert(src.header.stamp, des.timestamp);
    des.setSensorPose(pose);
    des.minSensorDistance = src.distance_min;
    des.maxSensorDistance = src.distance_max;

    des.sensedData.resize(src.markers.size());
    for(size_t i = 0; i < src.markers.size(); i++) {
        const marker_msgs::Marker &marker = src.markers[i];
        mrpt::obs::CObservationBeaconRanges::TMeasurement &measurment = des.sensedData[i];
        measurment.sensedDistance = sqrt(marker.pose.position.x * marker.pose.position.x + marker.pose.position.y * marker.pose.position.y);
        measurment.sensorLocationOnRobot.m_coords = pose.m_coords;
        if(marker.ids.size() > 0) {
            measurment.beaconID = marker.ids[0];
        } else {
            measurment.beaconID = -1;
        }
    }
	return true;
};
}
