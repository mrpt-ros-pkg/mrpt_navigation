/**
 * @file   marker_msgs.h
 * @author Markus Bader <markus.bader@tuwien.ac.at>
 * @date   September, 2017
 * @brief  Funtions to convert marker_msgs to mrpt msgs
 **/


#ifndef MRPT_BRIDGE_MARKER_MSGS_H
#define MRPT_BRIDGE_MARKER_MSGS_H

#include <marker_msgs/MarkerDetection.h>
#include <mrpt/obs/CObservationRange.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationBeaconRanges.h>


namespace std
{
template <class T>
class allocator;
}

namespace geometry_msgs
{
template <class ContainerAllocator>
struct Pose_;
typedef Pose_<std::allocator<void>> Pose;
}

namespace marker_msgs
{
template <class ContainerAllocator>
struct MarkerDetection_;
typedef MarkerDetection_<std::allocator<void>> MarkerDetection;
}

namespace mrpt
{
namespace poses
{
class CPose3D;
}
}

#include <mrpt/version.h>
namespace mrpt
{
namespace obs
{
class CObservationBearingRange;
}
}

namespace mrpt_bridge
{
    
    bool convert(const marker_msgs::MarkerDetection& _msg, const mrpt::poses::CPose3D& _pose, mrpt::obs::CObservationBearingRange& _obj);
    bool convert(const marker_msgs::MarkerDetection& _msg, const mrpt::poses::CPose3D& _pose, mrpt::obs::CObservationBeaconRanges& _obj);
}

#endif //MRPT_BRIDGE_MARKER_MSGS_H
