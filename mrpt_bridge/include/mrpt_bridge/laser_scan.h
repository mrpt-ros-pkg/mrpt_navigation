#ifndef MRPT_BRIDGE_LASER_SCAN_H
#define MRPT_BRIDGE_LASER_SCAN_H



#include <stdint.h>
#include <string>

namespace std{
    template <class T> class allocator;
}

namespace geometry_msgs{
    template <class ContainerAllocator> struct Pose_;
    typedef Pose_<std::allocator<void> > Pose;
}

namespace sensor_msgs{
    template <class ContainerAllocator> struct LaserScan_;
    typedef LaserScan_<std::allocator<void> > LaserScan;
}

namespace mrpt
{
namespace slam
{
class CObservation2DRangeScan;
}
}

namespace mrpt_bridge {

	/**
	  * \return true on sucessful conversion, false on any error.
	  * \sa mrpt2ros
	  */
    bool convert(const sensor_msgs::LaserScan &_msg, const mrpt::poses::CPose3D &_pose, mrpt::slam::CObservation2DRangeScan &_obj);

    /**
      * \return true on sucessful conversion, false on any error.
      * \sa ros2mrpt
      */
    bool convert(const mrpt::slam::CObservation2DRangeScan &_obj, sensor_msgs::LaserScan &_msg);
    
    /**
      * \return true on sucessful conversion, false on any error.
      * \sa ros2mrpt
      */
    bool convert(const mrpt::slam::CObservation2DRangeScan &_obj, sensor_msgs::LaserScan &_msg, geometry_msgs::Pose &_pose);

} //namespace mrpt_bridge

#endif //MRPT_BRIDGE_LASER_SCAN_H
