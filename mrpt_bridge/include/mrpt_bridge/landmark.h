/*
 * File: landmark.h
 * Author: Vladislav Tananaev
 *
 */

#ifndef MRPT_BRIDGE_LANDMARK_H
#define MRPT_BRIDGE_LANDMARK_H



#include <stdint.h>
#include <string>

namespace std{
    template <class T> class allocator;
}

namespace geometry_msgs{
    template <class ContainerAllocator> struct Pose_;
    typedef Pose_<std::allocator<void> > Pose;
}

namespace mrpt_msgs{
    template <class ContainerAllocator> struct ObservationRangeBearing_;
    typedef ObservationRangeBearing_<std::allocator<void> > ObservationRangeBearing;
}

namespace mrpt { namespace poses { class CPose3D; } }
#include <mrpt/version.h>
#if MRPT_VERSION>=0x130
	namespace mrpt { namespace obs { class CObservationBearingRange; } }
#else
	namespace mrpt { namespace slam  { class CObservationBearingRange; } }
#endif

namespace mrpt_bridge {

	/** @name LaserScan: ROS <-> MRPT
	 *  @{ */

	/** ROS->MRPT: Takes a mrpt_msgs::CObservationBearingRange and the relative pose of the laser wrt base_link and builds a ObservationRangeBearing
	  * \return true on sucessful conversion, false on any error.
	  * \sa mrpt2ros
	  */
	bool convert(
			const mrpt_msgs::ObservationRangeBearing &_msg,
			const mrpt::poses::CPose3D &_pose,
#if MRPT_VERSION>=0x130
			mrpt::obs::CObservationBearingRange &_obj
#else
			mrpt::slam::CObservationBearingRange &_obj
#endif
			);

	/** MRPT->ROS: Takes a CObservationBearingRange and outputs range data in mrpt_msgs::ObservationRangeBearing
      * \return true on sucessful conversion, false on any error.
      * \sa ros2mrpt
      */
	bool convert(
#if MRPT_VERSION>=0x130
		const mrpt::obs::CObservationBearingRange &_obj,
#else
		const mrpt::slam::CObservationBearingRange &_obj,
#endif
		mrpt_msgs::ObservationRangeBearing &_msg);

	/** MRPT->ROS: Takes a CObservationBearingRange and outputs range data in mrpt_msgs::ObservationRangeBearing + the relative pose of the range sensor wrt base_link
	  * \return true on sucessful conversion, false on any error.
      * \sa ros2mrpt
      */
	bool convert(
		#if MRPT_VERSION>=0x130
				const mrpt::obs::CObservationBearingRange &_obj,
		#else
				const mrpt::slam::CObservationBearingRange &_obj,
		#endif
		mrpt_msgs::ObservationRangeBearing &_msg,
		geometry_msgs::Pose &_pose);

	/** @} */

} //namespace mrpt_bridge

#endif //MRPT_BRIDGE_LANDMARK_H
