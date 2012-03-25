#ifndef MRPT_POSE_CONVERSIONS_H_
#define MRPT_POSE_CONVERSIONS_H_

#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

namespace mrpt_bridge
{
namespace poses
{
	/** Convert: MRPT's CPose -> ROS's Pose */
	void mrpt2ros(
		const mrpt::poses::CPose3D& src,
		geometry_msgs::Pose& dst);

	/** Convert: MRPT's CPose3DPDFGaussian -> ROS's PoseWithCovariance */
	void mrpt2ros(
		const mrpt::poses::CPose3DPDFGaussian& src,
		geometry_msgs::PoseWithCovariance& dst);


	/** Convert: ROS's Pose -> MRPT's CPose  */
	void ros2mrpt(
		const geometry_msgs::Pose& src,
		mrpt::poses::CPose3D& dst);

	/** Convert: ROS's PoseWithCovariance -> MRPT's CPose3DPDFGaussian */
	void ros2mrpt(
		const geometry_msgs::PoseWithCovariance& src,
		mrpt::poses::CPose3DPDFGaussian& dst);

}
}
#endif /* POSE_CONVERSIONS_H_ */