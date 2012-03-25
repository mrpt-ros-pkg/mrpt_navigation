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
	void convertToROSPose(
		geometry_msgs::Pose& dst,
		const mrpt::poses::CPose3D& src);

	/** Convert: MRPT's CPose3DPDFGaussian -> ROS's PoseWithCovariance */
	void convertToROSPoseWithCovariance(
		geometry_msgs::PoseWithCovariance& dst,
		const mrpt::poses::CPose3DPDFGaussian& src);


	/** Convert: ROS's Pose -> MRPT's CPose  */
	void convertToMRPTCPose3D(
		mrpt::poses::CPose3D& dst,
		const geometry_msgs::Pose& src);

	/** Convert: ROS's PoseWithCovariance -> MRPT's CPose3DPDFGaussian */
	void convertToMRPTCPose3DPDFGaussian(
		mrpt::poses::CPose3DPDFGaussian& dst,
		const geometry_msgs::PoseWithCovariance& src);

}
}
#endif /* POSE_CONVERSIONS_H_ */
