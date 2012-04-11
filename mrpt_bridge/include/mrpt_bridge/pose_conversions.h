#ifndef MRPT_POSE_CONVERSIONS_H_
#define MRPT_POSE_CONVERSIONS_H_

#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/math/CQuaternion.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

namespace mrpt_bridge
{
namespace poses
{
	/** Convert: MRPT's CPose3D -> ROS's Pose */
	void mrpt2ros(
		const mrpt::poses::CPose3D& src,
		geometry_msgs::Pose& dst);

	/** Convert: MRPT's CPose2D (x,y,yaw) -> ROS's Pose */
	void mrpt2ros(
		const mrpt::poses::CPose2D& src,
		geometry_msgs::Pose& dst);

	/** Convert: MRPT's CPose3DPDFGaussian -> ROS's PoseWithCovariance */
	void mrpt2ros(
		const mrpt::poses::CPose3DPDFGaussian& src,
		geometry_msgs::PoseWithCovariance& dst);

	/** Convert: MRPT's CPosePDFGaussian (x,y,yaw) -> ROS's PoseWithCovariance */
	void mrpt2ros(
		const mrpt::poses::CPosePDFGaussian& src,
		geometry_msgs::PoseWithCovariance& dst);

	/** Convert: MRPT's CQuaternionDouble -> ROS's Quaternion  */
	void mrpt2ros(
	        const mrpt::poses::CQuaternionDouble& src,
	        geometry_msgs::Quaternion& dst);


	/** Convert: ROS's Pose -> MRPT's CPose2D  */
	void ros2mrpt(const geometry_msgs::Pose& src,
	                 mrpt::poses::CPose2D& dst);

	/** Convert: ROS's Pose -> MRPT's CPose3D  */
	void ros2mrpt(
		const geometry_msgs::Pose& src,
		mrpt::poses::CPose3D& dst);


	/** Convert: ROS's PoseWithCovariance -> MRPT's CPose3DPDFGaussian */
	void ros2mrpt(
		const geometry_msgs::PoseWithCovariance& src,
		mrpt::poses::CPose3DPDFGaussian& dst);

	/** Convert: ROS's PoseWithCovariance -> MRPT's CPosePDFGaussian (x,y,yaw) */
	void ros2mrpt(
		const geometry_msgs::PoseWithCovariance& src,
		mrpt::poses::CPosePDFGaussian& dst);

	/** Convert: ROS's Quaternion -> MRPT's CQuaternionDouble  */
	void ros2mrpt(
	        const geometry_msgs::Quaternion& src,
	        mrpt::poses::CQuaternionDouble& dst);



}
}
#endif /* POSE_CONVERSIONS_H_ */
