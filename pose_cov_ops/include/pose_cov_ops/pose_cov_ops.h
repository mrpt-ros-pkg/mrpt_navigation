#ifndef POSE_COV_OPS_H_
#define POSE_COV_OPS_H_

#include <mrpt_bridge/pose_conversions.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovariance.h>

namespace pose_cov_ops
{
	/** out = a (+) b   (Pose composition - without uncertainty) */
	void compose(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b, geometry_msgs::Pose& out);

	/** out = a (+) b   (Pose composition - with uncertainty, as Gaussians with 6x6 cov matrices) */
	void compose(const geometry_msgs::PoseWithCovariance& a, const geometry_msgs::PoseWithCovariance& b, geometry_msgs::PoseWithCovariance& out);

	/** out = a (-) b   (Pose inverse composition - without uncertainty) */
	void inverseCompose(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b, geometry_msgs::Pose& out);

	/** out = a (-) b   (Pose inverse composition - with uncertainty, as Gaussians with 6x6 cov matrices) */
	void inverseCompose(const geometry_msgs::PoseWithCovariance& a, const geometry_msgs::PoseWithCovariance& b, geometry_msgs::PoseWithCovariance& out);

}

#endif /* POSE_COV_OPS_H_ */
