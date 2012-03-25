/*
 * pose_cov_ops.cpp
 *
 *  Created on: Mar 25, 2012
 *      Author: JLBC
 *
 */

#include "pose_cov_ops/pose_cov_ops.h"

#include <mrpt/poses.h>

using namespace mrpt::poses;

void pose_cov_ops::compose(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b, geometry_msgs::Pose& out)
{
	CPose3D A(UNINITIALIZED_POSE),B(UNINITIALIZED_POSE),OUT(UNINITIALIZED_POSE);

	mrpt_bridge::poses::convertToMRPTCPose3D(A,a);
	mrpt_bridge::poses::convertToMRPTCPose3D(B,b);

	OUT.composeFrom(A,B);
	mrpt_bridge::poses::convertToROSPose(out,OUT);
}

void pose_cov_ops::compose(const geometry_msgs::PoseWithCovariance& a, const geometry_msgs::PoseWithCovariance& b, geometry_msgs::PoseWithCovariance& out)
{
	CPose3DPDFGaussian  A(UNINITIALIZED_POSE),B(UNINITIALIZED_POSE);

	mrpt_bridge::poses::convertToMRPTCPose3DPDFGaussian(A,a);
	mrpt_bridge::poses::convertToMRPTCPose3DPDFGaussian(B,b);

	const CPose3DPDFGaussian OUT = A + B;
	mrpt_bridge::poses::convertToROSPoseWithCovariance(out,OUT);
}

void pose_cov_ops::inverseCompose(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b, geometry_msgs::Pose& out)
{
	CPose3D A(UNINITIALIZED_POSE),B(UNINITIALIZED_POSE),OUT(UNINITIALIZED_POSE);

	mrpt_bridge::poses::convertToMRPTCPose3D(A,a);
	mrpt_bridge::poses::convertToMRPTCPose3D(B,b);

	OUT.inverseComposeFrom(A,B);
	mrpt_bridge::poses::convertToROSPose(out,OUT);
}

void pose_cov_ops::inverseCompose(const geometry_msgs::PoseWithCovariance& a, const geometry_msgs::PoseWithCovariance& b, geometry_msgs::PoseWithCovariance& out)
{
	CPose3DPDFGaussian  A(UNINITIALIZED_POSE),B(UNINITIALIZED_POSE);

	mrpt_bridge::poses::convertToMRPTCPose3DPDFGaussian(A,a);
	mrpt_bridge::poses::convertToMRPTCPose3DPDFGaussian(B,b);

	const CPose3DPDFGaussian OUT = A - B;
	mrpt_bridge::poses::convertToROSPoseWithCovariance(out,OUT);
}
