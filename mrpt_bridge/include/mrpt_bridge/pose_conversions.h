#ifndef MRPT_POSE_CONVERSIONS_H_
#define MRPT_POSE_CONVERSIONS_H_

#include <mrpt/base.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>

namespace mrpt_bridge
{

namespace poses
{

// TO ROS
void convertToROSPose(geometry_msgs::Pose& dst, const mrpt::poses::CPose3D& src);
void convertToROSPoseWithCovariance(geometry_msgs::PoseWithCovariance& dst, const mrpt::poses::CPose3DPDFGaussian& src);

// TO MRPT
void convertToMRPTCPose3D(mrpt::poses::CPose3D& dst, const geometry_msgs::Pose& src);
void convertToMRPTCPose3DPDFGaussian(mrpt::poses::CPose3DPDFGaussian& dst,
                                     const geometry_msgs::PoseWithCovariance& src);
}
;
}
;

#endif /* POSE_CONVERSIONS_H_ */
