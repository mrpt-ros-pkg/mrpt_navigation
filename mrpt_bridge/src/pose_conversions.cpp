/*
 * pose_conversions.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: Pablo Iñigo Blasco
 *
 *      To understand better how this is implemented see the references:
 *      - http://www.mrpt.org/2D_3D_Geometry
 *
 */

#include "mrpt_bridge/pose_conversions.h"

void mrpt_bridge::poses::mrpt2ros(
	const mrpt::poses::CPose3DPDFGaussian& src,
	geometry_msgs::PoseWithCovariance& dst
    )
{
  mrpt2ros(src.mean, dst.pose);

  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
    {
      int ros_i = i;
      int ros_j = j;

      if (ros_i == 5)
        ros_i = 3;
      else if (ros_i == 3)
        ros_i = 5;

      if (ros_j == 5)
        ros_j = 3;
      else if (ros_j == 3)
        ros_j = 5;

      dst.covariance[ros_i * 6 + ros_j] = src.cov(i, j);
    }

}

void mrpt_bridge::poses::mrpt2ros(
	const mrpt::poses::CPose3D& src,
	geometry_msgs::Pose& dst )
{

  dst.position.x = src[0];
  dst.position.y = src[1];
  dst.position.z = src[2];

  mrpt::math::CQuaternionDouble q;
  src.getAsQuaternion(q);

  dst.orientation.x = q.x();
  dst.orientation.y = q.y();
  dst.orientation.z = q.z();
  dst.orientation.w = q.r();

}

void mrpt_bridge::poses::ros2mrpt(
	const geometry_msgs::PoseWithCovariance& src,
	mrpt::poses::CPose3DPDFGaussian& dst)
{
  ros2mrpt(src.pose, dst.mean);

  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
    {
      int ros_i = i;
      int ros_j = j;

      if (ros_i == 5)
        ros_i = 3;
      else if (ros_i == 3)
        ros_i = 5;

      if (ros_j == 5)
        ros_j = 3;
      else if (ros_j == 3)
        ros_j = 5;

      dst.cov(i, j) = src.covariance[ros_i * 6 + ros_j];
    }
}

void mrpt_bridge::poses::ros2mrpt(
	const geometry_msgs::Pose& src,
	mrpt::poses::CPose3D& dst)
{
  dst.setFromValues(src.position.x, src.position.y, src.position.z, src.orientation.z, src.orientation.y,
                    src.orientation.x);
}
