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

  const unsigned int indxs_map[6] = { 0,1,2, 5,4,3 };

  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      dst.covariance[indxs_map[i] * 6 + indxs_map[j]] = src.cov(i, j);
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


/** Convert: MRPT's CPose2D (x,y,yaw) -> ROS's Pose */
void mrpt_bridge::poses::mrpt2ros(
	const mrpt::poses::CPose2D& src,
	geometry_msgs::Pose& dst)
{
  dst.position.x = src.x();
  dst.position.y = src.y();
  dst.position.z = 0;

  const double yaw = src.phi();
  if (std::abs(yaw)<1e-10)
  {
    dst.orientation.x = 0.;
    dst.orientation.y = 0.;
    dst.orientation.z = .5*yaw;
    dst.orientation.w = 1.;
  }
  else
  {
    const double s = ::sin(yaw*.5);
    const double c = ::cos(yaw*.5);
    dst.orientation.x = 0.;
    dst.orientation.y = 0.;
    dst.orientation.z = s;
    dst.orientation.w = c;
  }
}

void mrpt_bridge::poses::mrpt2ros(
	const mrpt::poses::CPosePDFGaussian& src,
	geometry_msgs::PoseWithCovariance& dst
    )
{
  mrpt2ros(src.mean, dst.pose);


  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
    {
      double cov_val;
      int ros_i=i;
      int ros_j=j;
      if (i>2 || j>2)
        cov_val = 0;
      else
      {
        ros_i = i==2 ? 5 : i;
        ros_j = j==2 ? 5 : j;
        cov_val = src.cov(i, j);
      }
      dst.covariance[ros_i * 6 + ros_j] = cov_val;
    }
}


void mrpt_bridge::poses::ros2mrpt(
	const geometry_msgs::PoseWithCovariance& src,
	mrpt::poses::CPose3DPDFGaussian& dst)
{
  ros2mrpt(src.pose, dst.mean);

  const unsigned int indxs_map[6] = { 0,1,2, 5,4,3 };

  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 6; j++)
      dst.cov(i, j) = src.covariance[indxs_map[i] * 6 + indxs_map[j]];
}

void mrpt_bridge::poses::ros2mrpt(
	const geometry_msgs::Pose& src,
	mrpt::poses::CPose3D& dst)
{
	dst.x(src.position.x);
	dst.y(src.position.y);
	dst.z(src.position.z);

	const mrpt::math::CQuaternionDouble q(src.orientation.w, src.orientation.x, src.orientation.y, src.orientation.z);
	q.rotationMatrix( dst.m_ROT );
}
