/*
 * pose.cpp
 *
 *  Created on: Mar 14, 2012
 *      Author: Pablo Iñigo Blasco
 *
 *      To understand better how this is implemented see the references:
 *      - http://www.mrpt.org/2D_3D_Geometry
 *
 */

/**
\mainpage

\htmlinclude manifest.html

\b mrpt_bridge is a set of functions to convert between common ROS messages and
MRPT C++ classes.

**/

#include <mrpt/poses/CPose2D.h>
#include <mrpt/poses/CPose3D.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/poses/CPosePDFGaussianInf.h>
#include <mrpt/poses/CPose3DPDFGaussianInf.h>
#include <mrpt/math/CQuaternion.h>
#include <mrpt/math/lightweight_geom_data.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <mrpt/math/CMatrixFixedNumeric.h>
#include <tf/tf.h>

#include <mrpt/version.h>
#if MRPT_VERSION<0x199
#include <mrpt/utils/mrpt_macros.h>
#else
#include <mrpt/core/exceptions.h>
#endif

#include "mrpt_bridge/pose.h"

mrpt::math::CMatrixDouble33& mrpt_bridge::convert(
	const tf::Matrix3x3& _src, mrpt::math::CMatrixDouble33& _des)
{
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++) _des(r, c) = _src[r][c];
	return _des;
}

tf::Matrix3x3& mrpt_bridge::convert(
	const mrpt::math::CMatrixDouble33& _src, tf::Matrix3x3& _des)
{
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++) _des[r][c] = _src(r, c);
	return _des;
}

tf::Transform& mrpt_bridge::convert(
	const mrpt::poses::CPose3DPDFGaussian& _src, tf::Transform& _des)
{
	return convert(_src.mean, _des);
}

geometry_msgs::PoseWithCovariance& mrpt_bridge::convert(
	const mrpt::poses::CPose3DPDFGaussian& _src,
	geometry_msgs::PoseWithCovariance& _des)
{
	convert(_src.mean, _des.pose);

	// Read REP103: http://ros.org/reps/rep-0103.html#covariance-representation
	// # Row-major representation of the 6x6 covariance matrix
	// # The orientation parameters use a fixed-axis representation.
	// # In order, the parameters are:
	// # (x, y, z, rotation about X axis, rotation about Y axis, rotation about
	// Z axis)
	// float64[36] covariance
	// Old comment: "MRPT uses non-fixed axis for 6x6 covariance: should use a
	// transform Jacobian here!"
	//           JL ==> Nope! non-fixed z-y-x equals fixed x-y-z rotations.

	const unsigned int indxs_map[6] = {0, 1, 2,
									   5, 4, 3};  // X,Y,Z,YAW,PITCH,ROLL

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			_des.covariance[indxs_map[i] * 6 + indxs_map[j]] = _src.cov(i, j);
		}
	}
	return _des;
}

geometry_msgs::PoseWithCovariance& mrpt_bridge::convert(
	const mrpt::poses::CPose3DPDFGaussianInf& _src,
	geometry_msgs::PoseWithCovariance& _des)
{
	mrpt::poses::CPose3DPDFGaussian mrpt_gaussian;
	mrpt_gaussian.copyFrom(_src);
	convert(mrpt_gaussian, _des);

	return _des;
}

tf::Transform& mrpt_bridge::convert(
	const mrpt::poses::CPose3D& _src, tf::Transform& _des)
{
	tf::Vector3 origin(_src[0], _src[1], _src[2]);
	mrpt::math::CMatrixDouble33 R;
	_src.getRotationMatrix(R);
	tf::Matrix3x3 basis;
	_des.setBasis(convert(R, basis));
	_des.setOrigin(origin);

	return _des;
}
mrpt::poses::CPose3D& mrpt_bridge::convert(
	const tf::Transform& _src, mrpt::poses::CPose3D& _des)
{
	const tf::Vector3& t = _src.getOrigin();
	_des.x() = t[0], _des.y() = t[1], _des.z() = t[2];
	const tf::Matrix3x3& basis = _src.getBasis();
	mrpt::math::CMatrixDouble33 R;
	convert(basis, R);
	_des.setRotationMatrix(R);

	return _des;
}

geometry_msgs::Pose& mrpt_bridge::convert(
	const mrpt::poses::CPose3D& _src, geometry_msgs::Pose& _des)
{
	_des.position.x = _src[0];
	_des.position.y = _src[1];
	_des.position.z = _src[2];

	mrpt::math::CQuaternionDouble q;
	_src.getAsQuaternion(q);

	_des.orientation.x = q.x();
	_des.orientation.y = q.y();
	_des.orientation.z = q.z();
	_des.orientation.w = q.r();

	return _des;
}

/** Convert: MRPT's CPose2D (x,y,yaw) -> ROS's Pose */
geometry_msgs::Pose& mrpt_bridge::convert(
	const mrpt::poses::CPose2D& _src, geometry_msgs::Pose& _des)
{
	_des.position.x = _src.x();
	_des.position.y = _src.y();
	_des.position.z = 0;

	const double yaw = _src.phi();
	if (std::abs(yaw) < 1e-10)
	{
		_des.orientation.x = 0.;
		_des.orientation.y = 0.;
		_des.orientation.z = .5 * yaw;
		_des.orientation.w = 1.;
	}
	else
	{
		const double s = ::sin(yaw * .5);
		const double c = ::cos(yaw * .5);
		_des.orientation.x = 0.;
		_des.orientation.y = 0.;
		_des.orientation.z = s;
		_des.orientation.w = c;
	}

	return _des;
}

mrpt::poses::CPose2D& mrpt_bridge::convert(
	const geometry_msgs::Pose& _src, mrpt::poses::CPose2D& _des)
{
	_des.x(_src.position.x);
	_des.y(_src.position.y);

	mrpt::math::CQuaternionDouble quat;
	convert(_src.orientation, quat);

	double roll, pitch, yaw;
	quat.rpy(roll, pitch, yaw);

	_des.phi(yaw);

	return _des;
}

geometry_msgs::PoseWithCovariance& mrpt_bridge::convert(
	const mrpt::poses::CPosePDFGaussian& _src,
	geometry_msgs::PoseWithCovariance& _des)
{
	convert(_src.mean, _des.pose);

	// Read REP103: http://ros.org/reps/rep-0103.html#covariance-representation
	// Old comment: "MRPT uses non-fixed axis for 6x6 covariance: should use a
	// transform Jacobian here!"
	//           JL ==> Nope! non-fixed z-y-x equals fixed x-y-z rotations.

	// geometry_msgs/PoseWithCovariance msg stores the covariance matrix in
	// row-major representation
	// Indexes are :
	// [ 0   1   2   3   4   5  ]
	// [ 6   7   8   9   10  11 ]
	// [ 12  13  14  15  16  17 ]
	// [ 18  19  20  21  22  23 ]
	// [ 24  25  26  27  28  29 ]
	// [ 30  31  32  33  34  35 ]

	_des.covariance[0] = _src.cov(0, 0);
	_des.covariance[1] = _src.cov(0, 1);
	_des.covariance[5] = _src.cov(0, 2);
	_des.covariance[6] = _src.cov(1, 0);
	_des.covariance[7] = _src.cov(1, 1);
	_des.covariance[11] = _src.cov(1, 2);
	_des.covariance[30] = _src.cov(2, 0);
	_des.covariance[31] = _src.cov(2, 1);
	_des.covariance[35] = _src.cov(2, 2);

	return _des;
}

geometry_msgs::PoseWithCovariance& mrpt_bridge::convert(
	const mrpt::poses::CPosePDFGaussianInf& _src,
	geometry_msgs::PoseWithCovariance& _des)
{
	mrpt::poses::CPosePDFGaussian mrpt_gaussian;
	mrpt_gaussian.copyFrom(_src);

	convert(mrpt_gaussian, _des);
	return _des;
}

mrpt::poses::CPose3DPDFGaussian& mrpt_bridge::convert(
	const geometry_msgs::PoseWithCovariance& _src,
	mrpt::poses::CPose3DPDFGaussian& _des)
{
	convert(_src.pose, _des.mean);

	const unsigned int indxs_map[6] = {0, 1, 2, 5, 4, 3};

	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			_des.cov(i, j) = _src.covariance[indxs_map[i] * 6 + indxs_map[j]];
		}
	}

	return _des;
}

mrpt::poses::CPose3DPDFGaussianInf& mrpt_bridge::convert(
	const geometry_msgs::PoseWithCovariance& _src,
	mrpt::poses::CPose3DPDFGaussianInf& _des)
{
	mrpt::poses::CPose3DPDFGaussian mrpt_gaussian;
	convert(
		_src, mrpt_gaussian);  // Intermediate transform => CPose3DPDFGaussian
	_des.copyFrom(mrpt_gaussian);

	return _des;
}

mrpt::poses::CPosePDFGaussian& mrpt_bridge::convert(
	const geometry_msgs::PoseWithCovariance& _src,
	mrpt::poses::CPosePDFGaussian& _des)
{
	convert(_src.pose, _des.mean);

	_des.cov(0, 0) = _src.covariance[0];
	_des.cov(0, 1) = _src.covariance[1];
	_des.cov(0, 2) = _src.covariance[5];
	_des.cov(1, 0) = _src.covariance[0 + 6];
	_des.cov(1, 1) = _src.covariance[1 + 6];
	_des.cov(1, 2) = _src.covariance[5 + 6];
	_des.cov(2, 0) = _src.covariance[0 + 30];
	_des.cov(2, 1) = _src.covariance[1 + 30];
	_des.cov(2, 2) = _src.covariance[5 + 30];

	return _des;
}

mrpt::poses::CPosePDFGaussianInf& mrpt_bridge::convert(
	const geometry_msgs::PoseWithCovariance& _src,
	mrpt::poses::CPosePDFGaussianInf& _des)
{
	mrpt::poses::CPosePDFGaussian mrpt_gaussian;
	convert(_src, mrpt_gaussian);  // intermediate transform: PoseWithCovariance
	// => CPosePDFGaussian
	_des.copyFrom(mrpt_gaussian);

	return _des;
}

mrpt::poses::CQuaternionDouble& mrpt_bridge::convert(
	const geometry_msgs::Quaternion& _src, mrpt::poses::CQuaternionDouble& _des)
{
	_des.x(_src.x);
	_des.y(_src.y);
	_des.z(_src.z);
	_des.r(_src.w);
	return _des;
}

geometry_msgs::Quaternion& mrpt_bridge::convert(
	const mrpt::poses::CQuaternionDouble& _src, geometry_msgs::Quaternion& _des)
{
	_des.x = _src.x();
	_des.y = _src.y();
	_des.z = _src.z();
	_des.w = _src.r();

	return _des;
}

mrpt::poses::CPose3D& mrpt_bridge::convert(
	const geometry_msgs::Pose& _src, mrpt::poses::CPose3D& _des)
{
	const mrpt::math::CQuaternionDouble q(
		_src.orientation.w, _src.orientation.x, _src.orientation.y,
		_src.orientation.z);
	_des = mrpt::poses::CPose3D(
		q, _src.position.x, _src.position.y, _src.position.z);

	return _des;
}

tf::Transform& mrpt_bridge::convert(
	const mrpt::math::TPose3D& _src, tf::Transform& _des)
{
	return mrpt_bridge::convert(mrpt::poses::CPose3D(_src), _des);
}

tf::Transform& mrpt_bridge::convert(
	const mrpt::poses::CPose2D& _src, tf::Transform& _des)
{
	return mrpt_bridge::convert(mrpt::poses::CPose3D(_src), _des);
}
tf::Transform& mrpt_bridge::convert(
	const mrpt::math::TPose2D& _src, tf::Transform& _des)
{
	return mrpt_bridge::convert(
		mrpt::poses::CPose3D(mrpt::math::TPose3D(_src)), _des);
}
