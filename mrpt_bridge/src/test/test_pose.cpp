/*
 * test_pose.cpp
 *
 *  Created on: Mar 15, 2012
 *      Author: Pablo Iñigo Blasco
 */

#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDFGaussian.h>
#include <mrpt/math/CQuaternion.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <mrpt_bridge/pose.h>
#include <gtest/gtest.h>

using namespace std;
using namespace mrpt::utils;  // DEG2RAD()

void checkPoseMatrixFromRotationParameters(
	const double roll, const double pitch, const double yaw)
{
	// TF-BULLET ROTATION
	tf::Pose original_pose;
	tf::Quaternion rotation;
	rotation.setRPY(roll, pitch, yaw);
	original_pose.setRotation(rotation);
	tf::Matrix3x3 basis = original_pose.getBasis();

	// MRPT-ROTATION
	mrpt::poses::CPose3D mrpt_original_pose;
	mrpt_original_pose.setYawPitchRoll(yaw, pitch, roll);
	mrpt::math::CMatrixDouble33 mrpt_basis =
		mrpt_original_pose.getRotationMatrix();

	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
			EXPECT_NEAR(basis[i][j], mrpt_basis(i, j), 0.01);
}

TEST(PoseConversions, copyMatrix3x3ToCMatrixDouble33)
{
	tf::Matrix3x3 src(0, 1, 2, 3, 4, 5, 6, 7, 8);
	mrpt::math::CMatrixDouble33 des;
	mrpt_bridge::convert(src, des);
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++) EXPECT_FLOAT_EQ(des(r, c), src[r][c]);
}
TEST(PoseConversions, copyCMatrixDouble33ToMatrix3x3)
{
	mrpt::math::CMatrixDouble33 src;
	src << 0, 1, 2, 3, 4, 5, 6, 7, 8;
	tf::Matrix3x3 des;
	mrpt_bridge::convert(src, des);
	for (int r = 0; r < 3; r++)
		for (int c = 0; c < 3; c++) EXPECT_FLOAT_EQ(des[r][c], src(r, c));
}

TEST(PoseConversions, checkPoseMatrixFromRotationParameters)
{
	checkPoseMatrixFromRotationParameters(0, 0, 0);
	checkPoseMatrixFromRotationParameters(0.2, 0, 0);
	checkPoseMatrixFromRotationParameters(0, 0.2, 0);
	checkPoseMatrixFromRotationParameters(0, 0, 0.2);
	checkPoseMatrixFromRotationParameters(0.4, 0.3, 0.2);
	checkPoseMatrixFromRotationParameters(M_PI, -M_PI / 2.0, 0);
}

// Declare a test
TEST(PoseConversions, reference_frame_change_with_rotations)
{
	geometry_msgs::PoseWithCovariance ros_msg_original_pose;

	ros_msg_original_pose.pose.position.x = 1;
	ros_msg_original_pose.pose.position.y = 0;
	ros_msg_original_pose.pose.position.z = 0;
	ros_msg_original_pose.pose.orientation.x = 0;
	ros_msg_original_pose.pose.orientation.y = 0;
	ros_msg_original_pose.pose.orientation.z = 0;
	ros_msg_original_pose.pose.orientation.w = 1;

	// to mrpt
	mrpt::poses::CPose3DPDFGaussian mrpt_original_pose;
	mrpt_bridge::convert(ros_msg_original_pose, mrpt_original_pose);
	EXPECT_EQ(
		ros_msg_original_pose.pose.position.x, mrpt_original_pose.mean[0]);

	// to tf
	tf::Pose tf_original_pose;
	tf::poseMsgToTF(ros_msg_original_pose.pose, tf_original_pose);

	// rotate yaw pi in MRPT
	mrpt::poses::CPose3D rotation_mrpt;
	double yaw = M_PI / 2.0;
	rotation_mrpt.setFromValues(0, 0, 0, yaw, 0, 0);
	mrpt::poses::CPose3D mrpt_result = rotation_mrpt + mrpt_original_pose.mean;
	EXPECT_NEAR(mrpt_result[1], 1.0, 0.01);

	// rotate yaw pi in TF
	tf::Quaternion rotation_tf;
	rotation_tf.setRPY(0, 0, yaw);
	tf::Pose rotation_pose_tf;
	rotation_pose_tf.setIdentity();
	rotation_pose_tf.setRotation(rotation_tf);
	tf::Pose tf_result = rotation_pose_tf * tf_original_pose;
	EXPECT_NEAR(tf_result.getOrigin()[1], 1.0, 0.01);

	geometry_msgs::Pose mrpt_ros_result;
	mrpt_bridge::convert(mrpt_result, mrpt_ros_result);

	EXPECT_NEAR(mrpt_ros_result.position.x, tf_result.getOrigin()[0], 0.01);
	EXPECT_NEAR(mrpt_ros_result.position.y, tf_result.getOrigin()[1], 0.01);
	EXPECT_NEAR(mrpt_ros_result.position.z, tf_result.getOrigin()[2], 0.01);
}

void check_CPose3D_tofrom_ROS(
	double x, double y, double z, double yaw, double pitch, double roll)
{
	const mrpt::poses::CPose3D p3D(x, y, z, yaw, pitch, roll);

	// Convert MRPT->ROS
	geometry_msgs::Pose ros_p3D;
	mrpt_bridge::convert(p3D, ros_p3D);

	// Compare ROS quat vs. MRPT quat:
	mrpt::math::CQuaternionDouble q;
	p3D.getAsQuaternion(q);

	EXPECT_NEAR(ros_p3D.position.x, p3D.x(), 1e-4) << "p: " << p3D << endl;
	EXPECT_NEAR(ros_p3D.position.y, p3D.y(), 1e-4) << "p: " << p3D << endl;
	EXPECT_NEAR(ros_p3D.position.z, p3D.z(), 1e-4) << "p: " << p3D << endl;

	EXPECT_NEAR(ros_p3D.orientation.x, q.x(), 1e-4) << "p: " << p3D << endl;
	EXPECT_NEAR(ros_p3D.orientation.y, q.y(), 1e-4) << "p: " << p3D << endl;
	EXPECT_NEAR(ros_p3D.orientation.z, q.z(), 1e-4) << "p: " << p3D << endl;
	EXPECT_NEAR(ros_p3D.orientation.w, q.r(), 1e-4) << "p: " << p3D << endl;

	// Test the other path: ROS->MRPT
	mrpt::poses::CPose3D p_bis;
	mrpt_bridge::convert(ros_p3D, p_bis);

	// p_bis==p3D?
	EXPECT_NEAR(
		(p_bis.getAsVectorVal() - p3D.getAsVectorVal())
			.array()
			.abs()
			.maxCoeff(),
		0, 1e-4)
		<< "p_bis: " << p_bis << endl
		<< "p3D: " << p3D << endl;
}

// Declare a test
TEST(PoseConversions, check_CPose3D_tofrom_ROS)
{
	check_CPose3D_tofrom_ROS(0, 0, 0, DEG2RAD(0), DEG2RAD(0), DEG2RAD(0));
	check_CPose3D_tofrom_ROS(1, 2, 3, DEG2RAD(0), DEG2RAD(0), DEG2RAD(0));

	check_CPose3D_tofrom_ROS(1, 2, 3, DEG2RAD(30), DEG2RAD(0), DEG2RAD(0));
	check_CPose3D_tofrom_ROS(1, 2, 3, DEG2RAD(0), DEG2RAD(30), DEG2RAD(0));
	check_CPose3D_tofrom_ROS(1, 2, 3, DEG2RAD(0), DEG2RAD(0), DEG2RAD(30));

	check_CPose3D_tofrom_ROS(1, 2, 3, DEG2RAD(-5), DEG2RAD(15), DEG2RAD(-30));
}

// Declare a test
TEST(PoseConversions, check_CPose2D_to_ROS)
{
	const mrpt::poses::CPose2D p2D(1, 2, 0.56);

	// Convert MRPT->ROS
	geometry_msgs::Pose ros_p2D;
	mrpt_bridge::convert(p2D, ros_p2D);

	// Compare vs. 3D pose:
	const mrpt::poses::CPose3D p3D = mrpt::poses::CPose3D(p2D);
	mrpt::poses::CPose3D p3D_ros;
	mrpt_bridge::convert(ros_p2D, p3D_ros);

	// p3D_ros should equal p3D
	EXPECT_NEAR(
		(p3D_ros.getAsVectorVal() - p3D.getAsVectorVal())
			.array()
			.abs()
			.maxCoeff(),
		0, 1e-4)
		<< "p3D_ros: " << p3D_ros << endl
		<< "p3D: " << p3D << endl;
}
