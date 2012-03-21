/*
 * test_pose_conversions.cpp
 *
 *  Created on: Mar 15, 2012
 *      Author: Pablo Iñigo Blasco
 */

#include <mrpt_bridge/pose_conversions.h>
#include <gtest/gtest.h>
#include <tf/tf.h>

TEST(PoseConversions, checkPoseMatrixFromRotationParameters)
{
  double roll = M_PI, pitch = -M_PI / 2.0, yaw = 0;

  //TF-BULLET ROTATION
  tf::Pose original_pose;
  tf::Quaternion rotation;
  rotation.setRPY(roll, pitch, yaw);
  original_pose.setRotation(rotation);
  btMatrix3x3 basis = original_pose.getBasis();

  //MRPT-ROTATION
  mrpt::poses::CPose3D mrpt_original_pose;
  mrpt_original_pose.setYawPitchRoll(yaw, pitch, roll);
  mrpt::math::CMatrixDouble33 mrpt_basis = mrpt_original_pose.getRotationMatrix();

  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      EXPECT_NEAR(basis[i][j], mrpt_basis(i,j), 0.01);
}

// Declare a test
TEST(PoseConversions, reference_frame_change_with_rotations)
{

  geometry_msgs::PoseWithCovariance ros_msg_original_pose;

  ros_msg_original_pose.pose.position.x = 1;
  ros_msg_original_pose.pose.position.y = 0;
  ros_msg_original_pose.pose.position.z = 0;

  //to mrpt
  mrpt::poses::CPose3DPDFGaussian mrpt_original_pose;
  mrpt_bridge::poses::convertToMRPTCPose3DPDFGaussian(mrpt_original_pose, ros_msg_original_pose);
  EXPECT_EQ(ros_msg_original_pose.pose.position.x, mrpt_original_pose.mean[0]);

  //to tf
  tf::Pose tf_original_pose;
  tf::poseMsgToTF(ros_msg_original_pose.pose, tf_original_pose);

  //rotate yaw pi in MRPT
  mrpt::poses::CPose3D rotation_mrpt;
  double yaw = M_PI / 2.0;
  rotation_mrpt.setFromValues(0, 0, 0, yaw, 0, 0);
  mrpt::poses::CPose3D mrpt_result = rotation_mrpt + mrpt_original_pose.mean;
  EXPECT_NEAR(mrpt_result[1], 1.0, 0.01);

  //rotate yaw pi in TF
  tf::Quaternion rotation_tf;
  rotation_tf.setRPY(0, 0, yaw);
  tf::Pose rotation_pose_tf;
  rotation_pose_tf.setRotation(rotation_tf);
  tf::Pose tf_result = rotation_pose_tf * tf_original_pose;
  EXPECT_NEAR(tf_result.getOrigin()[1], 1.0, 0.01);

  geometry_msgs::Pose mrpt_ros_result;
  mrpt_bridge::poses::convertToROSPose(mrpt_ros_result, mrpt_result);

  EXPECT_NEAR(mrpt_ros_result.position.x, tf_result.getOrigin()[0], 0.01);
  EXPECT_NEAR(mrpt_ros_result.position.y, tf_result.getOrigin()[1], 0.01);
  EXPECT_NEAR(mrpt_ros_result.position.z, tf_result.getOrigin()[2], 0.01);
}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

