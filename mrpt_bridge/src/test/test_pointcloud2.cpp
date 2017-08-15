/*
 * test_pose_conversions.cpp
 *
 *  Created on: Mar 15, 2012
 *      Author: Pablo Iñigo Blasco
 */

#include <mrpt_bridge/point_cloud2.h>

#include <mrpt/version.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <gtest/gtest.h>

TEST(PointCloud2, basicTest)
{
	pcl::PointCloud<pcl::PointXYZ> point_cloud;

	point_cloud.height = 10;
	point_cloud.width = 10;
	point_cloud.is_dense = true;

	int num_points = point_cloud.height * point_cloud.width;
	point_cloud.points.resize(num_points);

	float i_f = 0;
	for (int i = 0; i < num_points; i++)
	{
		pcl::PointXYZ& point = point_cloud.points[i];
		point.x = i_f;
		point.y = -i_f;
		point.z = -2 * i_f;
		i_f += 1.0;
	}

	sensor_msgs::PointCloud2 point_cloud2_msg;
	// pcl_conversions::fromPCL(point_cloud, point_cloud2_msg);
	pcl::toROSMsg(point_cloud, point_cloud2_msg);

	mrpt::maps::CSimplePointsMap mrpt_pc;

	// printf("step 3\n");
	mrpt_bridge::copy(point_cloud2_msg, mrpt_pc);

	i_f = 0;
	for (int i = 0; i < num_points; i++)
	{
		float mrpt_x, mrpt_y, mrpt_z;
		mrpt_pc.getPoint(i, mrpt_x, mrpt_y, mrpt_z);
		EXPECT_FLOAT_EQ(mrpt_x, i_f);
		EXPECT_FLOAT_EQ(mrpt_y, -i_f);
		EXPECT_FLOAT_EQ(mrpt_z, -2 * i_f);

		i_f += 1.0;
	}
	//;
}
