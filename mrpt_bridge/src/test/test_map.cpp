/*
 * test_map.cpp
 *
 *  Created on: July 21, 2014
 *      Author: Markus Bader
 */

#include <gtest/gtest.h>
#include <mrpt_bridge/map.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/console.h>

#include <mrpt/version.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
using mrpt::maps::COccupancyGridMap2D;

void getEmptyRosMsg(nav_msgs::OccupancyGrid& msg)
{
	msg.info.width = 300;
	msg.info.height = 500;
	msg.info.resolution = 0.1;
	msg.info.origin.position.x = rand() % 10 - 5;
	msg.info.origin.position.y = rand() % 10 - 5;
	msg.info.origin.position.z = 0;

	msg.info.origin.orientation.x = 0;
	msg.info.origin.orientation.y = 0;
	msg.info.origin.orientation.z = 0;
	msg.info.origin.orientation.w = 1;

	msg.data.resize(msg.info.width * msg.info.height, -1);
}

TEST(Map, basicTestHeader)
{
	nav_msgs::OccupancyGrid srcRos;
	COccupancyGridMap2D desMrpt;

	getEmptyRosMsg(srcRos);

	srcRos.info.origin.orientation.x = 1;  // roated maps are not supported
	EXPECT_FALSE(mrpt_bridge::convert(srcRos, desMrpt));
	srcRos.info.origin.orientation.x = 0;  // fix rotation
	EXPECT_TRUE(mrpt_bridge::convert(srcRos, desMrpt));

	EXPECT_EQ(srcRos.info.width, desMrpt.getSizeX());
	EXPECT_EQ(srcRos.info.height, desMrpt.getSizeY());
	EXPECT_EQ(srcRos.info.resolution, desMrpt.getResolution());
	for (int h = 0; h < srcRos.info.width; h++)
	{
		for (int w = 0; w < srcRos.info.width; w++)
		{
			EXPECT_EQ(
				desMrpt.getPos(w, h), 0.5);  // all -1 entreis should map to 0.5
		}
	}
}

TEST(Map, check_ros2mrpt_and_back)
{
	nav_msgs::OccupancyGrid srcRos;
	COccupancyGridMap2D desMrpt;
	nav_msgs::OccupancyGrid desRos;

	getEmptyRosMsg(srcRos);

	ASSERT_TRUE(mrpt_bridge::convert(srcRos, desMrpt));
	ASSERT_TRUE(mrpt_bridge::convert(desMrpt, desRos, desRos.header));
	for (int h = 0; h < srcRos.info.width; h++)
	{
		for (int w = 0; w < srcRos.info.width; w++)
		{
			EXPECT_EQ(
				desRos.data[h * srcRos.info.width + h],
				50);  // all -1 entreis should map to 50
		}
	}

	for (int i = 0; i <= 100; i++)
	{
		srcRos.data[i] = i;
	}
	EXPECT_TRUE(mrpt_bridge::convert(srcRos, desMrpt));
	EXPECT_TRUE(mrpt_bridge::convert(desMrpt, desRos, desRos.header));
	for (int i = 0; i <= 100; i++)
	{
		// printf("%4i, %4.3f = %4.3f,%4i\n", srcRos.data[i],
		// 1-((float)i)/100.0, desMrpt.getCell(i,0), desRos.data[i]);
		EXPECT_NEAR(1 - ((float)i) / 100.0, desMrpt.getCell(i, 0), 0.03)
			<< "ros to mprt";
		EXPECT_NEAR(srcRos.data[i], desRos.data[i], 1) << "ros to mprt to ros";
	}
}
