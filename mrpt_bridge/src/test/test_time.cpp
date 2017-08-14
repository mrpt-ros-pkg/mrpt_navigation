/*
 * test_time.cpp
 *
 *  Created on: July 15, 2014
 *      Author: Markus Bader
 */

#include <gtest/gtest.h>
#include <mrpt_bridge/time.h>
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#include <boost/date_time/posix_time/time_formatters.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

TEST(Time, basicTest)
{
	mrpt::system::TTimeStamp mtime = mrpt::system::getCurrentLocalTime();
	ros::Time rtimeDes;
	mrpt::system::TTimeStamp mtimeDes;
	mrpt_bridge::convert(mtime, rtimeDes);
	mrpt_bridge::convert(rtimeDes, mtimeDes);
	std::cout << "TimeNow: "
			  << boost::posix_time::to_simple_string(rtimeDes.toBoost())
			  << std::endl;
	EXPECT_EQ(mtime, mtimeDes);
}
