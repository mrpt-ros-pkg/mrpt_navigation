/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <mrpt/system/filesystem.h>
#include <mrpt/system/string_utils.h>
#include <mrpt_rawlog_record/rawlog_record.h>
#include <mrpt_rawlog_record/rawlog_record_defaults.h>
#include <stdarg.h>
#include <stdio.h>

#include "ros/console.h"

RawlogRecord::~RawlogRecord()
{
	MRPT_TODO("RawlogRecord writes the rawlog only on exit (Ctrl-C)");

	ROS_INFO_STREAM("writing dataset to disk...");
	ROS_INFO_STREAM("pRawLog    entries:" << pRawLog.size());
	ROS_INFO_STREAM("pRawLogASF entries:" << pRawLogASF.size());

	if (pRawLog.size() > 0)
	{
		const std::string filename =
			base_param_.raw_log_folder + "/" + base_param_.raw_log_name;
		ROS_INFO_STREAM("Writing dataset rawlog to: " << filename);

		if (!pRawLog.saveToRawLogFile(filename))
		{
			ROS_ERROR_STREAM("Error writing to " << filename);
		}
	}
	if (pRawLogASF.size() > 0)
	{
		const std::string filename =
			base_param_.raw_log_folder + "/" + base_param_.raw_log_name_asf;

		ROS_INFO_STREAM("Writing dataset rawlog to: " << filename);

		if (!pRawLogASF.saveToRawLogFile(filename))
		{
			ROS_ERROR_STREAM("Error writing to " << filename);
		}
	}
}

void RawlogRecord::updateRawLogName(const mrpt::system::TTimeStamp& t)
{
	const auto prefix = mrpt::system::dateTimeLocalToString(t);

	base_param_.raw_log_name = mrpt::system::fileNameStripInvalidChars(
		prefix + base_param_.raw_log_name);
	base_param_.raw_log_name_asf = mrpt::system::fileNameStripInvalidChars(
		prefix + base_param_.raw_log_name_asf);
}
