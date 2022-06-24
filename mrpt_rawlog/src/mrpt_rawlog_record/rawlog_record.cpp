/***********************************************************************************
 * Revised BSD License *
 * Copyright (c) 2014, Markus Bader <markus.bader@tuwien.ac.at> *
 * All rights reserved. *
 *                                                                                 *
 * Redistribution and use in source and binary forms, with or without *
 * modification, are permitted provided that the following conditions are met: *
 *     * Redistributions of source code must retain the above copyright *
 *       notice, this list of conditions and the following disclaimer. *
 *     * Redistributions in binary form must reproduce the above copyright *
 *       notice, this list of conditions and the following disclaimer in the *
 *       documentation and/or other materials provided with the distribution. *
 *     * Neither the name of the Vienna University of Technology nor the *
 *       names of its contributors may be used to endorse or promote products *
 *       derived from this software without specific prior written permission. *
 *                                                                                 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *AND *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 **
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE *
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY *
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES *
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 **
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND *
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 **
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **                       *
 ***********************************************************************************/
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
