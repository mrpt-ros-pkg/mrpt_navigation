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

#include <iostream>
#include <stdint.h>
#include <boost/interprocess/sync/interprocess_mutex.hpp>

#ifndef MRPT_RAWLOG_RECORD_H
#define MRPT_RAWLOG_RECORD_H

#include <mrpt/version.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationBeaconRanges.h>

using mrpt::obs::CActionRobotMovement2D;
using mrpt::obs::CRawlog;

#include <mrpt/poses/CPose2D.h>
#include <mrpt_bridge/mrpt_log_macros.h>

#include <mrpt_rawlog_record/rawlog_record_defaults.h>

class RawlogRecord
{
	MRPT_VIRTUAL_LOG_MACROS;

   public:
	struct Parameters
	{
		Parameters() = default;

		bool debug{RAWLOG_RECORD_DEFAULT_DEBUG};
		std::string raw_log_folder{RAWLOG_RECORD_DEFAULT_RAW_FOLDER};
		std::string raw_log_name{RAWLOG_RECORD_DEFAULT_RAW_LOG_NAME};
		std::string raw_log_name_asf{RAWLOG_RECORD_DEFAULT_RAW_LOG_NAME_ASF};
		bool record_range_scan{true};
		bool record_bearing_range{false};
		bool record_beacon_range{false};
		double bearing_range_std_range{0.1};
		double bearing_range_std_yaw{0.01};
		double bearing_range_std_pitch{0.01};
		CActionRobotMovement2D::TMotionModelOptions motionModelOptions;
	};
	RawlogRecord() = default;
	~RawlogRecord();

   protected:
	Parameters base_param_;

	CRawlog pRawLog;
	CRawlog pRawLogASF;
	void updateRawLogName(const mrpt::system::TTimeStamp& t);
	boost::interprocess::interprocess_mutex mutexRawLog;
};

#endif  // MRPT_RAWLOG_RECORD_H
