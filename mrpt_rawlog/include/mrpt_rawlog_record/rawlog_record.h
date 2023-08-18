/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2023, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/poses/CPose2D.h>
#include <mrpt/ros1bridge/logging.h>
#include <mrpt_rawlog_record/rawlog_record_defaults.h>
#include <stdint.h>

#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <iostream>

using mrpt::obs::CActionRobotMovement2D;
using mrpt::obs::CRawlog;

class RawlogRecord
{
   public:
	RawlogRecord() = default;
	~RawlogRecord();

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

   protected:
	Parameters base_param_;

	CRawlog pRawLog;
	CRawlog pRawLogASF;
	void updateRawLogName(const mrpt::system::TTimeStamp& t);
	boost::interprocess::interprocess_mutex mutexRawLog;
};
