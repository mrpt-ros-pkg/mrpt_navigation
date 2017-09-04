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
#include <stdio.h>
#include <stdarg.h>

#include <mrpt_rawlog_record/rawlog_record.h>
#include <mrpt_rawlog_record/rawlog_record_defaults.h>

RawlogRecord::~RawlogRecord()
{
	log_info("write data");
	MRPT_TODO("RawlogRecord writes the rawlog only on exit (Ctrl-C)");
	log_info("pRawLog    entries %i", pRawLog->size());
	log_info("pRawLogASF entries %i", pRawLogASF->size());
	if (pRawLog->size() > 0)
	{
		std::string filename =
			param_->raw_log_folder + "/" + param_->raw_log_name;
		log_info("write %s", filename.c_str());
		pRawLog->saveToRawLogFile(filename);
	}
	if (pRawLogASF->size() > 0)
	{
		std::string filename =
			param_->raw_log_folder + "/" + param_->raw_log_name_asf;
		log_info("write %s", filename.c_str());
		pRawLogASF->saveToRawLogFile(filename);
	}
	delete pRawLog;
	delete pRawLogASF;
}

RawlogRecord::RawlogRecord(Parameters* param) : param_(param)
{
	pRawLog = new CRawlog;
	pRawLogASF = new CRawlog;
}
void RawlogRecord::updateRawLogName(const mrpt::system::TTimeStamp& t)
{
	uint64_t tmp = (t - ((uint64_t)116444736 * 1000000000));
	time_t auxTime = tmp / (uint64_t)10000000;
	tm* ptm = localtime(&auxTime);
	param_->raw_log_name = mrpt::format(
		"%u-%02u-%02u--%02u-%02u-%02u--%s", 1900 + ptm->tm_year,
		ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min,
		(unsigned int)ptm->tm_sec, param_->raw_log_name.c_str());
	param_->raw_log_name_asf = mrpt::format(
		"%u-%02u-%02u--%02u-%02u-%02u--%s", 1900 + ptm->tm_year,
		ptm->tm_mon + 1, ptm->tm_mday, ptm->tm_hour, ptm->tm_min,
		(unsigned int)ptm->tm_sec, param_->raw_log_name_asf.c_str());
}


