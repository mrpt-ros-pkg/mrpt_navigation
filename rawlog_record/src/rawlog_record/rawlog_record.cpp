/***********************************************************************************
 * Revised BSD License                                                             *
 * Copyright (c) 2014, Markus Bader <markus.bader@tuwien.ac.at>                    *
 * All rights reserved.                                                            *
 *                                                                                 *
 * Redistribution and use in source and binary forms, with or without              *
 * modification, are permitted provided that the following conditions are met:     *
 *     * Redistributions of source code must retain the above copyright            *
 *       notice, this list of conditions and the following disclaimer.             *
 *     * Redistributions in binary form must reproduce the above copyright         *
 *       notice, this list of conditions and the following disclaimer in the       *
 *       documentation and/or other materials provided with the distribution.      *
 *     * Neither the name of the Vienna University of Technology nor the           *
 *       names of its contributors may be used to endorse or promote products      *
 *       derived from this software without specific prior written permission.     *
 *                                                                                 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND *
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED   *
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE          *
 * DISCLAIMED. IN NO EVENT SHALL Markus Bader BE LIABLE FOR ANY                    *
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES      *
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;    *
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND     *
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT      *
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS   *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                    *                       *
 ***********************************************************************************/


#include <rawlog_record/rawlog_record.h>
#include <rawlog_record/rawlog_record_defaults.h>

#include <mrpt/base.h>
#include <mrpt/slam.h>

RawlogRecord::~RawlogRecord()
{
    if(pRawLog->size() > 0)  pRawLog->saveToRawLogFile(param_->raw_log_name);
    if(pRawLogASF->size() > 0)  pRawLogASF->saveToRawLogFile(param_->raw_log_name_asf);
    delete pRawLog;
    delete pRawLogASF;
}

RawlogRecord::RawlogRecord(Parameters *param)
    :param_(param) {
    pRawLog = new mrpt::slam::CRawlog;
    pRawLogASF = new mrpt::slam::CRawlog;
}
void RawlogRecord::updateRawLogName(const mrpt::system::TTimeStamp &t) {
    uint64_t        tmp = (t - ((uint64_t)116444736*1000000000));
    time_t          auxTime = tmp / (uint64_t)10000000;
    unsigned int    secFractions = (unsigned int)( 1000000 * (tmp % 10000000) / 10000000.0 );
    tm  *ptm = localtime( &auxTime );
    param_->raw_log_name = mrpt::format(
                               "%u-%02u-%02u--%02u-%02u-%02u--%s",
                               1900+ptm->tm_year,
                               ptm->tm_mon+1,
                               ptm->tm_mday,
                               ptm->tm_hour,
                               ptm->tm_min,
                               (unsigned int)ptm->tm_sec,
                               param_->raw_log_name.c_str());
    param_->raw_log_name_asf = mrpt::format(
                                   "%u-%02u-%02u--%02u-%02u-%02u--%s",
                                   1900+ptm->tm_year,
                                   ptm->tm_mon+1,
                                   ptm->tm_mday,
                                   ptm->tm_hour,
                                   ptm->tm_min,
                                   (unsigned int)ptm->tm_sec,
                                   param_->raw_log_name_asf.c_str());
}


void RawlogRecord::incommingLaserData(mrpt::slam::CObservation2DRangeScanPtr laser) {
    mrpt::slam::CSensoryFramePtr sf = mrpt::slam::CSensoryFrame::Create();
    mrpt::slam::CObservationPtr obs = mrpt::slam::CObservationPtr(laser);
    sf->insert(obs);
    pRawLog->addObservationMemoryReference(laser);
    pRawLogASF->addObservationsMemoryReference(sf);
}

void RawlogRecord::incommingOdomData(mrpt::slam::CObservationOdometryPtr odometry) {

    mrpt::slam::CActionRobotMovement2D odom_move;
    odom_move.timestamp = odometry->timestamp;
    mrpt::poses::CPose2D incOdoPose;
    if(odomLastPose_.empty()) {
        incOdoPose = mrpt::poses::CPose2D(0, 0, 0);
    } else {
        incOdoPose = odometry->odometry - odomLastPose_;
    }
    odom_move.computeFromOdometry(incOdoPose, param_->motionModelOptions);
    mrpt::slam::CActionCollectionPtr action = mrpt::slam::CActionCollection::Create();
    action->insert(odom_move);
    odomLastPose_ = odometry->odometry;
    
    pRawLog->addObservationMemoryReference(odometry);
    pRawLogASF->addActionsMemoryReference(action);
}
