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

#ifndef MRPT_LOCALIZATION_NODE_H
#define MRPT_LOCALIZATION_NODE_H

#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include "mrpt_localization/LocalizationConfig.h"
#include "mrpt_localization/mrpt_localization.h"

/// ROS Node
class PFLocalizationNode : public PFLocalization {
public:
	struct ParametersNode : public Parameters{
        static const int MOTION_MODEL_GAUSSIAN = 0;
        static const int MOTION_MODEL_THRUN = 1;
		ParametersNode();
        ros::NodeHandle node;
        void callbackParameters (mrpt_localization::LocalizationConfig &config, uint32_t level );
        dynamic_reconfigure::Server<mrpt_localization::LocalizationConfig> reconfigureServer_;
        dynamic_reconfigure::Server<mrpt_localization::LocalizationConfig>::CallbackType reconfigureFnc_;        
		void update(const unsigned long &loop_count);
	    double rate;
        int parameter_update_skip;
	};
    
    PFLocalizationNode ( ros::NodeHandle &n );
    ~PFLocalizationNode();
    void init ();
    void loop ();
    void callbackOdometry (const nav_msgs::Odometry&);
    void callbackLaser (const sensor_msgs::LaserScan&);    
private: //functions
    ParametersNode *param();
    void update ();
    void updateLaserPose (std::string frame_id);
    ros::Subscriber subOdometry_;
    ros::Subscriber subLaser0_;
    ros::Subscriber subLaser1_;
    ros::Subscriber subLaser2_;
    tf::TransformListener listenerTF_;
    std::string base_link_;
    std::map<std::string, mrpt::poses::CPose3D> laser_poses_;
private: // variables
    ros::NodeHandle n_;
    unsigned long loop_count_;

};

#endif // MRPT_LOCALIZATION_NODE_H