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
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <dynamic_reconfigure/server.h>
#include "mrpt_localization/MotionConfig.h"
#include "mrpt_localization/mrpt_localization.h"
#include "nav_msgs/MapMetaData.h"

/// ROS Node
class PFLocalizationNode : public PFLocalization {
public:
	struct Parameters : public PFLocalization::Parameters{
        static const int MOTION_MODEL_GAUSSIAN = 0;
        static const int MOTION_MODEL_THRUN = 1;
		Parameters();
        ros::NodeHandle node;
        void callbackParameters (mrpt_localization::MotionConfig &config, uint32_t level );
        dynamic_reconfigure::Server<mrpt_localization::MotionConfig> reconfigureServer_;
        dynamic_reconfigure::Server<mrpt_localization::MotionConfig>::CallbackType reconfigureFnc_;
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
    void callbackInitPose (const geometry_msgs::PoseWithCovarianceStamped&);
    void updateMap (const nav_msgs::OccupancyGrid&);
private: //functions
    Parameters *param();
    void update ();
    void updateLaserPose (std::string frame_id);
    ros::Subscriber subOdometry_;
    ros::Subscriber subInitPose_;
    ros::Subscriber subLaser0_;
    ros::Subscriber subLaser1_;
    ros::Subscriber subLaser2_;
    ros::Subscriber subMap_;
    ros::ServiceClient clientMap_;
    ros::Publisher pubParticles_;
    tf::TransformListener listenerTF_;
    std::string base_link_;
    std::map<std::string, mrpt::poses::CPose3D> laser_poses_;
private: // variables
    ros::NodeHandle n_;
    unsigned long loop_count_;
    void publishParticles();
    bool mapCallback(nav_msgs::GetMap::Request  &req, nav_msgs::GetMap::Response &res );
    void publishMap ();
    virtual bool waitForMap();
    nav_msgs::OccupancyGrid rosOccupancyGrid_;
};

#endif // MRPT_LOCALIZATION_NODE_H
