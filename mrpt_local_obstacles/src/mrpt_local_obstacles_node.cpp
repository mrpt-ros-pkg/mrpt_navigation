/***********************************************************************************
 * Revised BSD License                                                             *
 * Copyright (c) 2014-2015, Jose-Luis Blanco <jlblanco@ual.es>                     *
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
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                    *
 ***********************************************************************************/

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/Odometry.h>
//#include <geometry_msgs/Twist.h>
//#include <rosgraph_msgs/Clock.h>

#include <tf/transform_listener.h>

#include <map>
#include <mrpt/slam/CSensoryFrame.h>
#include <mrpt/system/string_utils.h>
//#include <mrpt/system/threads.h>


// The ROS node
class LocalObstaclesNode
{
private:
	//nav_msgs::Odometry *groundTruthMsgs;
	//rosgraph_msgs::Clock clockMsg;

	struct TAuxInitializer {
		TAuxInitializer(int argc, char **argv)
		{
			ros::init(argc, argv, "mrpt_local_obstacles_node");
		}
	};
	TAuxInitializer m_auxinit; //!< Just to make sure ROS is init first
	ros::NodeHandle m_nh; //!< The node handle
	ros::NodeHandle m_localn; //!< "~"
	bool            m_show_gui;
	std::string     m_frameid_reference; //!< typ: "odom"
	std::string     m_frameid_robot;     //!< typ: "base_link"
	std::string     m_topic_local_map_pointcloud;  //!< Default: "local_map_pointcloud"
	std::string     m_source_topics_2dscan;     //!< Default: "scan,laser1"
	double          m_time_window;  //!< In secs (default: 0.2)

	ros::Publisher  m_pub_local_map_pointcloud;

	std::vector<ros::Subscriber>  m_subs_2dlaser; //!< Subscriber to 2D laser scans

	tf::TransformListener    m_tf_listener; //!< Use to retrieve TF data

	std::multimap<double,mrpt::slam::CSensoryFrame>  m_hist_obs;  //!< The history of past observations during the interest time window.


	/**
	 * @brief Subscribe to a variable number of topics.
	 * @param lstTopics String with list of topics separated with ","
	 * @param subs[in,out] List of subscribers will be here at return.
	 * @return The number of topics subscribed to.
	 */
	template <typename CALLBACK_METHOD_TYPE>
	size_t subscribeToMultipleTopics(const std::string &lstTopics, std::vector<ros::Subscriber> &subs, CALLBACK_METHOD_TYPE cb)
	{
		std::vector<std::string> lstSources;
		mrpt::utils::tokenize(lstTopics," ,\t\n",lstSources);
		subs.resize(lstSources.size());
		for (size_t i=0;i<lstSources.size();i++)
			subs[i]  = m_nh.subscribe(lstSources[i],  1, cb, this);
	}


	/** Callback: On new sensor data
	  */
	void onNewSensor_Laser2D(const sensor_msgs::LaserScanConstPtr & scan)
	{
		ROS_INFO("[onNewSensor_Laser2D] New scan received.");


	} // end onNewSensor_Laser2D



public:
	/**  Constructor: Inits ROS system */
	LocalObstaclesNode(int argc, char **argv) :
		m_auxinit   (argc,argv),
		m_nh        (),
		m_localn    ("~"),
		m_show_gui  (true),
		m_frameid_reference("odom"),
		m_frameid_robot("base_link"),
		m_topic_local_map_pointcloud("local_map_pointcloud"),
		m_source_topics_2dscan("scan,laser1"),
		m_time_window (0.2)
	{
		// Load params:
		m_localn.param("show_gui", m_show_gui, m_show_gui);
		m_localn.param("frameid_reference", m_frameid_reference, m_frameid_reference);
		m_localn.param("frameid_robot", m_frameid_robot, m_frameid_robot);
		m_localn.param("topic_local_map_pointcloud", m_topic_local_map_pointcloud, m_topic_local_map_pointcloud);
		m_localn.param("source_topics_2dscan",m_source_topics_2dscan,m_source_topics_2dscan);
		m_localn.param("time_window",m_time_window,m_time_window);

		// Init ROS publishers:
		m_pub_local_map_pointcloud = m_nh.advertise<sensor_msgs::PointCloud>(m_topic_local_map_pointcloud,10);

		// Init ROS subs:
		// Subscribe to one or more laser sources:
		size_t nSubsTotal = 0;
		nSubsTotal+= this->subscribeToMultipleTopics(m_source_topics_2dscan,m_subs_2dlaser, &LocalObstaclesNode::onNewSensor_Laser2D);

		ROS_INFO("Total number of sensor subscriptions: %u\n",static_cast<unsigned int>(nSubsTotal));
		ROS_ASSERT_MSG(nSubsTotal>0,"*Error* It is mandatory to set at least one source topic for sensory information!");

		// Init timers:


	} // end ctor




}; // end class



int main(int argc, char **argv)
{
	LocalObstaclesNode  the_node(argc, argv);
	ros::spin();
	return 0;
}

