/***********************************************************************************
 * Revised BSD License *
 * Copyright (c) 2014-2023, Jose-Luis Blanco <jlblanco@ual.es> *
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
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. *
 ***********************************************************************************/

/* mrpt deps*/
#include <mrpt/config/CConfigFile.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/maps/CSimplePointsMap.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CPointCloud.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/ros2bridge/laser_scan.h>
#include <mrpt/ros2bridge/point_cloud.h>
#include <mrpt/ros2bridge/pose.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/system/string_utils.h>
#include <mp2p_icp_filters/FilterDecimateVoxels.h>

/* ros2 deps */
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <mutex>

using namespace mrpt::system;
using namespace mrpt::config;
using namespace mrpt::img;
using namespace mrpt::maps;
using namespace mrpt::obs;

class LocalObstaclesNode: public rclcpp::Node
{
    public:
    /* Ctor*/
    explicit LocalObstaclesNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    /* Dtor*/
    ~LocalObstaclesNode(){}

    private:
    /* Read parameters from the node handle*/
    void read_parameters();

    /* Callback: On recalc local map & publish it*/
    void on_do_publish();

    /* Callback: On new sensor data*/
    void on_new_sensor_laser_2d(const sensor_msgs::msg::LaserScan::SharedPtr& scan);

    /* Callback: On new pointcloud data*/
    void on_new_sensor_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr& pts);

    /**
	 * @brief Subscribe to a variable number of topics.
	 * @param lstTopics String with list of topics separated with ","
	 * @param subs[in,out] List of subscribers will be here at return.
	 * @return The number of topics subscribed to.
	 */
	template <typename MessageT, typename CallbackMethodType>
	size_t subscribe_to_multiple_topics(
		const std::string& lstTopics,
        std::vector<typename rclcpp::Subscription<MessageT>::SharedPtr>& subscriptions,
        CallbackMethodType callback);

    // member variables
    CTimeLogger m_profiler;
    bool m_show_gui = false;
    std::string m_frameid_reference = "ddom"; //!< type:"odom" 
    std::string m_frameid_robot = "base_link"; //!< type: "base_link"
    std::string m_topic_local_map_pointcloud = "local_map_pointcloud"; 
    //!< Default: "local_map_pointcloud"
    std::string m_topics_source_2dscan = "scan, laser1"; //!< Default: "scan, laser1"
    std::string m_topics_source_pointclouds = "";

    double m_time_window = 0.20; //!< In secs (default: 0.2). Can't be smaller than m_publish_period
    double m_publish_period = 0.05; //!< In secs (default: 0.05). Can't be larger than m_time_window

    rclcpp::TimerBase::SharedPtr m_timer_publish;

    // Sensor data:
	struct TInfoPerTimeStep
	{
		CObservation::Ptr observation;
		mrpt::poses::CPose3D robot_pose;
	};
	typedef std::multimap<double, TInfoPerTimeStep> TListObservations;

	TListObservations m_hist_obs;  //!< The history of past observations during
	                               //! the interest time window.
                                    
	std::mutex m_hist_obs_mtx;   //!< mutex

    CSimplePointsMap::Ptr m_localmap_pts = CSimplePointsMap::Create();

	mrpt::gui::CDisplayWindow3D::Ptr m_gui_win;

    /// Used for example to run voxel grid decimation, etc.
	/// Refer to mp2p_icp docs
	mp2p_icp_filters::FilterPipeline m_filter_pipeline;

	std::string m_filter_output_layer_name;	 //!< mp2p_icp output layer name

    /**
     * @name ROS2 pubs/subs
     * @{
     */
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pub_local_map_pointcloud;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> m_subs_2dlaser;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> m_subs_pointclouds;

    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;

};