/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2024, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <mrpt/ros2bridge/time.h>
#include <mrpt_pointcloud_pipeline/mrpt_pointcloud_pipeline_node.h>

#include <sstream>

// for now, not needed (node=executable)
// #include "rclcpp_components/register_node_macro.hpp"

using namespace mrpt::system;
using namespace mrpt::config;
using namespace mrpt::img;
using namespace mrpt::maps;
using namespace mrpt::obs;

LocalObstaclesNode::LocalObstaclesNode(const rclcpp::NodeOptions& options)
	: Node("mrpt_pointcloud_pipeline", options)
{
	m_profiler.setName(Node::get_name());

	read_parameters();

	// Init ROS subs:
	// Subscribe to one or more laser sources:
	size_t nSubsTotal = 0;
	nSubsTotal += subscribe_to_multiple_topics<sensor_msgs::msg::LaserScan>(
		m_topics_source_2dscan, m_subs_2dlaser,
		[this](
			const sensor_msgs::msg::LaserScan::SharedPtr scan,
			const std::string& topicName) {
			this->on_new_sensor_laser_2d(scan, topicName);
		});

	nSubsTotal += subscribe_to_multiple_topics<sensor_msgs::msg::PointCloud2>(
		m_topics_source_pointclouds, m_subs_pointclouds,
		[this](
			const sensor_msgs::msg::PointCloud2::SharedPtr pts,
			const std::string& topicName) {
			this->on_new_sensor_pointcloud(pts, topicName);
		});

	RCLCPP_INFO(
		get_logger(), "Total number of sensor subscriptions: %u",
		static_cast<unsigned int>(nSubsTotal));

	if (!(nSubsTotal > 0))
	{
		RCLCPP_ERROR(
			get_logger(),
			"*Error* It is mandatory to set at least one source topic for "
			"sensory information!");

		rclcpp::shutdown();
	}

	// Create the tf2 buffer and listener
	m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
	m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

	m_timer_publish = create_wall_timer(
		std::chrono::duration<double>(m_publish_period),
		[this]() { this->on_do_publish(); });
}  // end ctor

/** Callback: On recalc local map & publish it */
void LocalObstaclesNode::on_do_publish()
{
	CTimeLoggerEntry tle(m_profiler, "on_do_publish");

	// Purge old observations & latch a local copy:
	obs_list_t obs;
	{
		CTimeLoggerEntry tle(m_profiler, "onDoPublish.removingOld");
		m_hist_obs_mtx.lock();

		// Purge old obs:
		if (!m_hist_obs.empty())
		{
			const double last_time = m_hist_obs.rbegin()->first;
			obs_list_t::iterator it_first_valid =
				m_hist_obs.lower_bound(last_time - m_time_window);
			const size_t nToRemove =
				std::distance(m_hist_obs.begin(), it_first_valid);
			RCLCPP_DEBUG(
				get_logger(),
				"[onDoPublish] Removing %u old entries, last_time=%lf",
				static_cast<unsigned int>(nToRemove), last_time);
			m_hist_obs.erase(m_hist_obs.begin(), it_first_valid);
		}
		// Local copy in this thread:
		obs = m_hist_obs;
		m_hist_obs_mtx.unlock();
	}

	// Keep only one obs per topic?
	if (m_one_observation_per_topic)
	{
		// TODO(jlbc): Remove in reverse order to keep the latest one!
		std::set<std::string> foundTopics;
		for (auto it = obs.begin(); it != obs.end();)
		{
			const auto& topic = it->second.sourceTopic;
			if (foundTopics.count(topic) != 0)
			{
				// duplicated entry, delete:
				it = obs.erase(it);
			}
			else
			{
				foundTopics.insert(topic);
				++it;  // move on:
			}
		}
	}

	RCLCPP_DEBUG(
		get_logger(), "Building local map with %u observations.",
		static_cast<unsigned int>(obs.size()));

	if (obs.empty()) return;

	// Build local map(s):
	// -----------------------------------------------
	mp2p_icp::metric_map_t mm;

	mrpt::poses::CPose3D curRobotPose;
	{
		CTimeLoggerEntry tle2(m_profiler, "on_do_publish.buildLocalMap");

		// Get the latest robot pose in the reference frame (typ: /odom ->
		// /base_link)
		// so we can build the local map RELATIVE to it:

		try
		{
			geometry_msgs::msg::TransformStamped tx;
			tx = m_tf_buffer->lookupTransform(
				m_frameid_reference, m_frameid_robot, tf2::TimePointZero);

			tf2::Transform tfx;
			tf2::fromMsg(tx.transform, tfx);
			curRobotPose = mrpt::ros2bridge::fromROS(tfx);
		}
		catch (const tf2::ExtrapolationException& ex)
		{
			RCLCPP_ERROR(get_logger(), "%s", ex.what());
			return;
		}

		RCLCPP_DEBUG(
			get_logger(),
			"[onDoPublish] Building local map relative to latest robot "
			"pose: %s",
			curRobotPose.asString().c_str());

		// For each observation: compute relative robot pose & insert obs
		// into map:
		for (const auto& [timestamp, ipt] : obs)
		{
			// Relative pose in the past:
			const mrpt::poses::CPose3D relPose = ipt.robot_pose - curRobotPose;

			// Insert obs:
			CTimeLoggerEntry tleObsFilter(
				m_profiler, "on_do_publish.apply_per_obs_pipeline");

			// Apply optional generators for auxiliary map layers, etc:
			mp2p_icp_filters::apply_generators(
				m_generator, *ipt.observation, mm);

			// per-observation filtering:
			mp2p_icp_filters::apply_filter_pipeline(m_per_obs_pipeline, mm);

			tleObsFilter.stop();
		}
	}

	// Apply final filtering:
	CTimeLoggerEntry tleFilter(
		m_profiler, "on_do_publish.apply_final_pipeline");

	mp2p_icp_filters::apply_filter_pipeline(m_final_pipeline, mm);

	tleFilter.stop();

	// Publish them:
	for (auto& e : layer2topic_)
	{
		if (e.pub->get_subscription_count() == 0) continue;

		sensor_msgs::msg::PointCloud2 msg_pts;
		msg_pts.header.frame_id = m_frameid_robot;

		// Publish using the timestamp of the *latest* observation:
		msg_pts.header.stamp = mrpt::ros2bridge::toROS(
			mrpt::Clock::fromDouble(obs.rbegin()->first));

		const auto& outPtsMap = mm.point_layer(e.layer);
		ASSERT_(outPtsMap);

		// TODO(jlbc): We could add an if-else chain if XYZI / XYZIRT point
		// clouds are desired as output too, but it seems not likely for this
		// kind of filtering node.
		auto simplPts =
			std::dynamic_pointer_cast<mrpt::maps::CSimplePointsMap>(outPtsMap);
		ASSERT_(simplPts);

		mrpt::ros2bridge::toROS(*simplPts, msg_pts.header, msg_pts);
		e.pub->publish(msg_pts);
	}

	// Show gui:
	if (m_show_gui)
	{
		if (!m_gui_win)
		{
			m_gui_win = mrpt::gui::CDisplayWindow3D::Create(
				"LocalObstaclesNode", 800, 600);
			mrpt::opengl::COpenGLScene::Ptr& scene =
				m_gui_win->get3DSceneAndLock();
			scene->insert(mrpt::opengl::CGridPlaneXY::Create());
			scene->insert(
				mrpt::opengl::stock_objects::CornerXYZSimple(1.0, 4.0));

			auto gl_obs = mrpt::opengl::CSetOfObjects::Create();
			gl_obs->setName("obstacles");
			scene->insert(gl_obs);

			auto gl_rawpts = mrpt::opengl::CPointCloud::Create();
			gl_rawpts->setName("raw_points");
			gl_rawpts->setPointSize(1.0);
			gl_rawpts->setColor_u8(TColor(0x00ff00));
			scene->insert(gl_rawpts);

			auto gl_pts = mrpt::opengl::CPointCloud::Create();
			gl_pts->setName("final_points");
			gl_pts->setPointSize(4.0);
			gl_pts->setColor_u8(TColor(0x0000ff));
			scene->insert(gl_pts);

			scene->getViewport()->addTextMessage(
				5, 5, "Press keys '1'/'2' to switch raw/output clouds", 0);

			m_gui_win->unlockAccess3DScene();
		}

		// Process key ops on the gui:
		if (m_gui_win->keyHit())
		{
			const auto key = m_gui_win->getPushedKey();
			m_gui_win->clearKeyHitFlag();
			switch (key)
			{
				case '1':
					m_visible_raw = !m_visible_raw;
					break;

				case '2':
					m_visible_output = !m_visible_output;
					break;

				default:
					break;
			};
		}

		auto& scene = m_gui_win->get3DSceneAndLock();

		auto gl_obs = mrpt::ptr_cast<mrpt::opengl::CSetOfObjects>::from(
			scene->getByName("obstacles"));
		ASSERT_(!!gl_obs);
		gl_obs->clear();

		auto glRawPts = mrpt::ptr_cast<mrpt::opengl::CPointCloud>::from(
			scene->getByName("raw_points"));

		auto glFinalPts = mrpt::ptr_cast<mrpt::opengl::CPointCloud>::from(
			scene->getByName("final_points"));

		scene->getViewport()->addTextMessage(
			5, 25,
			mrpt::format(
				"Raw points: %10s | Output points: %10s",
				m_visible_raw ? "VISIBLE" : "hidden",
				m_visible_output ? "VISIBLE" : "hidden"),
			1);

		for (const auto& o : obs)
		{
			const InfoPerTimeStep& ipt = o.second;
			// Relative pose in the past:
			mrpt::poses::CPose3D relPose(mrpt::poses::UNINITIALIZED_POSE);
			relPose.inverseComposeFrom(ipt.robot_pose, curRobotPose);

			mrpt::opengl::CSetOfObjects::Ptr gl_axis =
				mrpt::opengl::stock_objects::CornerXYZSimple(0.9, 2.0);
			gl_axis->setPose(relPose);
			gl_obs->insert(gl_axis);
		}  // end for

		if (auto pts = mm.point_layer(mp2p_icp::metric_map_t::PT_LAYER_RAW);
			pts)
		{
			glRawPts->loadFromPointsMap(pts.get());
			glRawPts->setVisibility(m_visible_raw);
		}

		// This shows the first output layer only (!). TODO: show all of them?
		if (!layer2topic_.empty())
		{
			if (auto pts = mm.point_layer(layer2topic_.front().layer); pts)
			{
				glFinalPts->loadFromPointsMap(pts.get());
				glFinalPts->setVisibility(m_visible_output);
			}
		}

		m_gui_win->unlockAccess3DScene();
		m_gui_win->repaint();
	}

}  // onDoPublish

void LocalObstaclesNode::on_new_sensor_laser_2d(
	const sensor_msgs::msg::LaserScan::SharedPtr& scan,
	const std::string& topicName)
{
	CTimeLoggerEntry tle(m_profiler, "on_new_sensor_laser_2d");

	geometry_msgs::msg::TransformStamped sensorOnRobot;
	try
	{
		CTimeLoggerEntry tle2(
			m_profiler, "onNewSensor_Laser2D.lookupTransform_sensor");
		sensorOnRobot = m_tf_buffer->lookupTransform(
			m_frameid_robot, scan->header.frame_id, tf2::TimePointZero);
	}
	catch (const tf2::TransformException& ex)
	{
		RCLCPP_ERROR(get_logger(), "%s", ex.what());
		return;
	}

	const mrpt::poses::CPose3D sensorOnRobot_mrpt = [&]() {
		tf2::Transform tx;
		tf2::fromMsg(sensorOnRobot.transform, tx);
		return mrpt::ros2bridge::fromROS(tx);
	}();

	// In MRPT, CObservation2DRangeScan holds both: sensor data +
	// relative pose:
	auto obsScan = CObservation2DRangeScan::Create();
	mrpt::ros2bridge::fromROS(*scan, sensorOnRobot_mrpt, *obsScan);

	RCLCPP_DEBUG(
		get_logger(), "[onNewSensor_Laser2D] %u rays, sensor pose on robot %s",
		static_cast<unsigned int>(obsScan->getScanSize()),
		sensorOnRobot_mrpt.asString().c_str());

	// Get sensor timestamp:
	auto stamp = scan->header.stamp;
	const double timestamp =
		stamp.sec + static_cast<double>(stamp.nanosec) / 1e9;

	// Get robot pose at that time in the reference frame, typ: /odom ->
	// /base_link
	mrpt::poses::CPose3D robotPose;
	try
	{
		CTimeLoggerEntry tle3(
			m_profiler, "onNewSensor_Laser2D.lookupTransform_robot");

		geometry_msgs::msg::TransformStamped robotTfStamp;
		try
		{
			robotTfStamp = m_tf_buffer->lookupTransform(
				m_frameid_reference, m_frameid_robot, tf2::TimePointZero);
		}
		catch (const tf2::ExtrapolationException& ex)
		{
			RCLCPP_ERROR(get_logger(), "%s", ex.what());
			return;
		}

		robotPose = [&]() {
			tf2::Transform tx;
			tf2::fromMsg(robotTfStamp.transform, tx);
			return mrpt::ros2bridge::fromROS(tx);
		}();

		RCLCPP_DEBUG(
			get_logger(), "[onNewSensor_Laser2D] robot pose %s",
			robotPose.asString().c_str());
	}
	catch (const tf2::TransformException& ex)
	{
		RCLCPP_ERROR(get_logger(), "%s", ex.what());
		return;
	}

	// Insert into the observation history:
	InfoPerTimeStep ipt;
	ipt.sourceTopic = topicName;
	ipt.observation = obsScan;
	ipt.robot_pose = robotPose;

	m_hist_obs_mtx.lock();
	m_hist_obs.insert(m_hist_obs.end(), obs_list_t::value_type(timestamp, ipt));
	m_hist_obs_mtx.unlock();

}  // end on_new_sensor_laser_2d

void LocalObstaclesNode::on_new_sensor_pointcloud(
	const sensor_msgs::msg::PointCloud2::SharedPtr& pts,
	const std::string& topicName)
{
	CTimeLoggerEntry tle(m_profiler, "on_new_sensor_pointcloud");

	// Get the relative position of the sensor wrt the robot:
	geometry_msgs::msg::TransformStamped sensorOnRobot;
	try
	{
		CTimeLoggerEntry tle2(
			m_profiler, "on_new_sensor_pointcloud.lookupTransform_sensor");

		sensorOnRobot = m_tf_buffer->lookupTransform(
			m_frameid_robot, pts->header.frame_id, tf2::TimePointZero);
	}
	catch (const tf2::TransformException& ex)
	{
		RCLCPP_ERROR(get_logger(), "%s", ex.what());
		return;
	}

	// Convert data to MRPT format:
	const mrpt::poses::CPose3D sensorOnRobot_mrpt = [&]() {
		tf2::Transform tx;
		tf2::fromMsg(sensorOnRobot.transform, tx);
		return mrpt::ros2bridge::fromROS(tx);
	}();

	// In MRPT, CObservationPointCloud holds both: sensor data +
	// relative pose:
	auto obsPts = CObservationPointCloud::Create();
	const auto ptsMap = mrpt::maps::CSimplePointsMap::Create();
	obsPts->pointcloud = ptsMap;
	obsPts->sensorPose = sensorOnRobot_mrpt;
	mrpt::ros2bridge::fromROS(*pts, *ptsMap);

	RCLCPP_DEBUG(
		get_logger(),
		"[on_new_sensor_pointcloud] %u points, sensor pose on robot %s",
		static_cast<unsigned int>(ptsMap->size()),
		sensorOnRobot_mrpt.asString().c_str());

	// Get sensor timestamp:
	auto stamp = pts->header.stamp;
	const double timestamp =
		stamp.sec + static_cast<double>(stamp.nanosec) / 1e9;

	// Get robot pose at that time in the reference frame, typ: /odom ->
	// /base_link
	mrpt::poses::CPose3D robotPose;
	try
	{
		CTimeLoggerEntry tle3(
			m_profiler, "onNewSensor_pointcloud.lookupTransform_robot");

		geometry_msgs::msg::TransformStamped robotTfStamp;
		try
		{
			robotTfStamp = m_tf_buffer->lookupTransform(
				m_frameid_reference, m_frameid_robot, tf2::TimePointZero);
		}
		catch (const tf2::ExtrapolationException& ex)
		{
			RCLCPP_ERROR(get_logger(), "%s", ex.what());
			return;
		}

		robotPose = [&]() {
			tf2::Transform tx;
			tf2::fromMsg(robotTfStamp.transform, tx);
			return mrpt::ros2bridge::fromROS(tx);
		}();

		RCLCPP_DEBUG(
			get_logger(), "[onNewSensor_pointcloud] robot pose %s",
			robotPose.asString().c_str());
	}
	catch (const tf2::TransformException& ex)
	{
		RCLCPP_ERROR(get_logger(), "%s", ex.what());
		return;
	}

	// Insert into the observation history:
	InfoPerTimeStep ipt;
	ipt.sourceTopic = topicName;
	ipt.observation = obsPts;
	ipt.robot_pose = robotPose;

	m_hist_obs_mtx.lock();
	m_hist_obs.insert(m_hist_obs.end(), obs_list_t::value_type(timestamp, ipt));
	m_hist_obs_mtx.unlock();
}  // end on_new_sensor_pointcloud

// read params from parameter server
void LocalObstaclesNode::read_parameters()
{
	this->declare_parameter<bool>("show_gui", false);
	this->get_parameter("show_gui", m_show_gui);
	RCLCPP_INFO(get_logger(), "show_gui: %s", m_show_gui ? "Yes" : "No");

	this->declare_parameter<std::string>("frameid_reference", "odom");
	this->get_parameter("frameid_reference", m_frameid_reference);
	RCLCPP_INFO(
		get_logger(), "frameid_reference: %s", m_frameid_reference.c_str());

	this->declare_parameter<std::string>("frameid_robot", "base_link");
	this->get_parameter("frameid_robot", m_frameid_robot);
	RCLCPP_INFO(get_logger(), "frameid_robot: %s", m_frameid_robot.c_str());

	this->declare_parameter<double>("time_window", 0.20);
	this->get_parameter("time_window", m_time_window);
	RCLCPP_INFO(get_logger(), "time_window: %f", m_time_window);

	this->declare_parameter<bool>(
		"one_observation_per_topic", m_one_observation_per_topic);
	this->get_parameter(
		"one_observation_per_topic", m_one_observation_per_topic);
	RCLCPP_INFO(
		get_logger(), "one_observation_per_topic: %s",
		m_one_observation_per_topic ? "true" : "false");

	this->declare_parameter<double>("publish_period", 0.05);
	this->get_parameter("publish_period", m_publish_period);
	RCLCPP_INFO(get_logger(), "publish_period: %f", m_publish_period);

	// publish_period can't be larger than m_time_window:
	ASSERT_LE_(m_publish_period, m_time_window);

	this->declare_parameter<std::string>(
		"source_topics_2d_scans", "scan, laser1");
	this->get_parameter("source_topics_2d_scans", m_topics_source_2dscan);
	RCLCPP_INFO(
		get_logger(), "source_topics_2d_scans: %s",
		m_topics_source_2dscan.c_str());

	this->declare_parameter<std::string>("source_topics_pointclouds", "");
	this->get_parameter(
		"source_topics_pointclouds", m_topics_source_pointclouds);
	RCLCPP_INFO(
		get_logger(), "source_topics_pointclouds: %s",
		m_topics_source_pointclouds.c_str());

	this->declare_parameter<std::string>(
		"pipeline_yaml_file", m_pipeline_yaml_file);
	this->get_parameter("pipeline_yaml_file", m_pipeline_yaml_file);
	RCLCPP_INFO(
		get_logger(), "pipeline_yaml_file: %s", m_pipeline_yaml_file.c_str());
	{
		ASSERT_FILE_EXISTS_(m_pipeline_yaml_file);
		const mrpt::containers::yaml cfg =
			mrpt::containers::yaml::FromFile(m_pipeline_yaml_file);

		RCLCPP_DEBUG_STREAM(get_logger(), cfg);

		ASSERT_(cfg.has("generators"));
		ASSERT_(cfg.has("per_observation"));
		ASSERT_(cfg.has("final"));

		m_generator = mp2p_icp_filters::generators_from_yaml(cfg["generators"]);
		m_per_obs_pipeline =
			mp2p_icp_filters::filter_pipeline_from_yaml(cfg["per_observation"]);
		m_final_pipeline =
			mp2p_icp_filters::filter_pipeline_from_yaml(cfg["final"]);
	}

	// Output layer(s) ==> ROS topic(s) mapping:
	// --------------------------------------------------
	std::string filter_output_layer_name = "output";
	std::string topic_local_map_pointcloud = "local_map_pointcloud";

	this->declare_parameter<std::string>(
		"topic_local_map_pointcloud", topic_local_map_pointcloud);
	this->get_parameter(
		"topic_local_map_pointcloud", topic_local_map_pointcloud);
	RCLCPP_INFO(
		get_logger(), "topic_local_map_pointcloud: %s",
		topic_local_map_pointcloud.c_str());

	this->declare_parameter<std::string>(
		"filter_output_layer_name", filter_output_layer_name);
	this->get_parameter("filter_output_layer_name", filter_output_layer_name);
	RCLCPP_INFO(
		get_logger(), "filter_output_layer_name: %s",
		filter_output_layer_name.c_str());

	// parse lists:
	std::vector<std::string> lstLayers, lstTopics;
	mrpt::system::tokenize(filter_output_layer_name, ", \t\r\n", lstLayers);
	mrpt::system::tokenize(topic_local_map_pointcloud, ", \t\r\n", lstTopics);
	ASSERT_EQUAL_(lstLayers.size(), lstTopics.size());
	for (size_t i = 0; i < lstLayers.size(); i++)
	{
		auto& e = layer2topic_.emplace_back();
		e.layer = lstLayers.at(i);
		e.topic = lstTopics.at(i);

		// Create publisher for local map point cloud:
		e.pub =
			this->create_publisher<sensor_msgs::msg::PointCloud2>(e.topic, 10);
	}
}

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);

	auto node = std::make_shared<LocalObstaclesNode>();

	rclcpp::spin(node);

	rclcpp::shutdown();

	return 0;
}
