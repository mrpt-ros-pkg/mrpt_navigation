#include <mrpt_local_obstacles/mrpt_local_obstacles_node.hpp>
#include "rclcpp_components/register_node_macro.hpp"

using namespace mrpt::system;
using namespace mrpt::config;
using namespace mrpt::img;
using namespace mrpt::maps;
using namespace mrpt::obs;


LocalObstaclesNode::LocalObstaclesNode(const rclcpp::NodeOptions& options)
: Node("mrpt_local_obstacles", options)
{
  read_parameters();

  // Create publisher for local map point cloud
  m_pub_local_map_pointcloud = 
            create_publisher<sensor_msgs::msg::PointCloud2>(m_topic_local_map_pointcloud, 10);


  // Init ROS subs:
  // Subscribe to one or more laser sources:
  size_t nSubsTotal = 0;
  nSubsTotal += subscribe_to_multiple_topics<sensor_msgs::msg::LaserScan>(
                m_topics_source_2dscan, m_subs_2dlaser,
                std::bind(&LocalObstaclesNode::on_new_sensor_laser_2d, this, std::placeholders::_1));

  nSubsTotal += subscribe_to_multiple_topics<sensor_msgs::msg::PointCloud2>(
      m_topics_source_pointclouds, m_subs_pointclouds,
      std::bind(&LocalObstaclesNode::on_new_sensor_pointcloud, this, std::placeholders::_1));

  RCLCPP_INFO( this->get_logger(),
      "Total number of sensor subscriptions: %u",
       static_cast<unsigned int>(nSubsTotal));

  if(!(nSubsTotal>0))
    RCLCPP_ERROR( this->get_logger(),
      "*Error* It is mandatory to set at least one source topic for sensory information!");

  // Local map params:
  m_localmap_pts->insertionOptions.minDistBetweenLaserPoints = 0;
  m_localmap_pts->insertionOptions.also_interpolate = false;

  // Create the tf2 buffer and listener
  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);

  m_timer_publish = create_wall_timer(
            std::chrono::duration<double>(m_publish_period), 
            std::bind(&LocalObstaclesNode::on_do_publish, this));
} //end ctor

/** Callback: On recalc local map & publish it */
void LocalObstaclesNode::on_do_publish()
{
	CTimeLoggerEntry tle(m_profiler, "on_do_publish");

	// Purge old observations & latch a local copy:
	TListObservations obs;
  {
    CTimeLoggerEntry tle(m_profiler, "onDoPublish.removingOld");
    m_hist_obs_mtx.lock();

    // Purge old obs:
    if (!m_hist_obs.empty())
    {
      const double last_time = m_hist_obs.rbegin()->first;
      TListObservations::iterator it_first_valid =
        m_hist_obs.lower_bound(last_time - m_time_window);
      const size_t nToRemove =
        std::distance(m_hist_obs.begin(), it_first_valid);
      ROS_DEBUG(
        "[onDoPublish] Removing %u old entries, last_time=%lf",
        static_cast<unsigned int>(nToRemove), last_time);
      m_hist_obs.erase(m_hist_obs.begin(), it_first_valid);
    }
    // Local copy in this thread:
    obs = m_hist_obs;
    m_hist_obs_mtx.unlock();
  }

  RCLCPP_DEBUG(this->logger(),
    "Building local map with %u observations.",
    static_cast<unsigned int>(obs.size()));

  if (obs.empty()) return;

		// Build local map(s):
		// -----------------------------------------------
		m_localmap_pts->clear();
		mrpt::poses::CPose3D curRobotPose;
		{
			CTimeLoggerEntry tle2(m_profiler, "on_do_publish.buildLocalMap");

			// Get the latest robot pose in the reference frame (typ: /odom ->
			// /base_link)
			// so we can build the local map RELATIVE to it:
			std::chrono::duration<double> timeout(1.0);
      rclcpp::Duration ros2_timeout(timeout.count());

			try
			{
				geometry_msgs::TransformStamped tx;
				tx = m_tf_buffer->lookupTransform(
					m_frameid_reference, m_frameid_robot, ros::Time(0),
					timeout);

				tf2::Transform tfx;
				tf2::fromMsg(tx.transform, tfx);
				curRobotPose = mrpt::ros1bridge::fromROS(tfx);
			}
			catch (const tf2::ExtrapolationException& ex)
			{
				ROS_ERROR("%s", ex.what());
				return;
			}

			ROS_DEBUG(
				"[onDoPublish] Building local map relative to latest robot "
				"pose: %s",
				curRobotPose.asString().c_str());

			// For each observation: compute relative robot pose & insert obs
			// into map:
			for (TListObservations::const_iterator it = obs.begin();
				 it != obs.end(); ++it)
			{
				const TInfoPerTimeStep& ipt = it->second;

				// Relative pose in the past:
				mrpt::poses::CPose3D relPose(mrpt::poses::UNINITIALIZED_POSE);
				relPose.inverseComposeFrom(ipt.robot_pose, curRobotPose);

				// Insert obs:
				m_localmap_pts->insertObservationPtr(ipt.observation, relPose);

			}  // end for
		}

		// Filtering:
		mrpt::maps::CPointsMap::Ptr filteredPts;

		if (!m_filter_pipeline.empty())
		{
			mp2p_icp::metric_map_t mm;
			mm.layers[mp2p_icp::metric_map_t::PT_LAYER_RAW] = m_localmap_pts;
			mp2p_icp_filters::apply_filter_pipeline(m_filter_pipeline, mm);

			filteredPts = mm.point_layer(m_filter_output_layer_name);
		}
		else
		{
			filteredPts = m_localmap_pts;
		}

		// Publish them:
		if (m_pub_local_map_pointcloud.getNumSubscribers() > 0)
		{
			sensor_msgs::PointCloud2 msg_pts;
			msg_pts.header.frame_id = m_frameid_robot;
			msg_pts.header.stamp = ros::Time(obs.rbegin()->first);

			auto simplPts =
				std::dynamic_pointer_cast<mrpt::maps::CSimplePointsMap>(
					filteredPts);
			ASSERT_(simplPts);

			mrpt::ros1bridge::toROS(*simplPts, msg_pts.header, msg_pts);
			m_pub_local_map_pointcloud.publish(msg_pts);
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
				gl_pts->setPointSize(3.0);
				gl_pts->setColor_u8(TColor(0x0000ff));
				scene->insert(gl_pts);

				m_gui_win->unlockAccess3DScene();
			}

			auto& scene = m_gui_win->get3DSceneAndLock();
			auto gl_obs = mrpt::ptr_cast<mrpt::opengl::CSetOfObjects>::from(
				scene->getByName("obstacles"));
			ROS_ASSERT(!!gl_obs);
			gl_obs->clear();

			auto glRawPts = mrpt::ptr_cast<mrpt::opengl::CPointCloud>::from(
				scene->getByName("raw_points"));

			auto glFinalPts = mrpt::ptr_cast<mrpt::opengl::CPointCloud>::from(
				scene->getByName("final_points"));

			for (const auto& o : obs)
			{
				const TInfoPerTimeStep& ipt = o.second;
				// Relative pose in the past:
				mrpt::poses::CPose3D relPose(mrpt::poses::UNINITIALIZED_POSE);
				relPose.inverseComposeFrom(ipt.robot_pose, curRobotPose);

				mrpt::opengl::CSetOfObjects::Ptr gl_axis =
					mrpt::opengl::stock_objects::CornerXYZSimple(0.9, 2.0);
				gl_axis->setPose(relPose);
				gl_obs->insert(gl_axis);
			}  // end for

			glRawPts->loadFromPointsMap(m_localmap_pts.get());
			glFinalPts->loadFromPointsMap(filteredPts.get());

			m_gui_win->unlockAccess3DScene();
			m_gui_win->repaint();
		}

	}  // onDoPublish

}

void LocalObstaclesNode::on_new_sensor_laser_2d(const sensor_msgs::msg::LaserScan::SharedPtr& scan)
{

}

void LocalObstaclesNode::on_new_sensor_pointcloud(const sensor_msgs::msg::PointCloud2::SharedPtr& pts)
{

}

template <typename MessageT, typename CallbackMethodType>
size_t LocalObstaclesNode::subscribe_to_multiple_topics(const std::string& lstTopics,
        std::vector<typename rclcpp::Subscription<MessageT>::SharedPtr>& subscriptions,
        CallbackMethodType callback)
{

}

void LocalObstaclesNode::read_parameters()
{
  
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<LocalObstaclesNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
