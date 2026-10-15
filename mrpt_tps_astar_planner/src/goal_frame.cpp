/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

#include <chrono>
#include <mrpt_tps_astar_planner/goal_frame.hpp>
#include <tf2/LinearMath/Matrix3x3.hpp>
#include <tf2/LinearMath/Transform.hpp>
#include <tf2/exceptions.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace mrpt_tps_astar_planner
{
std::optional<mrpt::math::TPose2D> goal_in_map_frame(
	const geometry_msgs::msg::PoseStamped& goal, const std::string& frame_id_map,
	const tf2::BufferCore& tf, std::string* error)
{
	tf2::Transform pose;
	tf2::fromMsg(goal.pose, pose);

	const std::string& frame = goal.header.frame_id;
	if (!frame.empty() && frame != frame_id_map)
	{
		// A zero stamp means "latest" for tf2:
		const tf2::TimePoint stamp(
			std::chrono::seconds(goal.header.stamp.sec) +
			std::chrono::nanoseconds(goal.header.stamp.nanosec));

		geometry_msgs::msg::TransformStamped mapToFrame;
		try
		{
			mapToFrame = tf.lookupTransform(frame_id_map, frame, stamp);
		}
		catch (const tf2::TransformException&)
		{
			try
			{
				mapToFrame = tf.lookupTransform(frame_id_map, frame, tf2::TimePointZero);
			}
			catch (const tf2::TransformException& e)
			{
				if (error)
				{
					*error = e.what();
				}
				return std::nullopt;
			}
		}
		tf2::Transform t;
		tf2::fromMsg(mapToFrame.transform, t);
		pose = t * pose;
	}

	double roll = 0;
	double pitch = 0;
	double yaw = 0;
	tf2::Matrix3x3(pose.getRotation()).getRPY(roll, pitch, yaw);
	return mrpt::math::TPose2D(pose.getOrigin().x(), pose.getOrigin().y(), yaw);
}

}  // namespace mrpt_tps_astar_planner
