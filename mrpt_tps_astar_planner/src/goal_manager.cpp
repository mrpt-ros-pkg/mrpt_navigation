/* +------------------------------------------------------------------------+
   |                             mrpt_navigation                            |
   |                                                                        |
   | Copyright (c) 2014-2026, Individual contributors, see commit authors   |
   | See: https://github.com/mrpt-ros-pkg/mrpt_navigation                   |
   | All rights reserved. Released under BSD 3-Clause license. See LICENSE  |
   +------------------------------------------------------------------------+ */

// Goal manager: plans navigation goals in a background thread. The best
// partial path is published early (anytime planning), and the search keeps
// going while the robot drives (continuous planning): each cycle, the search
// root moves to a node ahead of the robot on the published path (so the path
// up to there is committed and never changes), the search goes on with the
// latest obstacles, and the published path is checked against them,
// restarting the search if it is blocked.

#include <mpp/ptgs/SpeedTrimmablePTG.h>
#include <mrpt/math/wrap2pi.h>

#include <cmath>
#include <mrpt_tps_astar_planner/goal_frame.hpp>
#include <mrpt_tps_astar_planner/mrpt_tps_astar_planner_node.hpp>
#include <unordered_map>

namespace mrpt_tps_astar_planner
{
bool TPS_Astar_Planner_Node::goal_manager_enabled() const
{
#if HAVE_MPP_CONTINUOUS_PLANNING
	return continuous_planning_ || anytime_first_path_time_ > 0;
#else
	return false;
#endif
}

#if HAVE_MPP_CONTINUOUS_PLANNING

namespace
{
using mrpt::math::TPoint2D;
using mrpt::math::TPose2D;

// Status strings of mrpt_trajectory_follower:
const char* FOLLOWER_RUNNING = "Running";
const char* FOLLOWER_WAITING = "WaitingForPath";
const char* FOLLOWER_REACHED = "ReachedGoal";
const char* FOLLOWER_FAILED = "Failed";

// Time between path samples [s] (as for one-shot plans):
constexpr double PATH_SAMPLE_PERIOD = 0.25;

double now_seconds()
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
		.count();
}

// Poses along PTG edges (global frame), appended to `out`. `edgeEnd`
// receives, for each edge, the index in `out` of its last pose.
void sample_edges(
	const std::vector<mpp::MoveEdgeSE2_TPS*>& edges, const mpp::TrajectoriesAndRobotShape& trs,
	std::vector<TPose2D>& out, std::vector<size_t>& edgeEnd)
{
	for (const auto* e : edges)
	{
		auto& ptg = trs.ptgs.at(e->ptgIndex);
		ptg->updateNavDynamicState(e->getPTGDynState());
		if (auto* trim = dynamic_cast<mpp::ptg::SpeedTrimmablePTG*>(ptg.get()); trim != nullptr)
		{
			trim->trimmableSpeed_ = e->ptgTrimmableSpeed;
		}
		const auto finalStepOpt = ptg->getPathStepForDist(e->ptgPathIndex, e->ptgDist);
		const uint32_t finalStep = finalStepOpt ? *finalStepOpt : e->ptgStepIndex;
		const uint32_t stepIncr = std::max<uint32_t>(
			1, static_cast<uint32_t>(std::round(PATH_SAMPLE_PERIOD / ptg->getPathStepDuration())));

		// The first pose of an edge is the last one of the previous edge:
		for (uint32_t step = out.empty() ? 0 : std::min(stepIncr, finalStep);; step += stepIncr)
		{
			step = std::min(step, finalStep);
			out.push_back(e->stateFrom.pose + ptg->getPathPose(e->ptgPathIndex, step));
			if (step == finalStep)
			{
				break;
			}
		}
		edgeEnd.push_back(out.size() - 1);
	}
}

// Cumulative arc-length of a path:
std::vector<double> arc_lengths(const std::vector<TPose2D>& poses)
{
	std::vector<double> s(poses.size(), 0.0);
	for (size_t i = 1; i < poses.size(); i++)
	{
		s[i] = s[i - 1] + (poses[i].translation() - poses[i - 1].translation()).norm();
	}
	return s;
}

// Index of the path pose nearest to `p`, searching around `hint`:
size_t nearest_pose(const std::vector<TPose2D>& poses, const TPoint2D& p, size_t hint)
{
	constexpr size_t kBack = 10;
	constexpr size_t kFwd = 80;
	size_t best = hint;
	double bestD2 = std::numeric_limits<double>::infinity();
	const size_t i0 = hint > kBack ? hint - kBack : 0;
	const size_t i1 = std::min(poses.size(), hint + kFwd);
	for (size_t i = i0; i < i1; i++)
	{
		const double d2 = mrpt::square(poses[i].x - p.x) + mrpt::square(poses[i].y - p.y);
		if (d2 < bestD2)
		{
			bestD2 = d2;
			best = i;
		}
	}
	return best;
}

// Distance from a point (robot frame) to the robot footprint (<=0: inside).
double distance_to_footprint(const mrpt::math::TPolygon2D& poly, const TPoint2D& q)
{
	double minD2 = std::numeric_limits<double>::infinity();
	for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
	{
		const auto& a = poly[j];
		const auto& b = poly[i];
		const double ex = b.x - a.x;
		const double ey = b.y - a.y;
		const double l2 = ex * ex + ey * ey;
		const double t =
			l2 > 0 ? std::clamp(((q.x - a.x) * ex + (q.y - a.y) * ey) / l2, 0.0, 1.0) : 0.0;
		minD2 = std::min(
			minD2, mrpt::square(q.x - (a.x + t * ex)) + mrpt::square(q.y - (a.y + t * ey)));
	}
	return poly.contains(q) ? 0.0 : std::sqrt(minD2);
}

// Obstacle points in a coarse 2D hash grid, for fast footprint checks.
class ObstacleIndex
{
   public:
	explicit ObstacleIndex(double cellSize) : cell_(cellSize) {}

	void add(const mrpt::maps::CPointsMap& pts)
	{
		const auto& xs = pts.getPointsBufferRef_x();
		const auto& ys = pts.getPointsBufferRef_y();
		for (size_t i = 0; i < xs.size(); i++)
		{
			cells_[key(idx(xs[i]), idx(ys[i]))].emplace_back(xs[i], ys[i]);
		}
	}

	// An obstacle within `margin` of the footprint at `pose`, if any.
	std::optional<TPoint2D> collides(
		const TPose2D& pose, const mrpt::math::TPolygon2D& footprint, double footprintRadius,
		double margin) const
	{
		const double r = footprintRadius + margin;
		const int64_t ix0 = idx(pose.x - r);
		const int64_t ix1 = idx(pose.x + r);
		const int64_t iy0 = idx(pose.y - r);
		const int64_t iy1 = idx(pose.y + r);
		const double c = std::cos(pose.phi);
		const double s = std::sin(pose.phi);
		for (int64_t ix = ix0; ix <= ix1; ix++)
		{
			for (int64_t iy = iy0; iy <= iy1; iy++)
			{
				const auto it = cells_.find(key(ix, iy));
				if (it == cells_.end())
				{
					continue;
				}
				for (const auto& p : it->second)
				{
					const double dx = p.x - pose.x;
					const double dy = p.y - pose.y;
					if (dx * dx + dy * dy > r * r)
					{
						continue;
					}
					// In the robot frame:
					const TPoint2D q(c * dx + s * dy, -s * dx + c * dy);
					if (distance_to_footprint(footprint, q) <= margin)
					{
						return p;
					}
				}
			}
		}
		return std::nullopt;
	}

   private:
	double cell_;
	std::unordered_map<int64_t, std::vector<TPoint2D>> cells_;

	int64_t idx(double v) const { return static_cast<int64_t>(std::floor(v / cell_)); }
	static int64_t key(int64_t ix, int64_t iy) { return (ix << 32) ^ (iy & 0xffffffff); }
};

// The first pose index from `from` whose footprint collides, and the obstacle
// point, checking also between samples (every `step` meters).
std::optional<std::pair<size_t, TPoint2D>> first_colliding_pose(
	const std::vector<TPose2D>& poses, size_t from, const ObstacleIndex& obs,
	const mrpt::math::TPolygon2D& footprint, double footprintRadius, double margin)
{
	constexpr double step = 0.05;  // [m]
	for (size_t i = from; i < poses.size(); i++)
	{
		if (i == from)
		{
			if (const auto p = obs.collides(poses[i], footprint, footprintRadius, margin); p)
			{
				return std::make_pair(i, *p);
			}
			continue;
		}
		const auto& a = poses[i - 1];
		const auto& b = poses[i];
		const double len = (b.translation() - a.translation()).norm();
		const int n = std::max(1, static_cast<int>(std::ceil(len / step)));
		const double dphi = mrpt::math::wrapToPi(b.phi - a.phi);
		for (int k = 1; k <= n; k++)
		{
			const double t = static_cast<double>(k) / n;
			const TPose2D q(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.phi + t * dphi);
			if (const auto p = obs.collides(q, footprint, footprintRadius, margin); p)
			{
				return std::make_pair(i, *p);
			}
		}
	}
	return std::nullopt;
}

nav_msgs::msg::Path to_path_msg(
	const std::vector<TPose2D>& poses, size_t i0, size_t i1, const std::string& frame,
	const rclcpp::Time& stamp)
{
	nav_msgs::msg::Path p;
	p.header.frame_id = frame;
	p.header.stamp = stamp;
	for (size_t i = i0; i < std::min(i1, poses.size()); i++)
	{
		auto& q = p.poses.emplace_back();
		q.header = p.header;
		q.pose = mrpt::ros2bridge::toROS_Pose(poses[i]);
	}
	return p;
}

}  // namespace

void TPS_Astar_Planner_Node::submit_goal(
	const mrpt::math::TPose2D& goalInMap, std::shared_ptr<HandleNavigateGoal> h)
{
	{
		auto lck = std::lock_guard(goal_cs_);
		ActiveGoal g;
		g.id = next_goal_id_++;
		g.pose = goalInMap;
		g.action = std::move(h);
		if (pending_goal_ && pending_goal_->action)
		{
			// Replaced before being started:
			auto result = std::make_shared<NavigateGoal::Result>();
			result->state.navigation_status =
				mrpt_nav_interfaces::msg::NavigationFinalStatus::STATUS_CANCELLED;
			pending_goal_->action->abort(result);
		}
		pending_goal_ = std::move(g);
		RCLCPP_INFO(
			get_logger(), "New navigation goal #%u: (%.2f, %.2f, %.1f deg)", pending_goal_->id,
			goalInMap.x, goalInMap.y, mrpt::RAD2DEG(goalInMap.phi));
	}
	goal_cv_.notify_all();
}

void TPS_Astar_Planner_Node::callback_follower_status(const std_msgs::msg::String& msg)
{
	{
		auto lck = std::lock_guard(goal_cs_);
		if (msg.data == last_follower_status_)
		{
			return;
		}
		last_follower_status_ = msg.data;
		follower_status_changed_ = true;
		if (msg.data == FOLLOWER_RUNNING || msg.data == FOLLOWER_WAITING)
		{
			follower_active_seen_ = true;
		}
	}
	goal_cv_.notify_all();
}

rclcpp_action::GoalResponse TPS_Astar_Planner_Node::action_handle_goal(
	[[maybe_unused]] const rclcpp_action::GoalUUID& uuid,
	std::shared_ptr<const NavigateGoal::Goal> goal)
{
	if (!check_ptgs_ready("NavigateGoal action request"))
	{
		return rclcpp_action::GoalResponse::REJECT;
	}
	std::string error;
	if (!goal_in_map_frame(goal->target, frame_id_map_, *tf_buffer_, &error))
	{
		RCLCPP_ERROR(
			get_logger(), "NavigateGoal: cannot express the goal in '%s', rejecting it: %s",
			frame_id_map_.c_str(), error.c_str());
		return rclcpp_action::GoalResponse::REJECT;
	}
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TPS_Astar_Planner_Node::action_handle_cancel(
	[[maybe_unused]] const std::shared_ptr<HandleNavigateGoal> goal_handle)
{
	{
		auto lck = std::lock_guard(goal_cs_);
		cancel_requested_ = true;
	}
	goal_cv_.notify_all();
	return rclcpp_action::CancelResponse::ACCEPT;
}

void TPS_Astar_Planner_Node::action_handle_accepted(
	const std::shared_ptr<HandleNavigateGoal> goal_handle)
{
	std::string error;
	const auto g =
		goal_in_map_frame(goal_handle->get_goal()->target, frame_id_map_, *tf_buffer_, &error);
	if (!g)
	{
		auto result = std::make_shared<NavigateGoal::Result>();
		result->state.navigation_status =
			mrpt_nav_interfaces::msg::NavigationFinalStatus::STATUS_ERROR;
		goal_handle->abort(result);
		return;
	}
	submit_goal(*g, goal_handle);
}

void TPS_Astar_Planner_Node::publish_empty_paths()
{
	const auto stamp = this->now();
	mrpt_msgs::msg::WaypointSequence empty;
	empty.header.frame_id = frame_id_map_;
	empty.header.stamp = stamp;
	publish_waypoint_sequence(empty);

	mrpt_nav_interfaces::msg::ReferencePath rp;
	rp.path.header = empty.header;
	pub_reference_path_->publish(rp);
	pub_committed_path_->publish(rp.path);
	pub_tentative_path_->publish(rp.path);
}

void TPS_Astar_Planner_Node::goal_manager_loop()
{
	using NavStatus = mrpt_nav_interfaces::msg::NavigationFinalStatus;

	// The planner of this thread, with its own PTGs (see get_thread_ptgs()):
	mpp::TPS_Astar planner;
	planner.setMinLoggingLevel(mrpt::system::LVL_WARN);
	bool plannerReady = false;

	std::optional<ActiveGoal> goal;

	// State of the current goal:
	double tGoalStart = 0;	// when the goal was received [steady s]
	double tSearchStart = 0;  // when the current search started
	int restarts = 0;
	uint32_t revision = 0;
	bool published = false;	 // a path was published for this goal
	bool publishedFull = false;	 // ...reaching the goal
	bool followerEngaged = false;  // follower acknowledged the current path
	std::optional<mrpt::graphs::TNodeID> publishedTarget;

	// The published path, global frame. Its first `rootIdx+1` poses are the
	// committed part, up to the search root:
	std::vector<TPose2D> pathPoses;
	std::vector<double> pathS;
	size_t rootIdx = 0;
	size_t robotIdx = 0;
	// Tree nodes along the published path after the root, with their pose
	// index in pathPoses (in order):
	std::vector<std::pair<mrpt::graphs::TNodeID, size_t>> pathNodes;

	auto endGoal = [&](int8_t status, const char* why)
	{
		if (goal)
		{
			RCLCPP_INFO(get_logger(), "Navigation goal #%u ended: %s", goal->id, why);
			if (goal->action)
			{
				auto result = std::make_shared<NavigateGoal::Result>();
				result->state.navigation_status = status;
				if (status == NavStatus::STATUS_SUCCESS)
				{
					goal->action->succeed(result);
				}
				else if (status == NavStatus::STATUS_CANCELLED && goal->action->is_canceling())
				{
					goal->action->canceled(result);
				}
				else
				{
					goal->action->abort(result);
				}
			}
		}
		goal.reset();
	};

	auto robotPose = [&]() -> std::optional<TPose2D>
	{
		mrpt::poses::CPose3D p;
		if (!wait_for_transform(p, frame_id_robot_, frame_id_map_))
		{
			return std::nullopt;
		}
		return mrpt::poses::CPose2D(p).asTPose();
	};

	double maxLinSpeed = 0;
	bool searchJustStarted = false;	 // the search has the latest obstacles

	// Starts a search from `from` (the robot pose, or the root, keeping the
	// committed part of the path before it):
	auto startSearch = [&](const TPose2D& from)
	{
		mpp::PlannerInput pi;
		pi.ptgs = get_thread_ptgs();
		pi.stateStart.pose = from;
		pi.stateGoal.state = goal->pose;

		planner.costEvaluators_.clear();
		auto bbox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
		build_obstacles(from, pi.obstacles, nullptr, &planner.costEvaluators_, bbox);
		if (reverse_motion_cost_factor_ > 0)
		{
			auto ev = mpp::CostEvaluatorReverseMotion::Create();
			ev->params_.reverseTimeCostFactor = reverse_motion_cost_factor_;
			ev->setPTGs(pi.ptgs);
			planner.costEvaluators_.push_back(ev);
		}
		const auto margin =
			mrpt::math::TPoint3Df(problem_world_bbox_margin_, problem_world_bbox_margin_, .0f);
		for (const auto& p : {from.translation(), goal->pose.translation()})
		{
			const auto pt = mrpt::math::TPoint3Df(p.x, p.y, 0);
			bbox.updateWithPoint(pt - margin);
			bbox.updateWithPoint(pt + margin);
		}
		pi.worldBboxMax = {bbox.max.x, bbox.max.y, M_PI};
		pi.worldBboxMin = {bbox.min.x, bbox.min.y, -M_PI};

		planner.start(pi);
		searchJustStarted = true;
		tSearchStart = now_seconds();
		publishedTarget.reset();
		pathNodes.clear();
	};

	// Restarts the search from scratch, from the robot pose (dropping the
	// path) or from the root (keeping the committed part). Returns false if
	// there were too many restarts.
	auto restartSearch = [&](bool fromRobot, const char* why) -> bool
	{
		restarts++;
		RCLCPP_WARN(
			get_logger(), "Goal #%u: restarting the search from the %s (%d/%d): %s", goal->id,
			fromRobot ? "robot pose" : "committed path end", restarts, max_planning_restarts_, why);
		if (restarts > max_planning_restarts_)
		{
			publish_empty_paths();
			endGoal(NavStatus::STATUS_ERROR, "too many planning restarts");
			return false;
		}
		std::optional<TPose2D> from;
		if (fromRobot || pathPoses.empty() || rootIdx <= robotIdx)
		{
			from = robotPose();
			pathPoses.clear();
			pathS.clear();
			rootIdx = 0;
			robotIdx = 0;
		}
		else
		{
			from = pathPoses.at(rootIdx);
		}
		if (!from)
		{
			publish_empty_paths();
			endGoal(NavStatus::STATUS_ERROR, "no robot pose");
			return false;
		}
		startSearch(*from);
		return true;
	};

	// Publishes the path to `target` in the search output, after the
	// committed part:
	auto publishPath = [&](mpp::PlannerOutput& out, mrpt::graphs::TNodeID target, bool partial)
	{
		auto [nodes, edgeList] = out.motionTree.backtrack_path(target);
		if (!astar_skip_refine_)
		{
			mpp::refine_trajectory(nodes, edgeList, out.originalInput.ptgs);
		}
		std::vector<mpp::MoveEdgeSE2_TPS*> edges;
		for (auto* e : edgeList)
		{
			if (e != nullptr)
			{
				edges.push_back(e);
			}
		}

		// Keep the committed part (from a bit behind the robot, to avoid
		// growing forever) up to the root:
		std::vector<TPose2D> newPoses;
		constexpr size_t kKeepBehind = 4;
		const size_t first = std::min(robotIdx > kKeepBehind ? robotIdx - kKeepBehind : 0, rootIdx);
		if (!pathPoses.empty())
		{
			newPoses.assign(pathPoses.begin() + first, pathPoses.begin() + rootIdx + 1);
		}
		else
		{
			newPoses.push_back(out.originalInput.stateStart.pose);
		}
		const size_t newRootIdx = newPoses.size() - 1;
		std::vector<size_t> edgeEnd;
		sample_edges(edges, out.originalInput.ptgs, newPoses, edgeEnd);
		if (!partial)
		{
			// End exactly at the goal (refinement may leave it slightly off):
			newPoses.back() = goal->pose;
		}

		pathNodes.clear();
		auto itNode = nodes.begin();
		for (size_t i = 0; i < edgeEnd.size() && itNode != nodes.end(); i++)
		{
			++itNode;
			if (itNode != nodes.end())
			{
				pathNodes.emplace_back(itNode->nodeID_, edgeEnd[i]);
			}
		}

		robotIdx -= std::min(robotIdx, first);
		rootIdx = newRootIdx;
		pathPoses = std::move(newPoses);
		pathS = arc_lengths(pathPoses);
		publishedTarget = target;
		published = true;
		publishedFull = !partial;
		followerEngaged = false;
		revision++;

		const auto stamp = this->now();
		mrpt_nav_interfaces::msg::ReferencePath rp;
		rp.path = to_path_msg(pathPoses, 0, pathPoses.size(), frame_id_map_, stamp);
		rp.goal_id = goal->id;
		rp.revision = revision;
		rp.is_partial = partial;
		pub_reference_path_->publish(rp);

		// Also as a waypoint sequence (other path followers, visualization):
		mrpt_msgs::msg::WaypointSequence wps;
		wps.header = rp.path.header;
		for (size_t i = 0; i < pathPoses.size(); i++)
		{
			const bool last = i + 1 == pathPoses.size();
			auto& wp = wps.waypoints.emplace_back();
			wp.target = mrpt::ros2bridge::toROS_Pose(pathPoses[i]);
			wp.allowed_distance =
				last ? final_waypoint_allowed_distance_ : mid_waypoints_allowed_distance_;
			wp.allow_skip = last ? final_waypoint_allow_skip_ : mid_waypoints_allow_skip_;
			wp.ignore_heading =
				last ? final_waypoint_ignore_heading_ : mid_waypoints_ignore_heading_;
		}
		publish_waypoint_sequence(wps);

		pub_committed_path_->publish(to_path_msg(pathPoses, 0, rootIdx + 1, frame_id_map_, stamp));
		pub_tentative_path_->publish(
			to_path_msg(pathPoses, rootIdx, pathPoses.size(), frame_id_map_, stamp));

		RCLCPP_INFO(
			get_logger(),
			"Goal #%u: published %s path (rev. %u, %zu poses, %.1f m, %.1f s after goal)", goal->id,
			partial ? "PARTIAL" : "full", revision, pathPoses.size(), pathS.back(),
			now_seconds() - tGoalStart);
	};

	// Publishes the path up to pose `idx` (exclusive), so the follower stops
	// before it while a new path is searched:
	auto publishTruncated = [&](size_t idx)
	{
		const auto stamp = this->now();
		mrpt_nav_interfaces::msg::ReferencePath rp;
		rp.path = to_path_msg(pathPoses, 0, std::max<size_t>(idx, 2), frame_id_map_, stamp);
		rp.goal_id = goal->id;
		rp.revision = ++revision;
		rp.is_partial = true;
		pub_reference_path_->publish(rp);
		pub_tentative_path_->publish(to_path_msg({}, 0, 0, frame_id_map_, stamp));
		publishedFull = false;
	};

	mrpt::math::TPolygon2D footprintPoly;
	double footprintRadius = 0;
	// Obstacles closer than this to the footprint block the path [m]:
	constexpr double kValidationMargin = 0.02;

	while (true)
	{
		const double tCycleStart = now_seconds();

		// Events:
		std::optional<ActiveGoal> newGoal;
		bool cancel = false;
		std::string followerStatus;
		bool statusChanged = false;
		bool followerActiveSeen = false;
		{
			std::unique_lock lck(goal_cs_);
			if (shutting_down_)
			{
				break;
			}
			newGoal = std::exchange(pending_goal_, std::nullopt);
			cancel = std::exchange(cancel_requested_, false);
			followerStatus = last_follower_status_;
			statusChanged = std::exchange(follower_status_changed_, false);
			followerActiveSeen = std::exchange(follower_active_seen_, false);
		}

		if (!plannerReady && ptgs_ready_)
		{
			planner.params_from_yaml(planner_params_yaml_);
			footprintPoly = mpp::robotShapeAsPolygon(ptgs_.robotShape);
			for (const auto& p : footprintPoly)
			{
				footprintRadius = std::max(footprintRadius, p.norm());
			}
			for (const auto& ptg : ptgs_.ptgs)
			{
				maxLinSpeed = std::max(maxLinSpeed, ptg->getMaxLinVel());
			}
			plannerReady = true;
		}

		if (newGoal)
		{
			if (goal)
			{
				endGoal(NavStatus::STATUS_CANCELLED, "replaced by a new goal");
			}
			goal = std::move(newGoal);
			tGoalStart = now_seconds();
			restarts = 0;
			revision = 0;
			published = false;
			publishedFull = false;
			followerEngaged = false;
			pathPoses.clear();
			pathS.clear();
			rootIdx = 0;
			robotIdx = 0;
			if (!plannerReady)
			{
				endGoal(NavStatus::STATUS_ERROR, "PTGs not ready");
			}
			else if (const auto rp = robotPose(); rp)
			{
				startSearch(*rp);
			}
			else
			{
				publish_empty_paths();
				endGoal(NavStatus::STATUS_ERROR, "no robot pose");
			}
		}
		if (cancel && goal && goal->action && goal->action->is_canceling())
		{
			publish_empty_paths();
			endGoal(NavStatus::STATUS_CANCELLED, "canceled");
		}

		if (goal && plannerReady)
		{
			try
			{
				const auto rp = robotPose();

				// Follower status: act on terminal states only once the follower
				// is following one of our paths for this goal.
				if (followerActiveSeen || followerStatus == FOLLOWER_RUNNING ||
					followerStatus == FOLLOWER_WAITING)
				{
					followerEngaged = published;
				}
				if (followerEngaged && statusChanged)
				{
					if (followerStatus == FOLLOWER_REACHED && publishedFull)
					{
						endGoal(NavStatus::STATUS_SUCCESS, "goal reached");
					}
					else if (followerStatus == FOLLOWER_FAILED)
					{
						endGoal(NavStatus::STATUS_ERROR, "the path follower failed");
					}
					else if (
						followerStatus == "Blocked" || followerStatus == "OffPathExceeded" ||
						followerStatus == "MissedGoal")
					{
						followerEngaged = false;
						restartSearch(true, followerStatus.c_str());
					}
				}

				// Robot progress along the published path:
				if (goal && rp && !pathPoses.empty())
				{
					robotIdx = nearest_pose(pathPoses, rp->translation(), robotIdx);
				}

				// Current obstacles:
				std::vector<mpp::ObstacleSource::Ptr> obsSources;
				std::vector<mrpt::maps::CPointsMap::Ptr> obsPoints;
				if (goal && continuous_planning_)
				{
					auto unusedBbox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
					build_obstacles(
						rp ? *rp : goal->pose, obsSources, &obsPoints, nullptr, unusedBbox);
				}

				// Is the rest of the published path still clear?
				if (goal && continuous_planning_ && published && rp &&
					robotIdx + 1 < pathPoses.size())
				{
					ObstacleIndex index(0.5);
					for (const auto& pts : obsPoints)
					{
						index.add(*pts);
					}
					const auto hitInfo = first_colliding_pose(
						pathPoses, robotIdx + 1, index, footprintPoly, footprintRadius,
						kValidationMargin);
					if (hitInfo)
					{
						const std::optional<size_t> hit = hitInfo->first;
						// Stop well before it while searching a new path:
						constexpr double kStopBefore = 1.0;	 // [m]
						size_t stopIdx = *hit;
						while (stopIdx > robotIdx + 1 && pathS[*hit] - pathS[stopIdx] < kStopBefore)
						{
							stopIdx--;
						}
						RCLCPP_WARN(
							get_logger(),
							"Goal #%u: the path is blocked %.1f m ahead, at (%.2f, %.2f), by an "
							"obstacle at (%.2f, %.2f).",
							goal->id, pathS[*hit] - pathS[robotIdx], pathPoses[*hit].x,
							pathPoses[*hit].y, hitInfo->second.x, hitInfo->second.y);
						publishTruncated(stopIdx + 1);
						// Only the part before the obstacle remains (and the
						// committed part, if it is clear, to go on from its end):
						const size_t keep = std::max(stopIdx, rootIdx) + 1;
						if (keep < pathPoses.size())
						{
							pathPoses.resize(keep);
							pathS.resize(keep);
						}
						restartSearch(*hit <= rootIdx || rootIdx <= robotIdx, "path blocked");
					}
				}

				// Move the search root ahead of the robot on the path: the path
				// up to it is committed.
				if (goal && continuous_planning_ && published && rp && !pathNodes.empty())
				{
					const double sStitch = pathS[robotIdx] + stitch_lookahead_time_ * maxLinSpeed;
					for (size_t k = 0; k < pathNodes.size(); k++)
					{
						if (pathS[pathNodes[k].second] < sStitch)
						{
							continue;
						}
						if (planner.reroot(pathNodes[k].first))
						{
							rootIdx = pathNodes[k].second;
							pathNodes.erase(pathNodes.begin(), pathNodes.begin() + k + 1);
							pub_committed_path_->publish(
								to_path_msg(pathPoses, 0, rootIdx + 1, frame_id_map_, this->now()));
						}
						break;
					}
				}

				// Search:
				if (goal && planner.search_can_continue())
				{
					if (!obsSources.empty() && !searchJustStarted)
					{
						planner.set_obstacles(obsSources);
					}
					searchJustStarted = false;
					const double sinceGoal = now_seconds() - tGoalStart;
					double budget =
						std::max(0.05, replan_period_ - (now_seconds() - tCycleStart) - 0.05);
					if (!published && anytime_first_path_time_ > 0)
					{
						// Until a first path is out, search in short slices to
						// check for a partial path in time:
						constexpr double kAnytimeSlice = 0.1;  // [s]
						budget = std::min(
							budget, sinceGoal < anytime_first_path_time_
										? std::max(0.01, anytime_first_path_time_ - sinceGoal)
										: kAnytimeSlice);
					}

					mpp::PlannerOutput out;
					{
#if defined(MPP_HAS_PTG_CLONE)
						out = planner.run(budget);
#else
						auto lckPlan = mrpt::lockHelper(planning_cs_);
						out = planner.run(budget);
#endif
					}

					const double searchTime = now_seconds() - tSearchStart;
					if (out.success && out.goalNodeId)
					{
						if (!publishedFull || publishedTarget != out.goalNodeId)
						{
							publishPath(out, *out.goalNodeId, false);
						}
					}
					else if (
						!publishedFull && anytime_first_path_time_ > 0 &&
						now_seconds() - tGoalStart >= anytime_first_path_time_ && out.bestNodeId &&
						publishedTarget != out.bestNodeId)
					{
						// Best partial path, if it gets closer enough to the goal.
						// A full path is never replaced by a partial one: the
						// search may still be improving a solution it found.
						const auto& best = out.motionTree.nodes().at(*out.bestNodeId).pose;
						const auto& root = out.originalInput.stateStart.pose;
						const double dBest = (best.translation() - goal->pose.translation()).norm();
						const double dRoot = (root.translation() - goal->pose.translation()).norm();
						if (dBest <= dRoot - anytime_min_progress_)
						{
							publishPath(out, *out.bestNodeId, true);
						}
					}

					if (!planner.search_reached_goal() &&
						(!planner.search_can_continue() ||
						 searchTime > planner.params_.maximumComputationTime))
					{
						const char* why = planner.search_can_continue()
											  ? "search timeout"
											  : "no path in the search tree";
						if (!published)
						{
							RCLCPP_WARN(
								get_logger(), "Goal #%u: no path found (%s).", goal->id, why);
							publish_empty_paths();
							endGoal(NavStatus::STATUS_ERROR, "no path found");
						}
						else
						{
							restartSearch(true, why);
						}
					}
				}

				// Feedback:
				if (goal && goal->action)
				{
					auto fb = std::make_shared<NavigateGoal::Feedback>();
					fb->state.total_waypoints = 1;
					fb->state.reached_waypoints = 0;
					if (rp)
					{
						fb->state.distance_to_goal = static_cast<float>(
							(rp->translation() - goal->pose.translation()).norm());
					}
					fb->state.status = !published
										   ? "Planning"
										   : (publishedFull ? "Following" : "FollowingPartialPath");
					goal->action->publish_feedback(fb);
				}
			}
			catch (const std::exception& e)
			{
				RCLCPP_ERROR(get_logger(), "Goal manager: %s", e.what());
				publish_empty_paths();
				endGoal(NavStatus::STATUS_ERROR, "planning error");
			}
		}

		// Wait for the next cycle, or an event:
		{
			std::unique_lock lck(goal_cs_);
			const double elapsed = now_seconds() - tCycleStart;
			// (no wait while searching for a first path)
			const bool searchingFirst = goal && !published && planner.search_can_continue();
			const double waitTime =
				!goal ? 1.0 : (searchingFirst ? 0.0 : std::max(0.0, replan_period_ - elapsed));
			goal_cv_.wait_for(
				lck, std::chrono::duration<double>(waitTime),
				[this]()
				{
					return shutting_down_ || pending_goal_.has_value() || cancel_requested_ ||
						   follower_status_changed_;
				});
		}
	}
}

#else  // HAVE_MPP_CONTINUOUS_PLANNING

void TPS_Astar_Planner_Node::submit_goal(
	[[maybe_unused]] const mrpt::math::TPose2D& goalInMap,
	[[maybe_unused]] std::shared_ptr<HandleNavigateGoal> h)
{
}

#endif	// HAVE_MPP_CONTINUOUS_PLANNING

}  // namespace mrpt_tps_astar_planner
