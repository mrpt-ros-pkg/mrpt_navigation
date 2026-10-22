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
#include <mrpt_tps_astar_planner/path_validation.hpp>
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
using path_validation::arc_lengths;
using path_validation::distance_to_footprint;
using path_validation::first_colliding_pose;
using path_validation::nearest_pose;
using path_validation::ObstacleIndex;

// Status strings of mrpt_trajectory_follower:
const char* FOLLOWER_RUNNING = "Running";
const char* FOLLOWER_WAITING = "WaitingForPath";
const char* FOLLOWER_REACHED = "ReachedGoal";
const char* FOLLOWER_FAILED = "Failed";
const char* FOLLOWER_BLOCKED = "Blocked";

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
	// A goal replaced before being started (aborted after releasing the lock:
	// rclcpp_action takes its own locks and may call back into this node):
	std::shared_ptr<HandleNavigateGoal> replaced;
	{
		auto lck = std::lock_guard(goal_cs_);
		ActiveGoal g;
		g.id = next_goal_id_++;
		g.pose = goalInMap;
		g.action = std::move(h);
		if (pending_goal_ && pending_goal_->action)
		{
			replaced = std::move(pending_goal_->action);
		}
		pending_goal_ = std::move(g);
		RCLCPP_INFO(
			get_logger(), "New navigation goal #%u: (%.2f, %.2f, %.1f deg)", pending_goal_->id,
			goalInMap.x, goalInMap.y, mrpt::RAD2DEG(goalInMap.phi));
	}
	if (replaced)
	{
		auto result = std::make_shared<NavigateGoal::Result>();
		result->state.navigation_status =
			mrpt_nav_interfaces::msg::NavigationFinalStatus::STATUS_CANCELLED;
		replaced->abort(result);
	}
	goal_cv_.notify_all();
}

void TPS_Astar_Planner_Node::callback_follower_status(const std_msgs::msg::String& msg)
{
	{
		auto lck = std::lock_guard(goal_cs_);
		if (msg.data == last_follower_status_ && !follower_status_has_id_)
		{
			return;
		}
		last_follower_status_ = msg.data;
		follower_status_has_id_ = false;
		follower_status_changed_ = true;
		if (msg.data == FOLLOWER_RUNNING || msg.data == FOLLOWER_WAITING)
		{
			follower_active_seen_ = true;
		}
	}
	goal_cv_.notify_all();
}

void TPS_Astar_Planner_Node::callback_follower_status_ex(
	const mrpt_nav_interfaces::msg::FollowerStatus& msg)
{
	{
		auto lck = std::lock_guard(goal_cs_);
		if (msg.status == last_follower_status_ && follower_status_has_id_ == msg.has_goal_id &&
			follower_status_goal_id_ == msg.goal_id && follower_status_revision_ == msg.revision)
		{
			return;
		}
		last_follower_status_ = msg.status;
		follower_status_has_id_ = msg.has_goal_id;
		follower_status_goal_id_ = msg.goal_id;
		follower_status_revision_ = msg.revision;
		follower_status_changed_ = true;
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
	// The first path revision published by the current search:
	uint32_t searchFirstRevision = 1;
	bool published = false;	 // a path was published for this goal
	bool publishedFull = false;	 // ...reaching the goal
	bool followerEngaged = false;  // follower acknowledged the current path
	std::optional<mrpt::graphs::TNodeID> publishedTarget;
	// The node a partial path was asked for (publishedTarget is the one at
	// the end of the published prefix, if it was cut short):
	std::optional<mrpt::graphs::TNodeID> requestedTarget;

	// The published path, global frame. Its first `rootIdx+1` poses are the
	// committed part, up to the search root:
	std::vector<TPose2D> pathPoses;
	std::vector<double> pathS;
	size_t rootIdx = 0;
	size_t robotIdx = 0;
	// Tree nodes along the published path after the root, in order: with their
	// pose index in pathPoses, and their tree pose when published (to notice
	// if the search later rewires them):
	struct PathNode
	{
		mrpt::graphs::TNodeID id;
		size_t poseIdx;
		TPose2D treePose;
	};
	std::vector<PathNode> pathNodes;
	mpp::PlannerOutput lastOut;	 // of the last search slice (the tree now)
	bool searchHasEvaluators = false;  // cost evaluators can be replaced
	TPose2D searchStartPose;
	std::vector<std::pair<const void*, size_t>> costmapSignature;
	// Since when the follower waits at the end of a partial path:
	std::optional<double> followerWaitingSince;
	// Since when the follower is blocked, and replans for that in a row
	// (publishing a path resets `restarts`, so they are counted apart):
	std::optional<double> followerBlockedSince;
	int blockedRestarts = 0;
	bool blockedReplanned = false;	// since the follower was last running
	// A search that ends with no path to follow (no motion possible from its
	// start pose, or no way with the current obstacles, which may be
	// transient: people, remembered obstacles) is retried a while later:
	constexpr double kRetryPeriod = 2.0;  // [s]
	int failedSearches = 0;	 // in a row
	std::optional<double> retryAt;
	// Progress toward the goal (see no_progress_timeout): the closest the
	// robot got to it in a straight line and, along the full path of the
	// current search, the shortest remaining length; and since when.
	constexpr double kMinProgress = 0.2;  // [m]
	double bestGoalDist = std::numeric_limits<double>::infinity();
	double bestPathDist = std::numeric_limits<double>::infinity();
	double tLastProgress = 0;
	double tLastHeartbeat = 0;
	// Obstacle points that blocked a path of this goal (see
	// blocked_obstacles_memory), by 5 cm cell: when, and where.
	std::unordered_map<int64_t, std::pair<double, TPoint2D>> blockedMemory;
	// Whether a limit of max_planning_restarts is exceeded by `n`:
	auto tooManyRestarts = [this](int n)
	{ return max_planning_restarts_ > 0 && n > max_planning_restarts_; };

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
	bool searchPublished = false;  // the current search published a path

	// The robot footprint, and the radius of its circumscribed circle:
	mrpt::math::TPolygon2D footprintPoly;
	double footprintRadius = 0;

	// Remembers the obstacle points around `at` that blocked a path:
	auto rememberBlocking =
		[&](const TPoint2D& at, const std::vector<mrpt::maps::CPointsMap::Ptr>& points)
	{
		if (blocked_obstacles_memory_ <= 0)
		{
			return;
		}
		constexpr double kRadius = 0.6;	 // [m]
		constexpr double kCell = 0.05;	// [m]
		const double t = now_seconds();
		for (const auto& pts : points)
		{
			const auto& xs = pts->getPointsBufferRef_x();
			const auto& ys = pts->getPointsBufferRef_y();
			for (size_t i = 0; i < xs.size(); i++)
			{
				if (std::hypot(xs[i] - at.x, ys[i] - at.y) > kRadius)
				{
					continue;
				}
				const auto ix = static_cast<int64_t>(std::floor(xs[i] / kCell));
				const auto iy = static_cast<int64_t>(std::floor(ys[i] / kCell));
				const int64_t key = static_cast<int64_t>(
					(static_cast<uint64_t>(ix) << 32) ^
					(static_cast<uint64_t>(iy) & 0xffffffffULL));
				blockedMemory[key] = {t, TPoint2D(xs[i], ys[i])};
			}
		}
	};

	// The remembered points still in memory, as an obstacle source (none if
	// empty), except those close to `start`, where the robot may be now:
	auto rememberedObstacles = [&](const TPose2D& start) -> mpp::ObstacleSource::Ptr
	{
		const double t = now_seconds();
		auto pts = mrpt::maps::CSimplePointsMap::Create();
		for (auto it = blockedMemory.begin(); it != blockedMemory.end();)
		{
			const auto& [t0, p] = it->second;
			if (t - t0 > blocked_obstacles_memory_)
			{
				it = blockedMemory.erase(it);
				continue;
			}
			if ((p - start.translation()).norm() > footprintRadius + 0.2)
			{
				pts->insertPointFast(static_cast<float>(p.x), static_cast<float>(p.y), 0);
			}
			++it;
		}
		if (pts->empty())
		{
			return {};
		}
		pts->mark_as_modified();
		return mpp::ObstacleSource::FromStaticPointcloud(pts);
	};

	// Starts a search from `from` (the robot pose, or the root, keeping the
	// committed part of the path before it):
	auto startSearch = [&](const TPose2D& from, double speed)
	{
		mpp::PlannerInput pi;
		pi.ptgs = get_thread_ptgs();
		pi.stateStart.pose = from;
		pi.stateStart.vel = {speed * std::cos(from.phi), speed * std::sin(from.phi), 0.0};
		pi.stateGoal.state = goal->pose;

		planner.costEvaluators_.clear();
		auto bbox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
		std::vector<mrpt::maps::CPointsMap::Ptr> startPoints;
		build_obstacles(from, pi.obstacles, &startPoints, &planner.costEvaluators_, bbox);
		if (auto mem = rememberedObstacles(from); mem)
		{
			pi.obstacles.push_back(mem);
		}
		publish_costmaps(planner.costEvaluators_);
		costmapSignature.clear();
		for (const auto& p : startPoints)
		{
			costmapSignature.emplace_back(p.get(), p->size());
		}
		searchStartPose = from;
		add_motion_cost_evaluators(planner.costEvaluators_, pi.ptgs);
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
		searchHasEvaluators = !planner.costEvaluators_.empty();
		searchJustStarted = true;
		searchPublished = false;
		searchFirstRevision = revision + 1;
		// (a pending retry would replace this search)
		retryAt.reset();
		tSearchStart = now_seconds();
		publishedTarget.reset();
		requestedTarget.reset();
		pathNodes.clear();
	};

	// Robot speed [m/s] (absolute), from its TF poses over the last fraction
	// of a second (0 if unknown):
	auto robotSpeed = [&]() -> double
	{
		try
		{
			const auto latest =
				tf_buffer_->lookupTransform(frame_id_map_, frame_id_robot_, tf2::TimePointZero);
			const auto t1 = tf2_ros::fromMsg(latest.header.stamp);
			const auto t0 = t1 - tf2::durationFromSec(0.3);
			const auto before = tf_buffer_->lookupTransform(frame_id_map_, frame_id_robot_, t0);
			const double dt = tf2::durationToSec(t1 - t0);
			const auto& a = before.transform.translation;
			const auto& b = latest.transform.translation;
			return dt > 0 ? std::hypot(b.x - a.x, b.y - a.y) / dt : 0.0;
		}
		catch (const tf2::TransformException&)
		{
			return 0.0;
		}
	};

	// Where a new search should start from, for a robot that keeps following
	// the current path meanwhile: the pose it will reach on that path after
	// start_lookahead_time (not beyond pose index `limitIdx`). The current
	// path up to there is kept as the committed part of the new one. If the
	// robot is not moving along the path, it is the robot pose (and the path
	// is dropped). Returns the pose and the robot speed.
	auto predictedStart = [&](size_t limitIdx) -> std::optional<std::pair<TPose2D, double>>
	{
		const auto rp = robotPose();
		if (!rp)
		{
			return std::nullopt;
		}
		const double speed = robotSpeed();
		constexpr double kMinSpeed = 0.1;  // [m/s]
		constexpr double kMaxDistToPath = 0.5;	// [m]
		if (start_lookahead_time_ > 0 && speed > kMinSpeed && pathPoses.size() > 1)
		{
			robotIdx = nearest_pose(pathPoses, rp->translation(), robotIdx);
			const bool onPath =
				(pathPoses[robotIdx].translation() - rp->translation()).norm() < kMaxDistToPath;
			const double sTarget = pathS[robotIdx] + speed * start_lookahead_time_;
			size_t idx = robotIdx;
			while (idx + 1 < pathPoses.size() && idx < limitIdx && pathS[idx] < sTarget)
			{
				idx++;
			}
			idx = std::min(idx, limitIdx);
			if (onPath && idx > robotIdx)
			{
				pathPoses.resize(idx + 1);
				pathS.resize(idx + 1);
				rootIdx = idx;
				RCLCPP_INFO(
					get_logger(),
					"Goal #%u: robot moving at %.2f m/s: the search starts %.1f m ahead on its "
					"path, at (%.2f, %.2f).",
					goal->id, speed, pathS[idx] - pathS[robotIdx], pathPoses[idx].x,
					pathPoses[idx].y);
				return std::make_pair(pathPoses[idx], speed);
			}
		}
		pathPoses.clear();
		pathS.clear();
		rootIdx = 0;
		robotIdx = 0;
		return std::make_pair(*rp, speed);
	};

	// Whether the robot is too close to the end of the committed path (the
	// search root) to go on from there: a fast robot would drive past it
	// before the new path is published (closer than where a search from the
	// robot would start, see predictedStart()).
	auto rootTooClose = [&]()
	{
		return rootIdx > robotIdx && rootIdx < pathS.size() &&
			   pathS[rootIdx] - pathS[robotIdx] < robotSpeed() * start_lookahead_time_;
	};

	// Restarts the search from scratch, from the robot (its predicted pose,
	// see predictedStart(), not beyond `limitIdx`: 0 for its current pose, if
	// it does not follow the path anymore) or from the root (keeping the
	// committed part). Returns false if there were too many restarts.
	auto restartSearch = [&](bool fromRobot, const char* why,
							 size_t limitIdx = std::numeric_limits<size_t>::max()) -> bool
	{
		followerWaitingSince.reset();
		restarts++;
		RCLCPP_WARN(
			get_logger(), "Goal #%u: restarting the search from the %s (#%d): %s", goal->id,
			fromRobot ? "robot pose" : "committed path end", restarts, why);
		if (tooManyRestarts(restarts))
		{
			publish_empty_paths();
			endGoal(NavStatus::STATUS_ERROR, "too many planning restarts");
			return false;
		}
		// The current path will be replaced:
		publishedFull = false;
		std::optional<std::pair<TPose2D, double>> from;
		if (fromRobot || pathPoses.empty() || rootIdx <= robotIdx)
		{
			from = predictedStart(limitIdx);
		}
		else
		{
			from = std::make_pair(pathPoses.at(rootIdx), robotSpeed());
		}
		if (!from)
		{
			publish_empty_paths();
			endGoal(NavStatus::STATUS_ERROR, "no robot pose");
			return false;
		}
		startSearch(from->first, from->second);
		return true;
	};

	// Publishes the path to `target` in the search output, after the
	// committed part:
	// Obstacles block the path if this close to the footprint border [m]. It
	// is negative (into the footprint: half its inner radius) to tolerate the
	// noise of sensed obstacles and the localization, since paths may run at
	// zero clearance. Obstacles really on the path get much deeper.
	double validationMargin = 0;
	// Newly refined edges are not checked by the search: they must keep the
	// footprint clear of obstacles.
	constexpr double kRefinedPathMargin = 0.0;

	// The current obstacles, indexed for footprint checks:
	auto obstacleIndex = [&](const std::vector<mrpt::maps::CPointsMap::Ptr>& points)
	{
		ObstacleIndex index(0.5);
		for (const auto& pts : points)
		{
			index.add(*pts);
		}
		return index;
	};

	// Publishes the waypoint sequence of the first `n` poses of the path:
	auto publishWaypoints = [&](size_t n, const rclcpp::Time& stamp)
	{
		mrpt_msgs::msg::WaypointSequence wps;
		wps.header.frame_id = frame_id_map_;
		wps.header.stamp = stamp;
		for (size_t i = 0; i < n; i++)
		{
			const bool last = i + 1 == n;
			auto& wp = wps.waypoints.emplace_back();
			wp.target = mrpt::ros2bridge::toROS_Pose(pathPoses[i]);
			wp.allowed_distance =
				last ? final_waypoint_allowed_distance_ : mid_waypoints_allowed_distance_;
			wp.allow_skip = last ? final_waypoint_allow_skip_ : mid_waypoints_allow_skip_;
			wp.ignore_heading =
				last ? final_waypoint_ignore_heading_ : mid_waypoints_ignore_heading_;
		}
		publish_waypoint_sequence(wps);
	};

	// Returns false (publishing nothing) if the new path hits the current
	// obstacles.
	// Rebuilds the obstacle-proximity costs when the obstacles changed: new
	// obstacles get their clearance cost and moved ones lose the old one.
	// (Edges already in the tree keep the cost they had.) Only if the search
	// has cost evaluators since its start, as it then keeps full edge paths.
	auto refreshCostmaps = [&](const std::vector<mrpt::maps::CPointsMap::Ptr>& points)
	{
		if (!searchHasEvaluators)
		{
			return;
		}
		std::vector<std::pair<const void*, size_t>> signature;
		for (const auto& p : points)
		{
			signature.emplace_back(p.get(), p->size());
		}
		if (signature == costmapSignature)
		{
			return;
		}
		costmapSignature = std::move(signature);

		std::vector<mpp::CostEvaluator::Ptr> evaluators;
		std::vector<mpp::ObstacleSource::Ptr> unusedSources;
		auto bbox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
		build_obstacles(searchStartPose, unusedSources, nullptr, &evaluators, bbox);
		// Keep the other evaluators (reverse motion, turning):
		for (const auto& ev : planner.costEvaluators_)
		{
			if (!std::dynamic_pointer_cast<mpp::CostEvaluatorCostMap>(ev))
			{
				evaluators.push_back(ev);
			}
		}
		planner.costEvaluators_ = std::move(evaluators);
		publish_costmaps(planner.costEvaluators_);
	};

	auto publishPath = [&](mpp::PlannerOutput& out, mrpt::graphs::TNodeID target,
						   bool partial) -> bool
	{
		auto [nodes, edgeList] = out.motionTree.backtrack_path(target);
		const auto requested = target;

		// The current obstacles, to check the new path:
		std::vector<mpp::ObstacleSource::Ptr> unusedSources;
		std::vector<mrpt::maps::CPointsMap::Ptr> points;
		{
			auto unusedBbox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
			build_obstacles(
				out.originalInput.stateStart.pose, unusedSources, &points, nullptr, unusedBbox);
		}
		const auto index = obstacleIndex(points);

		std::vector<mpp::MoveEdgeSE2_TPS*> edges;
		for (auto* e : edgeList)
		{
			if (e != nullptr)
			{
				edges.push_back(e);
			}
		}
		// A partial path leads toward the node closest to the goal, which may
		// be a dead end: publish only a prefix of limited length, so the robot
		// is not taken far along a wrong guess before the search finds out.
		if (partial && anytime_max_partial_length_ > 0 && edges.size() > 1)
		{
			double length = 0;
			size_t keep = 0;
			for (const auto* e : edges)
			{
				if (keep > 0 && length + e->ptgDist > anytime_max_partial_length_)
				{
					break;
				}
				length += e->ptgDist;
				keep++;
			}
			if (keep < edges.size())
			{
				edges.resize(keep);
				edgeList.resize(keep);
				nodes.resize(keep + 1);
				target = nodes.back().nodeID_;
			}
		}

		// The refinement does not check obstacles: use it only if the refined
		// path is still clear (the search edges are).
		if (!astar_skip_refine_)
		{
			std::vector<mpp::MoveEdgeSE2_TPS> certified;
			for (const auto* e : edges)
			{
				certified.push_back(*e);
			}
			std::vector<TPose2D> searchPoses = {out.originalInput.stateStart.pose};
			std::vector<size_t> unusedEnds;
			sample_edges(edges, out.originalInput.ptgs, searchPoses, unusedEnds);

			mpp::refine_trajectory(nodes, edgeList, out.originalInput.ptgs);

			std::vector<TPose2D> refinedPoses = {out.originalInput.stateStart.pose};
			unusedEnds.clear();
			sample_edges(edges, out.originalInput.ptgs, refinedPoses, unusedEnds);

			// The refined edges are not checked by the search. They must keep
			// the footprint clear of obstacles, except where the search path
			// itself runs at zero clearance (paths may do so in narrow places):
			// there, they must at least stay within the tolerance used to
			// validate published paths. Unrefined paths have small jumps
			// between edges, so they are only a fallback.
			bool refinedOk = !first_colliding_pose(
				refinedPoses, 0, index, footprintPoly, footprintRadius, kRefinedPathMargin);
			if (!refinedOk &&
				first_colliding_pose(
					searchPoses, 0, index, footprintPoly, footprintRadius, kRefinedPathMargin))
			{
				refinedOk = !first_colliding_pose(
					refinedPoses, 0, index, footprintPoly, footprintRadius, validationMargin);
			}
			if (!refinedOk)
			{
				RCLCPP_INFO(
					get_logger(), "Goal #%u: the refined path is not clear: using the search one.",
					goal->id);
				for (size_t i = 0; i < edges.size(); i++)
				{
					*edges[i] = certified[i];
				}
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
			if (newPoses.size() < 2)
			{
				// The search starts at the goal (its lattice cell): nothing
				// to drive.
				publish_empty_paths();
				endGoal(NavStatus::STATUS_SUCCESS, "the robot is at the goal");
				return true;
			}
			// End exactly at the goal (refinement may leave it slightly off):
			newPoses.back() = goal->pose;
		}

		// The search edges were checked when created, maybe against older
		// obstacles: check the new part against the current ones.
		if (const auto hit = first_colliding_pose(
				newPoses, std::max(newRootIdx, robotIdx > first ? robotIdx - first : 0), index,
				footprintPoly, footprintRadius, validationMargin);
			hit)
		{
			RCLCPP_INFO(
				get_logger(),
				"Goal #%u: the new %s path hits an obstacle at (%.2f, %.2f): not published.",
				goal->id, partial ? "partial" : "full", hit->second.x, hit->second.y);
			rememberBlocking(hit->second, points);
			return false;
		}

		pathNodes.clear();
		auto itNode = nodes.begin();
		for (size_t i = 0; i < edgeEnd.size() && itNode != nodes.end(); i++)
		{
			++itNode;
			if (itNode != nodes.end())
			{
				pathNodes.push_back({itNode->nodeID_, edgeEnd[i], itNode->pose});
			}
		}

		robotIdx -= std::min(robotIdx, first);
		rootIdx = newRootIdx;
		pathPoses = std::move(newPoses);
		pathS = arc_lengths(pathPoses);
		publishedTarget = target;
		requestedTarget = requested;
		published = true;
		if (!searchPublished)
		{
			// (progress along this search's paths is measured from here)
			bestPathDist = std::numeric_limits<double>::infinity();
		}
		searchPublished = true;
		failedSearches = 0;
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
		publishWaypoints(pathPoses.size(), stamp);

		pub_committed_path_->publish(to_path_msg(pathPoses, 0, rootIdx + 1, frame_id_map_, stamp));
		pub_tentative_path_->publish(
			to_path_msg(pathPoses, rootIdx, pathPoses.size(), frame_id_map_, stamp));

		RCLCPP_INFO(
			get_logger(),
			"Goal #%u: published %s path (rev. %u, %zu poses, %.1f m, %.1f s after goal)", goal->id,
			partial ? "PARTIAL" : "full", revision, pathPoses.size(), pathS.back(),
			now_seconds() - tGoalStart);
		return true;
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
		publishWaypoints(std::min(std::max<size_t>(idx, 2), pathPoses.size()), stamp);
		pub_tentative_path_->publish(to_path_msg({}, 0, 0, frame_id_map_, stamp));
		publishedFull = false;
	};

	while (true)
	{
		const double tCycleStart = now_seconds();

		// Events:
		std::optional<ActiveGoal> newGoal;
		bool cancel = false;
		std::string followerStatus;
		bool statusChanged = false;
		bool followerActiveSeen = false;
		bool followerHasId = false;
		uint32_t followerGoalId = 0;
		uint32_t followerRevision = 0;
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
			followerHasId = follower_status_has_id_;
			followerGoalId = follower_status_goal_id_;
			followerRevision = follower_status_revision_;
		}

		if (!plannerReady && ptgs_ready_)
		{
			planner.params_from_yaml(planner_params_yaml_);
			footprintPoly = mpp::robotShapeAsPolygon(ptgs_.robotShape);
			for (const auto& p : footprintPoly)
			{
				footprintRadius = std::max(footprintRadius, p.norm());
			}
			validationMargin = 0.5 * distance_to_footprint(footprintPoly, {0, 0});
			for (const auto& ptg : ptgs_.ptgs)
			{
				maxLinSpeed = std::max(maxLinSpeed, ptg->getMaxLinVel());
			}
			plannerReady = true;
		}

		if (newGoal)
		{
			const bool wasNavigating = goal.has_value();
			if (goal)
			{
				endGoal(NavStatus::STATUS_CANCELLED, "replaced by a new goal");
			}
			goal = std::move(newGoal);
			tGoalStart = now_seconds();
			restarts = 0;
			failedSearches = 0;
			retryAt.reset();
			blockedRestarts = 0;
			blockedReplanned = false;
			bestGoalDist = std::numeric_limits<double>::infinity();
			bestPathDist = std::numeric_limits<double>::infinity();
			tLastProgress = tGoalStart;
			blockedMemory.clear();
			revision = 0;
			published = false;
			publishedFull = false;
			followerEngaged = false;
			if (!wasNavigating)
			{
				// (the path of a goal being followed may be continued instead)
				pathPoses.clear();
				pathS.clear();
				rootIdx = 0;
				robotIdx = 0;
			}
			std::optional<mrpt::math::TPoint2D> goalHit;
			if (plannerReady)
			{
				// A goal pose in collision can not be reached: reject it now
				// instead of driving toward it with partial paths.
				std::vector<mpp::ObstacleSource::Ptr> unusedSources;
				std::vector<mrpt::maps::CPointsMap::Ptr> points;
				auto unusedBbox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
				build_obstacles(goal->pose, unusedSources, &points, nullptr, unusedBbox);
				goalHit =
					obstacleIndex(points).collides(goal->pose, footprintPoly, footprintRadius, 0.0);
			}
			if (!plannerReady)
			{
				endGoal(NavStatus::STATUS_ERROR, "PTGs not ready");
			}
			else if (goalHit)
			{
				RCLCPP_WARN(
					get_logger(),
					"Goal #%u: the robot footprint at the goal hits an obstacle at (%.2f, %.2f): "
					"rejected.",
					goal->id, goalHit->x, goalHit->y);
				publish_empty_paths();
				endGoal(NavStatus::STATUS_ERROR, "goal in collision");
			}
			else if (const auto from = predictedStart(std::numeric_limits<size_t>::max()); from)
			{
				startSearch(from->first, from->second);
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

				// Follower status: with statuses carrying the path they refer
				// to, only those for the latest path published for this goal
				// count (the previous path may be reported late, e.g. as
				// reached). Otherwise, act on terminal states only once the
				// follower is following one of our paths for this goal.
				const bool statusHasIds = sub_follower_status_ex_ != nullptr;
				const bool statusIsCurrent =
					!statusHasIds ||
					(followerHasId && followerGoalId == goal->id && followerRevision == revision);
				// Failures to follow a path of the current search also hold
				// for its later revisions (published meanwhile: they share
				// the committed part with it):
				const bool failureIsCurrent =
					!statusHasIds || (followerHasId && followerGoalId == goal->id &&
									  followerRevision >= searchFirstRevision);
				if (statusIsCurrent && (followerActiveSeen || followerStatus == FOLLOWER_RUNNING ||
										followerStatus == FOLLOWER_WAITING))
				{
					followerEngaged = published;
				}
				const bool isBlocked = followerStatus == FOLLOWER_BLOCKED;
				if ((statusHasIds || followerEngaged) && statusChanged)
				{
					if (statusIsCurrent && followerStatus == FOLLOWER_REACHED && publishedFull)
					{
						endGoal(NavStatus::STATUS_SUCCESS, "goal reached");
					}
					else if (statusIsCurrent && followerStatus == FOLLOWER_FAILED)
					{
						endGoal(NavStatus::STATUS_ERROR, "the path follower failed");
					}
					else if (
						failureIsCurrent &&
						((isBlocked && !blockedReplanned) || followerStatus == "OffPathExceeded" ||
						 followerStatus == "MissedGoal"))
					{
						// (a follower that stays blocked on the new path is
						// replanned for periodically, see below)
						followerEngaged = false;
						if (isBlocked)
						{
							blockedReplanned = true;
							followerBlockedSince = now_seconds();
						}
						// (the follower stopped following the path: from where
						// the robot is, not ahead on that path)
						restartSearch(true, followerStatus.c_str(), 0);
					}
				}

				// A follower waiting at the end of a partial path for too long:
				// the search is not extending it (stuck or blocked), so start
				// another one from the robot.
				// (a backstop: the search timeout normally restarts it first)
				const double kMaxWaitForPath =
					1.5 * planner.params_.maximumComputationTime + 5.0;	 // [s]
				if (goal && statusIsCurrent && followerStatus == FOLLOWER_WAITING && published)
				{
					if (!followerWaitingSince)
					{
						followerWaitingSince = now_seconds();
					}
					else if (now_seconds() - *followerWaitingSince > kMaxWaitForPath)
					{
						followerWaitingSince.reset();
						restartSearch(true, "the follower waits for a path");
					}
				}
				else
				{
					followerWaitingSince.reset();
				}

				// A follower blocked for long, while the path is still clear for
				// the search (e.g. it can not track a tight maneuver close to
				// obstacles): plan again from the robot, instead of waiting
				// forever (until no_progress_timeout). Its status may not change
				// again, so it is not required to be for the latest revision,
				// but for a path of the current search (not one that a search
				// started meanwhile is replacing).
				constexpr double kMaxBlocked = 5.0;	 // [s]
				const bool followerBlocked =
					goal && isBlocked && searchPublished && failureIsCurrent;
				if (followerBlocked)
				{
					if (!followerBlockedSince)
					{
						followerBlockedSince = now_seconds();
					}
					else if (now_seconds() - *followerBlockedSince > kMaxBlocked)
					{
						followerBlockedSince.reset();
						blockedRestarts++;
						if (tooManyRestarts(blockedRestarts))
						{
							publish_empty_paths();
							endGoal(NavStatus::STATUS_ERROR, "the path follower stays blocked");
						}
						else
						{
							restartSearch(true, "the follower stays blocked", 0);
						}
					}
				}
				else
				{
					followerBlockedSince.reset();
					if (goal && followerStatus == FOLLOWER_RUNNING)
					{
						blockedRestarts = 0;
						blockedReplanned = false;
					}
				}

				// Progress log, to diagnose searches that do not finish:
				if (goal && now_seconds() - tLastHeartbeat > 10.0)
				{
					tLastHeartbeat = now_seconds();
					RCLCPP_INFO(
						get_logger(),
						"Goal #%u: %.0f s after the goal, search %.0f s old, %s, follower: %s, "
						"search %s, last progress %.0f s ago.",
						goal->id, now_seconds() - tGoalStart, now_seconds() - tSearchStart,
						publishedFull ? "full path published"
									  : (published ? "partial path published" : "no path yet"),
						followerStatus.c_str(), planner.search_can_continue() ? "running" : "ended",
						now_seconds() - tLastProgress);
				}

				// Robot progress along the published path:
				if (goal && rp && !pathPoses.empty())
				{
					robotIdx = nearest_pose(pathPoses, rp->translation(), robotIdx);
				}

				// Progress toward the goal: give up after no_progress_timeout
				// without any. (Meanwhile, blocked paths and failed searches
				// are replanned.)
				if (goal && rp)
				{
					const double dGoal = (rp->translation() - goal->pose.translation()).norm();
					if (dGoal < bestGoalDist - kMinProgress)
					{
						bestGoalDist = dGoal;
						tLastProgress = now_seconds();
					}
					if (publishedFull && robotIdx < pathS.size())
					{
						const double dPath = pathS.back() - pathS[robotIdx];
						if (dPath < bestPathDist - kMinProgress)
						{
							// (not the first path of the search: the robot moved)
							if (std::isfinite(bestPathDist))
							{
								tLastProgress = now_seconds();
							}
							bestPathDist = dPath;
						}
					}
					if (no_progress_timeout_ > 0 &&
						now_seconds() - tLastProgress > no_progress_timeout_)
					{
						RCLCPP_WARN(
							get_logger(),
							"Goal #%u: no progress toward the goal for %.0f s (closest: %.2f m): "
							"giving up.",
							goal->id, no_progress_timeout_, bestGoalDist);
						publish_empty_paths();
						endGoal(NavStatus::STATUS_ERROR, "no progress toward the goal");
					}
				}

				// Current obstacles:
				std::vector<mpp::ObstacleSource::Ptr> obsSources;
				std::vector<mrpt::maps::CPointsMap::Ptr> obsPoints;
				if (goal && continuous_planning_)
				{
					auto unusedBbox = mrpt::math::TBoundingBoxf::PlusMinusInfinity();
					build_obstacles(
						rp ? *rp : goal->pose, obsSources, &obsPoints, nullptr, unusedBbox);
					// (for the search, not for validating paths: it avoids
					// them, and they may be gone)
					if (auto mem = rememberedObstacles(rp ? *rp : goal->pose); mem)
					{
						obsSources.push_back(mem);
					}
				}

				// Is the rest of the published path still clear?
				if (goal && continuous_planning_ && rp && robotIdx + 1 < pathPoses.size())
				{
					const auto hitInfo = first_colliding_pose(
						pathPoses, robotIdx + 1, obstacleIndex(obsPoints), footprintPoly,
						footprintRadius, validationMargin);
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
						rememberBlocking(hitInfo->second, obsPoints);
						// Only the part before the obstacle remains (and the
						// committed part, if it ends well before it, to go on from
						// its end). A committed end closer to it may already touch
						// it (the validation margin tolerates some overlap), and a
						// search from there would keep the robot that close.
						const bool fromRobot = *hit <= rootIdx || rootIdx <= robotIdx ||
											   rootIdx > stopIdx || rootTooClose();
						// A search from the committed path end: the robot must not
						// drive past it meanwhile, or the new path would start
						// behind the robot.
						publishTruncated((fromRobot ? stopIdx : rootIdx) + 1);
						const size_t keep = stopIdx + 1;
						if (keep < pathPoses.size())
						{
							pathPoses.resize(keep);
							pathS.resize(keep);
						}
						restartSearch(fromRobot, "path blocked", stopIdx);
					}
				}

				// Move the search root ahead of the robot on the path: the path
				// up to it is committed.
				// (only when the robot gets close to it, not every cycle: it would
				// move ahead of a stopped robot)
				const double sStitch = goal && !pathS.empty()
										   ? pathS[robotIdx] + stitch_lookahead_time_ * maxLinSpeed
										   : 0.0;
				if (goal && continuous_planning_ && published && rp && !pathNodes.empty() &&
					pathS[rootIdx] < sStitch)
				{
					// The first node far enough ahead, or else the last one (the
					// end of a short partial path, which the robot will drive
					// anyway), so the search never goes on from behind the robot:
					for (size_t k = 0; k < pathNodes.size(); k++)
					{
						if (pathS[pathNodes[k].poseIdx] < sStitch && k + 1 < pathNodes.size())
						{
							continue;
						}
						// The search may have rewired the node (a cheaper parent
						// and a new exact pose in its cell) after it was
						// published: the path would then jump at the root.
						const auto& nodesNow = lastOut.motionTree.nodes();
						const auto itNow = nodesNow.find(pathNodes[k].id);
						if (itNow == nodesNow.end() ||
							(itNow->second.pose.translation() - pathNodes[k].treePose.translation())
									.norm() > 1e-6 ||
							std::abs(mrpt::math::wrapToPi(
								itNow->second.pose.phi - pathNodes[k].treePose.phi)) > 1e-6)
						{
							RCLCPP_INFO(
								get_logger(),
								"Goal #%u: the node to re-root at changed in the search: waiting "
								"for the next path.",
								goal->id);
							break;
						}
						if (planner.reroot(pathNodes[k].id))
						{
							rootIdx = pathNodes[k].poseIdx;
							pathNodes.erase(pathNodes.begin(), pathNodes.begin() + k + 1);
							pub_committed_path_->publish(
								to_path_msg(pathPoses, 0, rootIdx + 1, frame_id_map_, this->now()));
							// Progress: the search time and restarts count from here.
							tSearchStart = now_seconds();
							restarts = 0;
						}
						break;
					}
				}

				// Retry a search that ended with no path to follow:
				if (goal && retryAt && now_seconds() >= *retryAt)
				{
					retryAt.reset();
					if (const auto from = predictedStart(std::numeric_limits<size_t>::max()); from)
					{
						startSearch(from->first, from->second);
					}
				}

				// Search:
				if (goal && planner.search_can_continue())
				{
					if (!obsSources.empty() && !searchJustStarted)
					{
						planner.set_obstacles(obsSources);
						refreshCostmaps(obsPoints);
					}
					searchJustStarted = false;
					const double sinceSearch = now_seconds() - tSearchStart;
					double budget =
						std::max(0.05, replan_period_ - (now_seconds() - tCycleStart) - 0.05);
					if (!searchPublished && anytime_first_path_time_ > 0)
					{
						// Until this search publishes a first path, search in
						// short slices to check for a partial path in time:
						constexpr double kAnytimeSlice = 0.1;  // [s]
						budget = std::min(
							budget, sinceSearch < anytime_first_path_time_
										? std::max(0.01, anytime_first_path_time_ - sinceSearch)
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
					bool restartedNow = false;
					if (out.success && out.goalNodeId)
					{
						if ((!publishedFull || publishedTarget != out.goalNodeId) &&
							!publishPath(out, *out.goalNodeId, false))
						{
							// (some edge hits the latest obstacles)
							restartSearch(true, "the new path is blocked");
							restartedNow = true;
						}
						else
						{
							// max_planning_restarts limits consecutive failures:
							restarts = 0;
						}
					}
					else if (
						!publishedFull && anytime_first_path_time_ > 0 &&
						now_seconds() - tSearchStart >= anytime_first_path_time_ &&
						out.bestNodeId && requestedTarget != out.bestNodeId)
					{
						// Best partial path, if it gets closer enough to the goal.
						// A full path is never replaced by a partial one: the
						// search may still be improving a solution it found.
						const auto& best = out.motionTree.nodes().at(*out.bestNodeId).pose;
						const auto& root = out.originalInput.stateStart.pose;
						const double dBest = (best.translation() - goal->pose.translation()).norm();
						const double dRoot = (root.translation() - goal->pose.translation()).norm();
						if (dBest <= dRoot - anytime_min_progress_ &&
							!publishPath(out, *out.bestNodeId, true))
						{
							// (the tree has edges checked against older
							// obstacles: search again with the current ones)
							restartSearch(true, "the new partial path is blocked");
							restartedNow = true;
						}
					}

					if (goal && !restartedNow && !planner.search_reached_goal() &&
						(!planner.search_can_continue() ||
						 searchTime > planner.params_.maximumComputationTime))
					{
						const char* why = planner.search_can_continue()
											  ? "search timeout"
											  : "no path in the search tree";
						// No motion at all from the start pose:
						const bool startBlocked =
							!planner.search_can_continue() && out.motionTree.nodes().size() <= 1;
						if (!planner.search_can_continue() && !searchPublished)
						{
							// Nothing to follow from this search: the obstacles
							// may change, so try again a while later.
							failedSearches++;
							const char* what = startBlocked
												   ? "no motion is possible from the start pose "
													 "(obstacles too close)"
												   : "no path found";
							if (tooManyRestarts(failedSearches))
							{
								RCLCPP_WARN(get_logger(), "Goal #%u: %s.", goal->id, what);
								publish_empty_paths();
								endGoal(NavStatus::STATUS_ERROR, "no path found");
							}
							else
							{
								RCLCPP_WARN(
									get_logger(), "Goal #%u: %s: retrying in %.0f s (#%d).",
									goal->id, what, kRetryPeriod, failedSearches);
								retryAt = now_seconds() + kRetryPeriod;
							}
						}
						else
						{
							restartSearch(true, why);
						}
					}
					lastOut = std::move(out);
				}
				else if (
					goal && published && !publishedFull && !planner.search_reached_goal() &&
					!retryAt)
				{
					// The search ended outside run(): re-rooting keeps only the
					// subtree of the new root, which may have no open nodes left.
					// Go on with a new search from the end of the committed path
					// (not driving past it meanwhile), or from the robot if it
					// is already close to it.
					const bool fromRobot = rootTooClose();
					if (!fromRobot && rootIdx > robotIdx)
					{
						publishTruncated(rootIdx + 1);
					}
					restartSearch(fromRobot, "no open nodes after re-rooting");
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
			const bool searchingFirst = goal && !searchPublished && planner.search_can_continue();
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
