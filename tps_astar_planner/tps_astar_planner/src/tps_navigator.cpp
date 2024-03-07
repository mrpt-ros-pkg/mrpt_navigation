/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#include <mrpt/core/lock_helper.h>
#include <mrpt/math/TSegment2D.h>
#include <mrpt/nav/reactive/CLogFileRecord.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/serialization/CArchive.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/version.h>
#include <selfdriving/algos/CostEvaluatorCostMap.h>
#include <selfdriving/algos/CostEvaluatorPreferredWaypoint.h>
#include <selfdriving/algos/refine_trajectory.h>
#include <selfdriving/algos/render_tree.h>
#include <selfdriving/algos/render_vehicle.h>
#include <selfdriving/algos/trajectories.h>
#include <selfdriving/algos/viz.h>
#include <selfdriving/ptgs/SpeedTrimmablePTG.h>
#include <tps_astar_nav_node/tps_navigator.h>

using namespace selfdriving;

constexpr double MIN_TIME_BETWEEN_POSE_UPDATES = 20e-3;  // [s]
constexpr double PREVIOUS_POSES_MAX_AGE        = 20;  // [s]

TPS_Navigator::~TPS_Navigator()
{
    // stop vehicle, etc.
}

void TPS_Navigator::Configuration::loadFrom(const mrpt::containers::yaml& c)
{
    MCP_LOAD_REQ(c, planner_bbox_margin);
    MCP_LOAD_REQ(c, enqueuedActionsToleranceXY);
    MCP_LOAD_REQ_DEG(c, enqueuedActionsTolerancePhi);
    MCP_LOAD_REQ(c, enqueuedActionsTimeoutMultiplier);
    MCP_LOAD_REQ(c, lookAheadImmediateCollisionChecking);

    MCP_LOAD_REQ(c, maxDistanceForTargetApproach);
    MCP_LOAD_REQ_DEG(c, maxRelativeHeadingForTargetApproach);

    MCP_LOAD_OPT(c, generateNavLogFiles);
    MCP_LOAD_OPT(c, navLogFilesPrefix);
}

mrpt::containers::yaml TPS_Navigator::Configuration::saveTo() const
{
    mrpt::containers::yaml c = mrpt::containers::yaml::Map();

    MCP_SAVE(c, planner_bbox_margin);
    MCP_SAVE(c, enqueuedActionsToleranceXY);
    MCP_SAVE_DEG(c, enqueuedActionsTolerancePhi);
    MCP_SAVE(c, enqueuedActionsTimeoutMultiplier);

    MCP_SAVE(c, maxDistanceForTargetApproach);
    MCP_SAVE_DEG(c, maxRelativeHeadingForTargetApproach);

    MCP_SAVE(c, lookAheadImmediateCollisionChecking);
    MCP_SAVE(c, generateNavLogFiles);
    MCP_SAVE(c, navLogFilesPrefix);

    return c;
}

void TPS_Navigator::initialize()
{
    MRPT_START
    auto lck = mrpt::lockHelper(navMtx_);

    // Check that config_ holds all the required fields:
    ASSERT_(config_.vehicleMotionInterface);
    ASSERT_(
        config_.globalMapObstacleSource || config_.localSensedObstacleSource);
    ASSERT_(config_.ptgs.initialized());
    ASSERT_(!config_.ptgs.ptgs.empty());

    // logging msgs (run only once even if initialize() is called several times)
    if (!loggerToNavlog_)
    {
        loggerToNavlog_ =
            [this](
                std::string_view                                    msg,
                [[maybe_unused]] const mrpt::system::VerbosityLevel level,
                [[maybe_unused]] std::string_view                   loggerName,
                [[maybe_unused]] const mrpt::Clock::time_point      timestamp) {
                using namespace std::string_literals;

                innerState_.navlogDebugMessages.push_back(
                    "["s + mrpt::typemeta::enum2str(level) + "] "s +
                    std::string(msg));
            };
        mrpt::system::COutputLogger::logRegisterCallback(loggerToNavlog_);
    }
    // Copy absolute speed limit from the first PTG:
    absoluteSpeedLimits_.robotMax_V_mps =
        config_.ptgs.ptgs.at(0)->getMaxLinVel();

    initialized_ = true;

    MRPT_END
}

void TPS_Navigator::request_navigation(const WaypointSequence& navRequest)
{
    MRPT_START
    auto lck = mrpt::lockHelper(navMtx_);
    ASSERTMSG_(initialized_, "requestNavigation() called before initialize()");

    const size_t N = navRequest.waypoints.size();
    ASSERTMSG_(N > 0, "List of waypoints is empty!");

    // reset fields to default:
    innerState_.clear();

    innerState_.waypointNavStatus.waypoints.resize(N);
    // Copy waypoints fields data, leave status fields to defaults:
    for (size_t i = 0; i < N; i++)
    {
        ASSERT_(navRequest.waypoints[i].isValid());
        innerState_.waypointNavStatus.waypoints[i].Waypoint::operator=(
            navRequest.waypoints[i]);
    }
    innerState_.waypointNavStatus.timestamp_nav_started = mrpt::Clock::now();

    // new state:
    navigationStatus_ = NavStatus::NAVIGATING;
    navErrorReason_   = NavErrorReason();

    MRPT_LOG_DEBUG_STREAM(
        "requestNavigation() called, navigation plan:\n"
        << innerState_.waypointNavStatus.getAsText());

    // The main loop navigation_step() will iterate over waypoints
    MRPT_END
}

void TPS_Navigator::navigation_step()
{
    auto lck = mrpt::lockHelper(navMtx_);

    ASSERTMSG_(initialized_, "navigation_step() called before initialize()");

    mrpt::system::CTimeLoggerEntry tle(navProfiler_, "navigation_step()");

    // Record execution period:
    auto& _ = innerState_;
    _.clearPerIterationData();
    {
        const double tNow = mrpt::Clock::nowDouble();
        if (_.lastNavigationStepEndTime)
            navProfiler_.registerUserMeasure(
                "navigationStep_period", tNow - *_.lastNavigationStepEndTime,
                true /*has time units*/);
        _.lastNavigationStepEndTime = tNow;
    }
    _.timStartThisNavStep = mrpt::Clock::nowDouble();
#if MRPT_VERSION >= 0x257
    // Save all debug msgs in navlog files, even if the user changed
    // the verbosity level for the regular terminal output:
    mrpt::system::COutputLogger::setVerbosityLevelForCallbacks(
        mrpt::system::LVL_DEBUG);
#endif

    const NavStatus prevState = navigationStatus_;
    switch (navigationStatus_)
    {
        case NavStatus::IDLE:
        case NavStatus::SUSPENDED:
            if (lastNavigationState_ == NavStatus::NAVIGATING)
            {
                MRPT_LOG_INFO(
                    "TPS_Navigator::navigation_step(): Navigation "
                    "stopped.");
            }
            break;

        case NavStatus::NAV_ERROR:
            // Send end-of-navigation event:
            if (lastNavigationState_ == NavStatus::NAVIGATING &&
                navigationStatus_ == NavStatus::NAV_ERROR)
            {
                pendingEvents_.emplace_back([this]() {
                    ASSERT_(config_.vehicleMotionInterface);
                    config_.vehicleMotionInterface->on_nav_end_due_to_error();
                });
            }

            // If we just arrived at this state, stop the robot:
            if (lastNavigationState_ == NavStatus::NAVIGATING)
            {
                MRPT_LOG_ERROR(
                    "[TPS_Navigator::navigation_step()] Stopping "
                    "navigation "
                    "due to a NavStatus::NAV_ERROR state!");

                if (config_.vehicleMotionInterface)
                {
                    config_.vehicleMotionInterface->stop(STOP_TYPE::REGULAR);
                    config_.vehicleMotionInterface->stop_watchdog();
                }
            }
            break;

        case NavStatus::NAVIGATING:
            try
            {
                impl_navigation_step();
            }
            catch (const std::exception& e)
            {
                navigationStatus_ = NavStatus::NAV_ERROR;
                if (navErrorReason_.error_code == NavError::NONE)
                {
                    navErrorReason_.error_code = NavError::OTHER;
                    navErrorReason_.error_msg =
                        std::string("Exception: ") + std::string(e.what());
                }
                MRPT_LOG_ERROR_FMT(
                    "[TPS_Navigator::navigationStep] Exception:\n %s",
                    e.what());
            }
            break;
    };

    lastNavigationState_ = prevState;

    dispatch_pending_nav_events();
}

void TPS_Navigator::cancel()
{
    auto lck = mrpt::lockHelper(navMtx_);
    ASSERTMSG_(initialized_, "cancel() called before initialize()");

    MRPT_LOG_DEBUG("TPS_Navigator::cancel() called.");
    navigationStatus_ = NavStatus::IDLE;
    innerState_.active_plan_reset(true);

    if (config_.vehicleMotionInterface)
    {
        config_.vehicleMotionInterface->stop(STOP_TYPE::REGULAR);
        config_.vehicleMotionInterface->stop_watchdog();
    }
}
void TPS_Navigator::resume()
{
    auto lck = mrpt::lockHelper(navMtx_);
    ASSERTMSG_(initialized_, "resume() called before initialize()");

    MRPT_LOG_DEBUG("TPS_Navigator::resume() called.");

    if (navigationStatus_ == NavStatus::SUSPENDED)
        navigationStatus_ = NavStatus::NAVIGATING;
}
void TPS_Navigator::suspend()
{
    auto lck = mrpt::lockHelper(navMtx_);
    ASSERTMSG_(initialized_, "suspend() called before initialize()");

    MRPT_LOG_DEBUG("TPS_Navigator::suspend() called.");

    if (navigationStatus_ == NavStatus::NAVIGATING)
    {
        navigationStatus_ = NavStatus::SUSPENDED;
        innerState_.active_plan_reset(true);

        if (config_.vehicleMotionInterface)
        {
            config_.vehicleMotionInterface->stop(STOP_TYPE::REGULAR);
            config_.vehicleMotionInterface->stop_watchdog();
        }
    }
}

void TPS_Navigator::reset_nav_error()
{
    auto lck = mrpt::lockHelper(navMtx_);
    ASSERTMSG_(initialized_, "resetNavError() called before initialize()");

    if (navigationStatus_ == NavStatus::NAV_ERROR)
    {
        navigationStatus_ = NavStatus::IDLE;
        navErrorReason_   = NavErrorReason();
    }
}

WaypointStatusSequence TPS_Navigator::waypoint_nav_status() const
{
    // Make sure the data structure is not under modification:
    auto lck = mrpt::lockHelper(navMtx_);

    WaypointStatusSequence ret = innerState_.waypointNavStatus;
    return ret;
}

void TPS_Navigator::dispatch_pending_nav_events()
{
    // Invoke pending events:
    for (auto& ev : pendingEvents_)
    {
        try
        {
            ev();
        }
        catch (const std::exception& e)
        {
            MRPT_LOG_ERROR_STREAM("Exception in event handler: " << e.what());
        }
    }
    pendingEvents_.clear();
}

void TPS_Navigator::update_robot_kinematic_state()
{
    // Ignore calls too-close in time, e.g. from the navigation_step()
    // methods of AbstractNavigator and a derived, overriding class.

    // this is clockwall time for real robots, simulated time in simulators.
    const double robotTime = config_.vehicleMotionInterface->robot_time();

    if (lastVehiclePosRobotTime_ >= .0)
    {
        const double lastCallAge = robotTime - lastVehiclePosRobotTime_;
        if (lastCallAge < MIN_TIME_BETWEEN_POSE_UPDATES)
        {
            MRPT_LOG_THROTTLE_DEBUG_FMT(
                5.0,
                "updateCurrentPoseAndSpeeds: ignoring call, since last "
                "call "
                "was only %f ms ago.",
                lastCallAge * 1e3);
            // previous data is still valid: don't query the robot again
            return;
        }
    }

    {
        mrpt::system::CTimeLoggerEntry tle(
            navProfiler_, "updateCurrentPoseAndSpeeds()");

        lastVehicleLocalization_ =
            config_.vehicleMotionInterface->get_localization();

        lastVehicleOdometry_ = config_.vehicleMotionInterface->get_odometry();

        if (!lastVehicleLocalization_.valid)
        {
            navigationStatus_          = NavStatus::NAV_ERROR;
            navErrorReason_.error_code = NavError::EMERGENCY_STOP;
            navErrorReason_.error_msg  = std::string(
                "ERROR: get_localization() failed, stopping robot "
                "and finishing navigation");
            try
            {
                config_.vehicleMotionInterface->stop(STOP_TYPE::EMERGENCY);
            }
            catch (...)
            {
            }
            MRPT_LOG_ERROR(navErrorReason_.error_msg);
            throw std::runtime_error(navErrorReason_.error_msg);
        }
    }
    lastVehiclePosRobotTime_ = robotTime;

    MRPT_LOG_THROTTLE_DEBUG_STREAM(
        1.0,
        "updateCurrentPoseAndSpeeds:"
        "\nLocalization="
            << lastVehicleLocalization_.pose << "\n Odometry    : "
            << lastVehicleOdometry_.odometry << "\n Odometry vel Local: "
            << lastVehicleOdometry_.odometryVelocityLocal.asString()
            << "\n Odometry vel global: "
            << lastVehicleOdometry_.odometryVelocityLocal
                   .rotated(lastVehicleOdometry_.odometry.phi)
                   .asString());

    // TODO: Detect a change if frame_id and clear m_latestPoses,
    // m_latestOdomPoses.

    // Append to list of past poses:
    innerState_.latestPoses.insert(
        lastVehicleLocalization_.timestamp, lastVehicleLocalization_.pose);
    innerState_.latestOdomPoses.insert(
        lastVehicleOdometry_.timestamp, lastVehicleOdometry_.odometry);

    // Purge old ones:
    while (innerState_.latestPoses.size() > 1 &&
           mrpt::system::timeDifference(
               innerState_.latestPoses.begin()->first,
               innerState_.latestPoses.rbegin()->first) >
               PREVIOUS_POSES_MAX_AGE)
    { innerState_.latestPoses.erase(innerState_.latestPoses.begin()); }
    while (innerState_.latestOdomPoses.size() > 1 &&
           mrpt::system::timeDifference(
               innerState_.latestOdomPoses.begin()->first,
               innerState_.latestOdomPoses.rbegin()->first) >
               PREVIOUS_POSES_MAX_AGE)
    {
        innerState_.latestOdomPoses.erase(innerState_.latestOdomPoses.begin());
    }
}

void TPS_Navigator::impl_navigation_step()
{
    mrpt::system::CTimeLoggerEntry tle(navProfiler_, "impl_navigation_step");
    MRPT_LOG_DEBUG_STREAM("Enter impl Navigation Step");
    if (lastNavigationState_ != NavStatus::NAVIGATING)
    {
        internal_on_start_new_navigation();
        MRPT_LOG_DEBUG_STREAM("navigation step: start new navigation");
    }
    MRPT_LOG_DEBUG_STREAM("Check if All waypoints are done");
    // navigation ended?
    if (check_all_waypoints_are_done())
    {
        MRPT_LOG_INFO("All waypoints reached, status NAVIGATING -> IDLE.");
        navigationStatus_ = NavStatus::IDLE;
        innerState_.active_plan_reset();
        if (config_.vehicleMotionInterface)
        {
            config_.vehicleMotionInterface->stop(STOP_TYPE::REGULAR);
            config_.vehicleMotionInterface->stop_watchdog();
        }

        pendingEvents_.emplace_back(
            [this]() { config_.vehicleMotionInterface->on_nav_end(); });

        return;
    }

    // Get current robot kinematic state:
    update_robot_kinematic_state();
    MRPT_LOG_DEBUG_STREAM("Kinematic state update complete");
    // Check for immediate collisions:
    check_immediate_collision();
    MRPT_LOG_DEBUG_STREAM("Checking for Collision complete");
    // Checks whether we need to launch a new A* path planner:
    check_have_to_replan();
    MRPT_LOG_DEBUG_STREAM("Checking for Replan");

    // Checks whether the A* planner finished and we have to send a new active
    // trajectory to the path tracker:
    check_new_planner_output();
    MRPT_LOG_DEBUG_STREAM("Check for new planner output");

    // Check if the target seems to be at reach, but it's clearly
    // occupied by obstacles:
    // TODO... here?
    // m_counter_check_target_is_blocked = 0;

    // Send actual motion command, if needed, or a NOP if we are safely on track
    send_next_motion_cmd_or_nop();
    MRPT_LOG_DEBUG_STREAM("Send command or NOP");

    send_current_state_to_viz_and_navlog();  // optional debug viz
    internal_write_to_navlog_file();  // optional debug output file
    MRPT_LOG_DEBUG_STREAM("navigation step impl complete");
}

void TPS_Navigator::internal_on_start_new_navigation()
{
    ASSERT_(config_.vehicleMotionInterface);

    MRPT_LOG_INFO("Starting navigation. Watchdog enabled.");

    config_.vehicleMotionInterface->start_watchdog(1000 /*ms*/);

    // Have we just started the navigation?
    if (lastNavigationState_ == NavStatus::IDLE)
    {
        pendingEvents_.emplace_back([this]() {
            ASSERT_(config_.vehicleMotionInterface);
            config_.vehicleMotionInterface->on_nav_start();
        });

        // Start a new navlog file?
        internal_start_navlog_file();
    }
}

void TPS_Navigator::check_immediate_collision()
{
    mrpt::system::CTimeLoggerEntry tle(
        navProfiler_, "impl_navigation_step.check_immediate_collision");

    const unsigned int NUM_STEPS             = 3;
    const unsigned int NUM_CLOSEST_OBSTACLES = 40;

    auto& _ = innerState_;

    if (!config_.localSensedObstacleSource) return;

    auto obs = config_.localSensedObstacleSource->obstacles();

    if (!obs || obs->empty()) return;

    // Extrapolate the current motion into the future:
    const auto globalPos = lastVehicleLocalization_.pose;
    const auto localVel  = lastVehicleOdometry_.odometryVelocityLocal;

    const auto& xs = obs->getPointsBufferRef_x();
    const auto& ys = obs->getPointsBufferRef_y();

    _.collisionCheckingPosePrediction =
        globalPos + localVel * config_.lookAheadImmediateCollisionChecking;

    bool collision = false;

    for (unsigned int i = 0; i < NUM_STEPS && !collision; i++)
    {
        const double dt = (static_cast<double>(i) / (NUM_STEPS - 1)) *
                          config_.lookAheadImmediateCollisionChecking;

        const auto predictedPose = globalPos + localVel * dt;

        for (const auto& ptg : config_.ptgs.ptgs)
        {
            std::vector<size_t> idxs;
            std::vector<float>  distSq;
            obs->kdTreeNClosestPoint2DIdx(
                predictedPose.x, predictedPose.y, NUM_CLOSEST_OBSTACLES, idxs,
                distSq);

            for (size_t ptIdx : idxs)
            {
                const auto localPt =
                    predictedPose.inverseComposePoint({xs[ptIdx], ys[ptIdx]});
                const bool collide =
                    ptg->isPointInsideRobotShape(localPt.x, localPt.y);

                if (collide) collision = true;
            }
            if (collision) break;
        }
    }

    if (collision)
    {
        MRPT_LOG_WARN_STREAM("Collision predicted ahead! Stopping.");

        config_.vehicleMotionInterface->stop(STOP_TYPE::EMERGENCY);

        // clear path and recompute:
        _.active_plan_reset(true);

        // user callbacks:
        pendingEvents_.emplace_back([this]() {
            config_.vehicleMotionInterface->on_apparent_collision();
        });
    }
}

void TPS_Navigator::check_have_to_replan()
{
    auto& _ = innerState_;

    // We don't have yet neither a running or under-planning path:
    if (!_.pathPlannerTargetWpIdx.has_value())
    {
        // find next target wp:
        auto nextWp = find_next_waypoint_for_planner();

        // Start from the current pose, plus the motion delta if we are already
        // moving (ideally we should be still while planning...):
        selfdriving::SE2_KinState startingFrom;

        const double deltaTime =
            std::min(1.0, config_.plannerParams.maximumComputationTime);

        startingFrom.pose =
            lastVehicleLocalization_.pose +
            lastVehicleOdometry_.odometryVelocityLocal * deltaTime;
        startingFrom.vel = lastVehicleOdometry_.odometryVelocityLocal.rotated(
            startingFrom.pose.phi);

        // (this will fill in pathPlannerTargetWpIdx):
        enqueue_path_planner_towards(nextWp, startingFrom);
        return;
    }

    // Are we in the middle of a path tracking but the current motion under
    // execution is not yet the one taking us to the final waypoint?
    // Then, keep refining the path planning, launching new path planning tasks
    // starting from the next predicted motion command node:
    if (_.activePlanEdgeSentIndex.has_value() &&
        !_.pathPlannerFuture.valid() &&  // not already running!
        (!_.activePlanOutput.po.success ||
         (_.activePlanOutput.po.success &&  // the plan reached the final wp
          *_.activePlanEdgeSentIndex < _.activePlanPathEdges.size())))
    {
        MRPT_LOG_INFO_STREAM(
            "Launching a path continuation planning from edge #"
            << *_.activePlanEdgeSentIndex
            << " < |activePlanPathEdges|=" << _.activePlanPathEdges.size());

        // find next target wp:
        auto nextWp = find_next_waypoint_for_planner();

        // Start from the current pose, plus the motion delta if we are already
        // moving (ideally we should be still while planning...):
        selfdriving::SE2_KinState startingFrom;

        // Take the next node after the current under-execution motion edge:
        const auto& nextNode =
            _.activePlanPath.at(*_.activePlanEdgeSentIndex + 1);

        startingFrom.pose = nextNode.pose;
        startingFrom.vel  = nextNode.vel;

        // (this will fill in pathPlannerTargetWpIdx):
        enqueue_path_planner_towards(nextWp, startingFrom, nextNode.nodeID_);
    }
}

waypoint_idx_t TPS_Navigator::find_next_waypoint_for_planner()
{
    mrpt::system::CTimeLoggerEntry tle(
        navProfiler_, "impl_navigation_step.find_next_waypoint_for_planner");

    auto& _ = innerState_;

    ASSERT_(!_.waypointNavStatus.waypoints.empty());
    const auto& wps = _.waypointNavStatus.waypoints;

    std::optional<waypoint_idx_t> firstWpIdx;

    for (waypoint_idx_t i = 0; i < wps.size(); i++)
    {
        const auto& wp = wps.at(i);
        if (wp.reached) continue;

        firstWpIdx = i;

        if (!wp.allowSkip) break;
    }
    ASSERT_(firstWpIdx.has_value());

    // Raise a warning if the wp is the last one and has not a speed of zero,
    // i.e. the vehicle will keep moving afterwards. It might be desired by the
    // user, so do not abort/error but at least emit a warning:
    if (const auto& wp = wps.at(*firstWpIdx);
        *firstWpIdx + 1 == wps.size() && wp.speedRatio != 0)
    {
        MRPT_LOG_WARN_STREAM(
            "Selecting *last* waypoint #"
            << (*firstWpIdx + 1)
            << " which does not have a final speed of zero: the vehicle will "
               "not stop there. Waypoint: "
            << wp.getAsText());
    }

    return *firstWpIdx;
}

TPS_Navigator::PathPlannerOutput TPS_Navigator::path_planner_function(
    TPS_Navigator::PathPlannerInput ppi)
{
    mrpt::system::CTimeLoggerEntry tle(navProfiler_, "path_planner_function");

    const double BBOX_MARGIN = config_.planner_bbox_margin;  // [meters]

    mrpt::math::TBoundingBoxf bbox;

    // Make sure goal and start are within bbox:
    {
        const auto bboxMargin =
            mrpt::math::TPoint3Df(BBOX_MARGIN, BBOX_MARGIN, .0);
        const auto ptStart = mrpt::math::TPoint3Df(
            ppi.pi.stateStart.pose.x, ppi.pi.stateStart.pose.y, 0);
        const auto ptGoal = mrpt::math::TPoint3Df(
            ppi.pi.stateGoal.asSE2KinState().pose.x,
            ppi.pi.stateGoal.asSE2KinState().pose.y, 0);

        bbox.min = ptStart;
        bbox.max = ptStart;
        bbox.updateWithPoint(ptStart - bboxMargin);
        bbox.updateWithPoint(ptStart + bboxMargin);
        bbox.updateWithPoint(ptGoal - bboxMargin);
        bbox.updateWithPoint(ptGoal + bboxMargin);
    }

    ppi.pi.worldBboxMax = {bbox.max.x, bbox.max.y, M_PI};
    ppi.pi.worldBboxMin = {bbox.min.x, bbox.min.y, -M_PI};

    MRPT_LOG_DEBUG_STREAM(
        "[path_planner_function] Start state: "
        << ppi.pi.stateStart.asString());
    MRPT_LOG_DEBUG_STREAM(
        "[path_planner_function] Goal state : " << ppi.pi.stateGoal.asString());
    MRPT_LOG_DEBUG_STREAM(
        "[path_planner_function] World bbox: "
        << ppi.pi.worldBboxMin.asString() << " - "
        << ppi.pi.worldBboxMax.asString());

    MRPT_LOG_DEBUG_STREAM(
        "[path_planner_function] Using VehicleInterface class: "
        << config_.vehicleMotionInterface->GetRuntimeClass()->className);

    // Do the path planning :
    selfdriving::TPS_Astar planner;

    // time profiler:
    planner.attachExternalProfiler_(navProfiler_);

    // ~~~~~~~~~~~~~~
    // Add cost maps
    // ~~~~~~~~~~~~~~
    // TODO: Make static list instead of recreating each time?
    planner.costEvaluators_.clear();

    // cost map: prefer to go thru waypoints
    // =========================================
    {
        std::vector<mrpt::math::TPoint2D> lstPts;
        for (const auto& wp : innerState_.waypointNavStatus.waypoints)
        {
            if (wp.reached) continue;
            lstPts.emplace_back(wp.target);
        }

        if (!lstPts.empty())
        {
            auto cmWps = selfdriving::CostEvaluatorPreferredWaypoint::Create();
            cmWps->params_ = config_.preferWaypointsParameters;
            cmWps->setPreferredWaypoints(lstPts);

            planner.costEvaluators_.push_back(cmWps);
        }
    }

    // cost maps: from obstacles
    // ============================

    if (config_.globalMapObstacleSource)
    {
        if (auto obs = config_.globalMapObstacleSource->obstacles();
            obs && !obs->empty())
        {
            planner.costEvaluators_.push_back(
                selfdriving::CostEvaluatorCostMap::FromStaticPointObstacles(
                    *obs, config_.globalCostParameters,
                    ppi.pi.stateStart.pose));
        }
    }

    if (config_.localSensedObstacleSource)
    {
        if (auto obs = config_.localSensedObstacleSource->obstacles();
            obs && !obs->empty())
        {
            planner.costEvaluators_.push_back(
                selfdriving::CostEvaluatorCostMap::FromStaticPointObstacles(
                    *obs, config_.localCostParameters, ppi.pi.stateStart.pose));
        }
    }

    // ~~~~~~~~~~~~~~~~~~
    // Obstacles sources
    // ~~~~~~~~~~~~~~~~~~
    if (config_.globalMapObstacleSource)
        ppi.pi.obstacles.push_back(config_.globalMapObstacleSource);

    if (config_.localSensedObstacleSource)
        ppi.pi.obstacles.push_back(config_.localSensedObstacleSource);

    // verbosity level:
    planner.setMinLoggingLevel(this->getMinLoggingLevel());

    planner.params_ = config_.plannerParams;
    {
        std::stringstream ss;
        planner.params_.as_yaml().printAsYAML(ss);
        MRPT_LOG_DEBUG_STREAM(
            "[path_planner_function] A* planner parameters:\n"
            << ss.str());
    }

    // PTGs:
    ppi.pi.ptgs = config_.ptgs;

    // Insert custom progress callback for the GUI, if enabled:
    planner.progressCallback_ = [this](const ProgressCallbackData& pcd) {
        MRPT_LOG_DEBUG_STREAM(
            "[progressCallback] bestCostFromStart: "
            << pcd.bestCostFromStart
            << " bestCostToGoal: " << pcd.bestCostToGoal
            << " bestPathLength: " << pcd.bestPath.size());

        if (config_.vizSceneToModify || navlog_output_file_.has_value())
        {
            ASSERT_(pcd.tree);
            ASSERT_(pcd.originalPlanInput);
            ASSERT_(pcd.costEvaluators);

            send_path_to_viz_and_navlog(
                *pcd.tree, pcd.bestFinalNode, *pcd.originalPlanInput,
                *pcd.costEvaluators);
        }
    };

    mrpt::system::CTimeLoggerEntry tle2(
        navProfiler_, "path_planner_function.a_star");

    // ========== ACTUAL A* PLANNING ================
    PathPlannerOutput ret;
    ret.po = planner.plan(ppi.pi);
    // ================================================

    tle2.stop();

    // Keep a copy of the costs, for reference of the caller,
    // visualization,...
    ret.costEvaluators = planner.costEvaluators_;

    ret.startingFromCurrentPlanNode     = ppi.startingFromCurrentPlanNode;
    ret.startingFromCurrentPlanNodePose = ppi.startingFromCurrentPlanNodePose;

    return ret;
}

void TPS_Navigator::enqueue_path_planner_towards(
    const waypoint_idx_t             targetWpIdx,
    const selfdriving::SE2_KinState& startingFrom,
    const std::optional<TNodeID>&    startingFromNodeID)
{
    auto& _ = innerState_;

    MRPT_LOG_DEBUG_STREAM(
        "enqueue_path_planner_towards() called with targetWpIdx="
        << targetWpIdx << " startingFrom: " << startingFrom.asString());

    // ----------------------------------
    // prepare planner request:
    // ----------------------------------
    PathPlannerInput ppi;

    // Starting pose and velocity:
    // ---------------------------------------------------
    ppi.pi.stateStart = startingFrom;

    ASSERT_LT_(targetWpIdx, _.waypointNavStatus.waypoints.size());
    const auto& wp = _.waypointNavStatus.waypoints.at(targetWpIdx);

    // waypoint => pose or point:
    if (wp.targetHeading.has_value())
    {
        ppi.pi.stateGoal.state = mrpt::math::TPose2D(
            wp.target.x, wp.target.y, wp.targetHeading.value());
    }
    else
    {
        ppi.pi.stateGoal.state = mrpt::math::TPoint2D(wp.target.x, wp.target.y);
    }

    // save optional start node ID:
    ppi.startingFromCurrentPlanNode     = startingFromNodeID;
    ppi.startingFromCurrentPlanNodePose = startingFrom.pose;

    // speed at target:
    // ppi.pi.stateGoal.vel
    MRPT_TODO("Handle speed at target waypoint");

    // ----------------------------------
    // send it for running of the worker thread:
    // ----------------------------------
    _.pathPlannerFuture =
        pathPlannerPool_.enqueue(&TPS_Navigator::path_planner_function, this, ppi);
    _.pathPlannerTargetWpIdx = targetWpIdx;
}

void TPS_Navigator::check_new_planner_output()
{
    auto& _ = innerState_;

    if (!_.pathPlannerFuture.valid()) return;

    if (std::future_status::ready !=
        _.pathPlannerFuture.wait_for(std::chrono::milliseconds(0)))
        return;

    const auto result   = _.pathPlannerFuture.get();
    _.pathPlannerFuture = std::future<PathPlannerOutput>();  // Reset

    // Is the result obsolete because we have already moved on to a new motion
    // edge while planning this refining planning?
    if (result.startingFromCurrentPlanNode.has_value())
    {
        bool isObsolete = false;

        if (!_.activePlanEdgeSentIndex.has_value() ||
            *_.activePlanEdgeSentIndex + 1 > _.activePlanPath.size() - 1)
        {  //
            isObsolete = true;
        }
        else
        {
            const auto newNextNodeId =
                _.activePlanPath.at(*_.activePlanEdgeSentIndex + 1).nodeID_;
            const auto initialNextNode = *result.startingFromCurrentPlanNode;

            const auto& newNextNodePose =
                _.activePlanPath.at(*_.activePlanEdgeSentIndex + 1).pose;
            const auto& initialNextNodePose =
                *result.startingFromCurrentPlanNodePose;

            isObsolete =
                (newNextNodeId != initialNextNode) ||
                (initialNextNodePose - newNextNodePose).translation().norm() >
                    config_.plannerParams.grid_resolution_xy;
        }

        if (isObsolete)
        {
            MRPT_LOG_INFO(
                "[check_new_planner_output] Discarding refining path plan "
                "since it is now obsolete.");
            return;
        }
    }

    if (!result.po.success)
    {
        MRPT_LOG_INFO(
            "A* did not complete a plan towards the target, it only had time "
            "for a partial solution");
    }

    if (config_.vizSceneToModify || navlog_output_file_)
        send_planner_output_to_viz(result);

    // Merge or overwrite current plan:
    if (result.startingFromCurrentPlanNode.has_value())
    { merge_new_plan_if_better(result); }
    else
    {
        MRPT_LOG_INFO_STREAM("Taking new path planning result.");

        // first path planning, just copy it:
        _.activePlanOutput = std::move(result);
        _.active_plan_reset();

        auto [path, edges] = _.activePlanOutput.po.motionTree.backtrack_path(
            *_.activePlanOutput.po.bestNodeId);

        // Correct PTG arguments according to the final actual poses.
        // Needed to correct for lattice approximations:
        refine_trajectory(path, edges, config_.ptgs);

        // std::list -> std::vector for convenience:
        _.activePlanPath.clear();
        for (const auto& node : path) _.activePlanPath.push_back(node);

        // std::list -> std::vector for convenience:
        _.activePlanPathEdges.clear();
        for (const auto& edge : edges) _.activePlanPathEdges.push_back(*edge);


    }

    if (this->getMinLoggingLevel() == mrpt::system::LVL_DEBUG)
    {
        MRPT_LOG_DEBUG_STREAM("[check_new_planner_output] New path nodes:");
        for (const auto& step : _.activePlanPath)
            MRPT_LOG_DEBUG_STREAM(
                "[check_new_planner_output] " << step.asString());

        MRPT_LOG_DEBUG_STREAM("[check_new_planner_output] New path edges:");
        for (const auto& edge : _.activePlanPathEdges)
            MRPT_LOG_DEBUG_STREAM(
                "[check_new_planner_output]\n"
                << edge.asString());
    }

    // sanity check
    ASSERT_EQUAL_(_.activePlanPath.size(), _.activePlanPathEdges.size() + 1);
}

void TPS_Navigator::send_next_motion_cmd_or_nop()
{
    mrpt::system::CTimeLoggerEntry tle(
        navProfiler_, "impl_navigation_step.send_next_motion_cmd_or_nop");

    using namespace mrpt;  // "_deg"

    auto& _ = innerState_;

    // Special motions near a final waypoint?
    if (approach_target_controller()) return;

    // No active path planning?
    if (_.activePlanPath.empty()) return;

    // Following error due to enqueued motion time out?
    if (config_.vehicleMotionInterface->enqeued_motion_timed_out())
    {
        // stop the robot, report, and try to recover:
        if (config_.vehicleMotionInterface)
            config_.vehicleMotionInterface->stop(STOP_TYPE::EMERGENCY);

        MRPT_LOG_ERROR(
            "Enqueued motion primitive did not triggered (TIMEOUT). Stopping "
            "the robot and replanning.");

        innerState_.active_plan_reset();

        return;
    }

    MRPT_LOG_DEBUG_STREAM(
        "[send_next] activePlanPath: " << _.activePlanPathEdges.size()
                                       << " edges");
    MRPT_LOG_DEBUG_STREAM(
        "[send_next] activePlanEdgeIndex="
        << (_.activePlanEdgeIndex ? std::to_string(*_.activePlanEdgeIndex)
                                  : "(none)")
        << " activePlanEdgeSentIndex="
        << (_.activePlanEdgeSentIndex
                ? (std::to_string(*_.activePlanEdgeSentIndex) +
                   _.activePlanPathEdges.at(*_.activePlanEdgeSentIndex)
                       .asString())
                : "(none)"));

    // First edge of a plan?
    if (!_.activePlanEdgeIndex.has_value())
    {
        ASSERT_EQUAL_(
            _.activePlanPath.size(), _.activePlanPathEdges.size() + 1);

        _.activePlanEdgeIndex = 0;  // first edge

        // save odometry at the beginning of the first edge:
        ASSERT_LT_(
            mrpt::system::timeDifference(
                lastVehicleOdometry_.timestamp, mrpt::Clock::now()),
            1.0);

        _.activePlanInitOdometry = lastVehicleOdometry_.odometry;

        MRPT_LOG_INFO_STREAM(
            "Starting motion plan with:\n"
            " - odometry    : "
            << lastVehicleOdometry_.odometry << "\n"
            << " - localization: " << lastVehicleLocalization_.pose);
    }

    // Waiting for the end of this edge motion?
    // Must be done *before* the next if() block:
    if (_.activePlanEdgeSentIndex.has_value() &&
        *_.activePlanEdgeSentIndex == *_.activePlanEdgeIndex)
    {
        if (!config_.vehicleMotionInterface->enqeued_motion_pending())
        {
            const auto formerActiveNodeIndex = *_.activePlanEdgeIndex + 1;

            // We are ready for the next one:
            _.activePlanEdgeIndex.value()++;

            const auto newActiveNodeIndex = *_.activePlanEdgeIndex + 1;

            // Get the odometry value when triggered:
            _.lastEnqueuedTriggerOdometry =
                config_.vehicleMotionInterface
                    ->enqued_motion_last_odom_when_triggered();

            MRPT_LOG_INFO_STREAM(
                "Enqueued motion seems to have been done for odom="
                << (_.lastEnqueuedTriggerOdometry
                        ? _.lastEnqueuedTriggerOdometry.value()
                              .odometry.asString()
                        : "")
                << ". Moving to next edge #" << *_.activePlanEdgeIndex
                << " out of " << _.activePlanPathEdges.size());

            // Remove the visual mark of "pending trigger area":
            _.activeEnqueuedConditionForViz.reset();

            // Use odometry at the trigger point to correct path plan:
            if (*_.activePlanEdgeIndex < _.activePlanPathEdges.size())
            {
                const auto nodesDelta =
                    _.activePlanPath.at(newActiveNodeIndex).pose -
                    _.activePlanPath.at(formerActiveNodeIndex).pose;

                MRPT_LOG_DEBUG_STREAM(
                    "activePlanPath[former]:"
                    << _.activePlanPath.at(formerActiveNodeIndex).asString());
                MRPT_LOG_DEBUG_STREAM(
                    " activePlanPath[new]:"
                    << _.activePlanPath.at(newActiveNodeIndex).asString());

                const auto nextNodeOdomPose =
                    _.lastEnqueuedTriggerOdometry->odometry + nodesDelta;

                MRPT_LOG_DEBUG_STREAM("nextNodeOdomPose:" << nextNodeOdomPose);

                const auto nextNodeRelPoseWrtInit =
                    nextNodeOdomPose - *_.activePlanInitOdometry;

                const auto firstNodeLocPose = _.activePlanPath.at(0).pose;
                const auto nextNodeCorrected =
                    firstNodeLocPose + nextNodeRelPoseWrtInit;

                MRPT_LOG_DEBUG_STREAM("firstNodeLocPose: " << firstNodeLocPose);

                MRPT_LOG_INFO_STREAM(
                    "Applying triggered odometry-based node pose correction: "
                    << _.activePlanPath.at(newActiveNodeIndex).asString()
                    << " ==> " << nextNodeCorrected);

                // Apply correction to pose:
                _.activePlanPath.at(newActiveNodeIndex).pose =
                    nextNodeCorrected;

                // Correct PTG arguments according to the final actual poses.
                // Needed to correct for lattice approximations:
                refine_trajectory(
                    _.activePlanPath, _.activePlanPathEdges, config_.ptgs);
            }
        }
    }

    // Time to send out a new edge:
    if ((!_.activePlanEdgeSentIndex.has_value() ||
         *_.activePlanEdgeSentIndex != *_.activePlanEdgeIndex) &&
        _.activePlanPath.size() > *_.activePlanEdgeIndex + 1)
    {
        const auto& nFirst = _.activePlanPath.at(0);
        const auto& nCurr  = _.activePlanPath.at(*_.activePlanEdgeIndex);
        const auto& nNext  = _.activePlanPath.at(*_.activePlanEdgeIndex + 1);

        std::optional<MotionPrimitivesTreeSE2::node_t> nAfterNext;
        if (_.activePlanPath.size() > *_.activePlanEdgeIndex + 2)
            nAfterNext = _.activePlanPath.at(*_.activePlanEdgeIndex + 2);

        // Mark the next "motion edge" as "sent":
        _.activePlanEdgeSentIndex = *_.activePlanEdgeIndex;

        // If we have at least two actions, use the two actions at a time API:

        // Check if this robot supports enqueued actions:
        const auto supportsEnqueued =
            config_.vehicleMotionInterface->supports_enqeued_motions();
        ASSERT_(supportsEnqueued);  // TODO: Implement adaptor layer

        const auto& edge = _.activePlanPathEdges.at(*_.activePlanEdgeIndex);

        auto& ptg = config_.ptgs.ptgs.at(edge.ptgIndex);
        ptg->updateNavDynamicState(edge.getPTGDynState());
        if (auto* ptgTrim = dynamic_cast<ptg::SpeedTrimmablePTG*>(ptg.get());
            ptgTrim)
            ptgTrim->trimmableSpeed_ = edge.ptgTrimmableSpeed;

        const mrpt::kinematics::CVehicleVelCmd::Ptr generatedMotionCmd =
            ptg->directionToMotionCommand(edge.ptgPathIndex);

        _.sentOutCmdInThisIteration = generatedMotionCmd;  // log record copy

        ASSERT_(generatedMotionCmd);

        // Before changing the dynamic status of the (potentially same one) PTG
        // for the "next" edge, query the PTG for the extra additional motion
        // required for the condPose below:
        std::optional<mrpt::math::TPose2D> poseCondDeltaForTolerance;
        {
            uint32_t stepEnd = 0, stepAfter = 0;
            bool     ok1 = ptg->getPathStepForDist(
                edge.ptgPathIndex, edge.ptgDist, stepEnd);
            bool ok2 = ptg->getPathStepForDist(
                edge.ptgPathIndex,
                edge.ptgDist + config_.enqueuedActionsToleranceXY, stepAfter);
            if (ok1 && ok2)
            {
                poseCondDeltaForTolerance =
                    ptg->getPathPose(edge.ptgPathIndex, stepAfter) -
                    ptg->getPathPose(edge.ptgPathIndex, stepEnd);
            }
        }

        // Next edge motion:
        mrpt::kinematics::CVehicleVelCmd::Ptr generatedMotionCmdAfter;
        if (nAfterNext.has_value())
        {
            const auto& nextEdge =
                _.activePlanPathEdges.at(*_.activePlanEdgeIndex + 1);

            auto& nextPtg = config_.ptgs.ptgs.at(nextEdge.ptgIndex);
            nextPtg->updateNavDynamicState(nextEdge.getPTGDynState());
            if (auto* ptgTrim =
                    dynamic_cast<ptg::SpeedTrimmablePTG*>(nextPtg.get());
                ptgTrim)
                ptgTrim->trimmableSpeed_ = nextEdge.ptgTrimmableSpeed;

            generatedMotionCmdAfter =
                nextPtg->directionToMotionCommand(nextEdge.ptgPathIndex);

            ASSERT_(generatedMotionCmdAfter);

            const auto relPoseNext = nNext.pose - nCurr.pose;
            // const auto relPoseAfterNext = nAfterNext->pose - nCurr.pose;

            ASSERT_(_.activePlanInitOdometry.has_value());

            // Convert from the "map" localization frame to "odom" frame:
            auto condPose =
                _.activePlanInitOdometry.value() + (nNext.pose - nFirst.pose);

            // Shift the "condition pose" such that the desired nominal pose is
            // reached within one box of size toleranceXY:
            if (poseCondDeltaForTolerance.has_value())
                condPose = condPose + poseCondDeltaForTolerance.value();

            // Create the motion command and send to the user-provided interface
            // to the vehicle:
            MRPT_LOG_INFO_STREAM(
                "Generating compound motion cmd to move from node ID "
                << nCurr.nodeID_ << " => " << nNext.nodeID_ << " => "
                << nAfterNext->nodeID_
                << "\n CMD1: " << generatedMotionCmd->asString()
                << "\n CMD2: " << generatedMotionCmdAfter->asString()
                << "\n relPoseNext: " << relPoseNext  //
                << "\n CondPose: " << condPose  //
                << "\n ETA: " << edge.estimatedExecTime  //
                << "\n withTolDelta: "
                << (poseCondDeltaForTolerance
                        ? poseCondDeltaForTolerance.value().asString()
                        : std::string("(none)"))  //
                << "\n nNext.pose: " << nNext.pose  //
                << "\n nCurr.pose: " << nCurr.pose  //
            );

            selfdriving::EnqueuedMotionCmd enqMotion;
            enqMotion.nextCmd                = generatedMotionCmdAfter;
            enqMotion.nextCondition.position = condPose;

            enqMotion.nextCondition.tolerance = {
                config_.enqueuedActionsToleranceXY,
                config_.enqueuedActionsToleranceXY,
                config_.enqueuedActionsTolerancePhi};

            enqMotion.nextCondition.timeout =
                std::max(1.0, edge.estimatedExecTime) *
                config_.enqueuedActionsTimeoutMultiplier;

            _.activeEnqueuedConditionForViz = enqMotion.nextCondition;

            // if the immediate cmd was already sent out, skip it and just send
            // the enqueued part:
            if (_.activePlanEdgesSentOut.count(*_.activePlanEdgeIndex) == 0)
            {
                // send both, both are new:
                config_.vehicleMotionInterface->motion_execute(
                    generatedMotionCmd, enqMotion);

                _.activePlanEdgesSentOut.insert(*_.activePlanEdgeIndex);
                _.activePlanEdgesSentOut.insert(*_.activePlanEdgeIndex + 1);
            }
            else
            {
                // Only the enqueued part is new:
                config_.vehicleMotionInterface->motion_execute(
                    std::nullopt, enqMotion);

                _.activePlanEdgesSentOut.insert(*_.activePlanEdgeIndex + 1);
            }

            // Cancel current path planner thread?
            // Launch a new planner from the new predicted pose?
            // It's already handled by check_have_to_replan()
        }
        else
        {
            MRPT_LOG_INFO_STREAM(
                "Generating single motion cmd to move from node ID "
                << nCurr.nodeID_ << " => " << nNext.nodeID_
                << " CMD:" << generatedMotionCmd->asString());

            config_.vehicleMotionInterface->motion_execute(
                generatedMotionCmd, std::nullopt);

            _.activePlanEdgesSentOut.insert(*_.activePlanEdgeIndex);
        }

        // new motion generated and sent out. We are done.
        return;
    }

    // If we are here, it is because we are in the middle of a navigation,
    // an edge motion is under execution and we are waiting for its end.
    // Send out a "dead man's switch" reset signal:
    config_.vehicleMotionInterface->motion_execute(std::nullopt, std::nullopt);
    MRPT_LOG_DEBUG("[send_next] NOP.");
}

void TPS_Navigator::send_planner_output_to_viz(const PathPlannerOutput& ppo)
{
    // Visualize the motion tree:
    // ----------------------------------
    RenderOptions ro;
    ro.highlight_path_to_node_id = ppo.po.bestNodeId;
    ro.width_normal_edge         = 0;  // hidden
    ro.draw_obstacles            = false;
    ro.ground_xy_grid_frequency  = 0;  // disabled
    ro.phi2z_scale               = 0;

    mrpt::opengl::CSetOfObjects::Ptr planViz =
        render_tree(ppo.po.motionTree, ppo.po.originalInput, ro);
    planViz->setName("astar_plan_result");

    planViz->setLocation(0, 0, 0.01);  // to easy the vis wrt the ground

    // Overlay the costmaps, if any:
    // ----------------------------------
    if (!ppo.costEvaluators.empty())
    {
        auto glCostMaps = mrpt::opengl::CSetOfObjects::Create();
        glCostMaps->setName("glCostMaps");

        float zOffset = 0.01f;  // to help visualize several costmaps at once

        for (const auto& ce : ppo.costEvaluators)
        {
            if (!ce) continue;
            auto glCostMap = ce->get_visualization();

            zOffset += 0.01f;
            glCostMap->setLocation(0, 0, zOffset);
            glCostMaps->insert(glCostMap);
        }

        planViz->insert(glCostMaps);
    }

    // Send to the viz "server":
    // ----------------------------------
    if (config_.vizSceneToModify)
    {
        // lock:
        if (config_.on_viz_pre_modify) config_.on_viz_pre_modify();

        if (auto glObj =
                config_.vizSceneToModify->getByName(planViz->getName());
            glObj)
        {
            auto glContainer =
                std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(glObj);
            ASSERT_(glContainer);
            *glContainer = *planViz;
        }
        else
        {
            config_.vizSceneToModify->insert(planViz);
        }

        // unlock:
        if (config_.on_viz_post_modify) config_.on_viz_post_modify();
    }

    // Send to the buffer to be saved to the nav log file:
    // ------------------------------------------------------
    {
        // Create object wrapper to fix coordinate origin, since the navlog
        // custom visuals are relative to the robot position, not global:
        auto glGlobalWrtRobot = mrpt::opengl::CSetOfObjects::Create();
        glGlobalWrtRobot->insert(planViz);
        glGlobalWrtRobot->setPose(-lastVehicleLocalization_.pose);
        innerState_.planVizForNavLog = glGlobalWrtRobot;
    }
}

void TPS_Navigator::send_path_to_viz_and_navlog(
    const MotionPrimitivesTreeSE2&         tree,
    const std::optional<TNodeID>&          finalNode,
    const PlannerInput&                    originalPlanInput,
    const std::vector<CostEvaluator::Ptr>& costEvaluators)
{
    // Visualize the motion tree:
    // ----------------------------------
    RenderOptions ro;
    ro.highlight_path_to_node_id = finalNode;
    ro.width_normal_edge         = 0;  // hidden
    ro.draw_obstacles            = false;
    ro.ground_xy_grid_frequency  = 0;  // disabled
    ro.phi2z_scale               = 0;

    mrpt::opengl::CSetOfObjects::Ptr planViz =
        render_tree(tree, originalPlanInput, ro);
    planViz->setName("astar_plan_result");

    planViz->setLocation(0, 0, 0.01);  // to easy the vis wrt the ground

    // Overlay the costmaps, if any:
    // ----------------------------------
    if (!costEvaluators.empty())
    {
        auto glCostMaps = mrpt::opengl::CSetOfObjects::Create();
        glCostMaps->setName("glCostMaps");

        float zOffset = 0.01f;  // to help visualize several costmaps at once

        for (const auto& ce : costEvaluators)
        {
            if (!ce) continue;
            auto glCostMap = ce->get_visualization();

            zOffset += 0.01f;
            glCostMap->setLocation(0, 0, zOffset);
            glCostMaps->insert(glCostMap);
        }

        planViz->insert(glCostMaps);
    }

    // Send to the viz "server":
    // ----------------------------------
    if (config_.vizSceneToModify)
    {
        // lock:
        if (config_.on_viz_pre_modify) config_.on_viz_pre_modify();

        if (auto glObj =
                config_.vizSceneToModify->getByName(planViz->getName());
            glObj)
        {
            auto glContainer =
                std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(glObj);
            ASSERT_(glContainer);
            *glContainer = *planViz;
        }
        else
        {
            config_.vizSceneToModify->insert(planViz);
        }

        // unlock:
        if (config_.on_viz_post_modify) config_.on_viz_post_modify();
    }

    // Send to the buffer to be saved to the nav log file:
    // ------------------------------------------------------
    {
        // Create object wrapper to fix coordinate origin, since the navlog
        // custom visuals are relative to the robot position, not global:
        auto glGlobalWrtRobot = mrpt::opengl::CSetOfObjects::Create();
        glGlobalWrtRobot->insert(planViz);
        glGlobalWrtRobot->setPose(-lastVehicleLocalization_.pose);
        innerState_.planVizForNavLog = glGlobalWrtRobot;
    }
}

void TPS_Navigator::send_current_state_to_viz_and_navlog()
{
    if (!config_.vizSceneToModify && !navlog_output_file_) return;

    const auto& _ = innerState_;

    auto glStateDetails = mrpt::opengl::CSetOfObjects::Create();
    glStateDetails->setName("glStateDetails");
    glStateDetails->setLocation(0, 0, 0.02);  // to easy the vis wrt the ground

    // last poses track:
    if (const auto& poses = _.latestPoses; !poses.empty())
    {
        auto glRobotPath = mrpt::opengl::CSetOfLines::Create();
        glRobotPath->setColor_u8(0x80, 0x80, 0x80, 0x80);
        const auto p0 = poses.begin()->second;
        glRobotPath->appendLine(p0.x, p0.y, 0, p0.x, p0.y, 0);
        for (const auto& p : poses)
        {
            glRobotPath->appendLineStrip(p.second.x, p.second.y, 0);

            auto glCorner =
                mrpt::opengl::stock_objects::CornerXYSimple(0.1, 1.0);
            glCorner->setPose(p.second);
            glStateDetails->insert(glCorner);
        }
        glStateDetails->insert(glRobotPath);
    }

    if (const auto& actCond = _.activeEnqueuedConditionForViz;
        actCond.has_value() && _.activePlanInitOdometry.has_value() &&
        !_.activePlanPath.empty())
    {
        const auto p = _.activePlanPath.at(0).pose +
                       (actCond->position - _.activePlanInitOdometry.value());
        const auto tol = actCond->tolerance;

        const mrpt::math::TPose2D p0 = {
            p.x - tol.x, p.y - tol.y, p.phi - tol.phi};
        const mrpt::math::TPose2D p1 = {
            p.x + tol.x, p.y + tol.y, p.phi + tol.phi};

        auto glCondPoly = mrpt::opengl::CSetOfLines::Create();
        glCondPoly->setColor_u8(0xf0, 0xf0, 0xf0, 0xa0);

        glCondPoly->appendLine(p0.x, p0.y, 0, p1.x, p0.y, 0);
        glCondPoly->appendLineStrip(p1.x, p1.y, 0);
        glCondPoly->appendLineStrip(p0.x, p1.y, 0);
        glCondPoly->appendLineStrip(p0.x, p0.y, 0);

        glStateDetails->insert(glCondPoly);

        {
            auto glCorner =
                mrpt::opengl::stock_objects::CornerXYSimple(0.15, 1.0);
            glCorner->setPose(p0);
            glStateDetails->insert(glCorner);
        }
        {
            auto glCorner =
                mrpt::opengl::stock_objects::CornerXYSimple(0.15, 1.0);
            glCorner->setPose(p1);
            glStateDetails->insert(glCorner);
        }
    }

    if (const auto& triggOdom = _.lastEnqueuedTriggerOdometry;
        triggOdom.has_value() && !_.activePlanPath.empty() &&
        _.activePlanInitOdometry.has_value())
    {
        auto glCorner = mrpt::opengl::stock_objects::CornerXYZ(0.15);
        glCorner->setPose(
            _.activePlanPath.at(0).pose +
            (triggOdom.value().odometry - _.activePlanInitOdometry.value()));
        glStateDetails->insert(glCorner);
    }

    if (const auto& predPose = _.collisionCheckingPosePrediction;
        predPose.has_value())
    {
        auto glVehShape = mrpt::opengl::CSetOfLines::Create();

        glVehShape->setLineWidth(1);
        glVehShape->setColor_u8(0x40, 0x40, 0x40, 0x80);

        render_vehicle(config_.ptgs.robotShape, *glVehShape);

        glVehShape->setPose(predPose.value());

        glStateDetails->insert(glVehShape);
    }

    // Send to the viz "server":
    // ----------------------------------
    if (config_.vizSceneToModify)
    {
        // lock:
        if (config_.on_viz_pre_modify) config_.on_viz_pre_modify();

        if (auto glObj =
                config_.vizSceneToModify->getByName(glStateDetails->getName());
            glObj)
        {
            auto glContainer =
                std::dynamic_pointer_cast<mrpt::opengl::CSetOfObjects>(glObj);
            ASSERT_(glContainer);
            *glContainer = *glStateDetails;
        }
        else
        {
            config_.vizSceneToModify->insert(glStateDetails);
        }

        // unlock:
        if (config_.on_viz_post_modify) config_.on_viz_post_modify();
    }

    // Send to the buffer to be saved to the nav log file:
    // ------------------------------------------------------
    {
        // Create object wrapper to fix coordinate origin, since the navlog
        // custom visuals are relative to the robot position, not global:
        auto glGlobalWrtRobot = mrpt::opengl::CSetOfObjects::Create();
        glGlobalWrtRobot->insert(glStateDetails);
        glGlobalWrtRobot->setPose(-lastVehicleLocalization_.pose);
        innerState_.stateVizForNavLog = glGlobalWrtRobot;
    }
}

void TPS_Navigator::internal_start_navlog_file()
{
    if (!config_.generateNavLogFiles) return;

    navlog_output_file_.reset();  // close any previous file
    navlogOutputFirstEntry_ = true;

    // Select output file name:
    std::string outFileName;
    int         fileCnt = 0;
    for (;;)
    {
        outFileName = config_.navLogFilesPrefix +
                      mrpt::format("_%03i.reactivenavlog", fileCnt++);
        if (mrpt::system::fileExists(outFileName))
            continue;
        else
            break;
    }

    MRPT_LOG_INFO_STREAM("Initiating navlog file: " << outFileName);

    navlog_output_file_.emplace(outFileName);

    if (!navlog_output_file_->is_open())
    {  // report error:
        MRPT_LOG_ERROR_STREAM("Error creating file: " << outFileName);
    }
}

void TPS_Navigator::internal_write_to_navlog_file()
{
    try
    {
        auto& _ = innerState_;

        if (!navlog_output_file_ || !navlog_output_file_->is_open()) return;

        // save:
        mrpt::nav::CLogFileRecord r;

        if (config_.globalMapObstacleSource)
        {
            if (const auto pts = config_.globalMapObstacleSource->obstacles();
                pts)
                r.WS_Obstacles.insertAnotherMap(
                    pts.get(),
                    -mrpt::poses::CPose3D(lastVehicleLocalization_.pose));
        }

        if (config_.localSensedObstacleSource)
        {
            if (const auto pts = config_.localSensedObstacleSource->obstacles();
                pts)
                r.WS_Obstacles_original.insertAnotherMap(
                    pts.get(),
                    -mrpt::poses::CPose3D(lastVehicleLocalization_.pose));
        }

        const auto& rs = config_.ptgs.robotShape;

        if (auto pPoly = std::get_if<mrpt::math::TPolygon2D>(&rs); pPoly)
        {
            const auto&  poly   = *pPoly;
            const size_t nVerts = poly.size();
            r.robotShape_x.resize(nVerts);
            r.robotShape_y.resize(nVerts);
            for (size_t i = 0; i < nVerts; i++)
            {
                r.robotShape_x[i] = poly.at(i).x;
                r.robotShape_y[i] = poly.at(i).y;
            }
        }
        else if (auto pRadius = std::get_if<double>(&rs); pRadius)
        {
            const double R      = *pRadius;
            r.robotShape_radius = R;
        }

        r.robotPoseLocalization = lastVehicleLocalization_.pose;
        r.robotPoseOdometry     = lastVehicleOdometry_.odometry;

        // r.WS_targets_relative                = relTargets;

        r.nSelectedPTG = -1;  // None

        r.cur_vel = lastVehicleOdometry_.odometryVelocityLocal.rotated(
            lastVehicleOdometry_.odometry.phi);
        r.cur_vel_local = lastVehicleOdometry_.odometryVelocityLocal;

        // r.cmd_vel = new_vel_cmd; // TODO!

        // r.values["executionTime"]            = executionTimeValue;

        r.timestamps["tim_start_iteration"] =
            mrpt::Clock::fromDouble(_.timStartThisNavStep.value());
        r.timestamps["curPoseAndVel"] = lastVehicleLocalization_.timestamp;

        r.nPTGs = config_.ptgs.ptgs.size();

        r.infoPerPTG.resize(r.nPTGs + 1);  // convention: NumPTGs + NOP choice

        // At the beginning of each log file, add an introductory block
        // explaining which PTGs we use:
        if (navlogOutputFirstEntry_)
        {
            navlogOutputFirstEntry_ = false;
            for (size_t i = 0; i < r.nPTGs; i++)
            {
                // If we make a direct copy (=) we will store the entire,
                // heavy, collision grid. Let's just store the parameters of
                // each PTG by serializing it, so paths can be reconstructed
                // by invoking initialize()
                mrpt::io::CMemoryStream buf;
                auto arch = mrpt::serialization::archiveFrom(buf);
                arch << config_.ptgs.ptgs.at(i);
                buf.Seek(0);
                r.infoPerPTG[i].ptg = std::dynamic_pointer_cast<
                    mrpt::nav::CParameterizedTrajectoryGenerator>(
                    arch.ReadObject());
            }
        }
#if 0
    // NOP mode  stuff:
    r.rel_cur_pose_wrt_last_vel_cmd_NOP = rel_cur_pose_wrt_last_vel_cmd_NOP;
    r.rel_pose_PTG_origin_wrt_sense_NOP = rel_pose_PTG_origin_wrt_sense_NOP;
    r.ptg_index_NOP  = best_is_NOP_cmdvel ? m_lastSentVelCmd.ptg_index : -1;
    r.ptg_last_k_NOP = m_lastSentVelCmd.ptg_alpha_index;
    r.ptg_last_navDynState = m_lastSentVelCmd.ptg_dynState;
#endif

        // Sent-out motion command:
        r.cmd_vel = _.sentOutCmdInThisIteration;

        // opengl additional viz stuff:
#if MRPT_VERSION >= 0x257
        if (_.planVizForNavLog) r.visuals.push_back(_.planVizForNavLog);
        if (_.stateVizForNavLog) r.visuals.push_back(_.stateVizForNavLog);
#endif

        // debug strings:
        {
            std::vector<std::string> splitLines;
            for (const auto& str : innerState_.navlogDebugMessages)
            {
                std::vector<std::string> lins;
                mrpt::system::tokenize(str, "\n", lins);
                // Add in reverse order since navlog-viewer shows lines
                // down-up:
                for (auto rit = lins.rbegin(); rit != lins.rend(); rit++)
                    splitLines.push_back(*rit);
            }

            for (unsigned int i = 0; i < splitLines.size(); i++)
                r.additional_debug_msgs[mrpt::format("%03u", i)] =
                    splitLines[i];
        }

        mrpt::serialization::archiveFrom(*navlog_output_file_) << r;
    }
    catch (const std::exception& e)
    {
    }
}

bool TPS_Navigator::approach_target_controller()
{
    auto& _ = innerState_;

    const auto atrw = internal_check_about_to_reach_stop_wp();

    // Check for no-getting-closer timeout here, since we have the distance to
    // goal:
    if (!_.lastDistanceToGoal.has_value() ||
        atrw.distanceToWaypoint < *_.lastDistanceToGoal)
    {
        // Good: we are making progress:
        _.lastDistanceToGoal = atrw.distanceToWaypoint;
        _.lastDistanceToGoalTimestamp =
            config_.vehicleMotionInterface->robot_time();
    }
    else if (_.lastDistanceToGoalTimestamp.has_value())
    {
        // we are not making progress:
        const double age = config_.vehicleMotionInterface->robot_time() -
                           *_.lastDistanceToGoalTimestamp;
        if (age > config_.timeoutNotGettingCloserGoal)
        {
            MRPT_LOG_INFO_FMT(
                "Triggering on_path_seems_blocked() event since distance to "
                "goal could not get shorter than %f for %f seconds.",
                _.lastDistanceToGoal.value(), age);

            pendingEvents_.emplace_back([this]() {
                config_.vehicleMotionInterface->on_path_seems_blocked();
            });
        }
    }

    // check about-to-reach-waypoint return values:
    if (atrw.aboutToReach)
    {
        MRPT_LOG_DEBUG_STREAM(
            "[approach_target_controller] aboutToReach="
            << (atrw.aboutToReach ? "true" : "false")
            << " distanceToWaypoint=" << atrw.distanceToWaypoint);
    }

    if (!atrw.aboutToReach)
    {
        // Reset statuses for approaching algorithm:
        if (config_.targetApproachController)
            config_.targetApproachController->reset_state();

        // go on as normal
        return false;
    }

    // No user-defined controller? -> skip the rest of this step
    if (!config_.targetApproachController) return false;

    const auto& wps = _.waypointNavStatus.waypoints;
    const auto& wp  = wps.at(*_.pathPlannerTargetWpIdx);

    TargetApproachInput tacIn;
    tacIn.speedLimits = absoluteSpeedLimits_;
    tacIn.vls         = lastVehicleLocalization_;
    tacIn.vos         = lastVehicleOdometry_;
    tacIn.target      = wp;
    if (*_.pathPlannerTargetWpIdx >= 1)
        tacIn.previous = wps.at(*_.pathPlannerTargetWpIdx - 1);
    tacIn.localSensedObstacleSource = config_.localSensedObstacleSource;
    tacIn.vehicleMotionInterface    = config_.vehicleMotionInterface;
    tacIn.ptgsAndShape              = config_.ptgs;

    const auto out = config_.targetApproachController->execute(tacIn);

    MRPT_LOG_DEBUG_FMT(
        "Controller towards target executed since atrw. aboutToReach=true, "
        "results: targetDist=%f handled=%s reachedDetected=%s",
        atrw.distanceToWaypoint, out.handled ? "YES" : "NO",
        out.reachedDetected ? "YES" : "NO");

    if (out.handled)
    {
        if (out.reachedDetected)
        {
            // end of navigation to waypoint:
            internal_mark_current_wp_as_reached();
        }
        else
        {
            // It's a motion command:
            ASSERT_(config_.vehicleMotionInterface);
            if (out.generatedMotion)
            {
                config_.vehicleMotionInterface->motion_execute(
                    out.generatedMotion, std::nullopt);

                // log record copy:
                _.sentOutCmdInThisIteration = out.generatedMotion;
            }
            else
            {
                config_.vehicleMotionInterface->motion_execute(
                    std::nullopt, std::nullopt);
            }
        }
    }

    return out.handled;
}

// Called from internal_rnav_step()
TPS_Navigator::AboutToReachWpInfo TPS_Navigator::internal_check_about_to_reach_stop_wp()
{
    AboutToReachWpInfo ret;

    auto& _ = innerState_;

    // no plan or no active target wp?
    if (_.waypointNavStatus.waypoints.empty() ||
        !_.pathPlannerTargetWpIdx.has_value())
        return ret;

    const auto& wps = _.waypointNavStatus.waypoints;
    const auto& wp  = wps.at(*_.pathPlannerTargetWpIdx);

    // Get SE(2) (x,y,phi) relative pose of next target with respect to the
    // current robot pose:
    const mrpt::math::TPose2D nextTargetWrtRobot =
        wp.targetAsPose() - lastVehicleLocalization_.pose;

    ret.distanceToWaypoint = nextTargetWrtRobot.norm();

    // If the waypoint is not a reach-and-stop one, ignore special behaviors:
    if (wp.speedRatio > 0)
    {
        // Regular, non-stop waypoint. Do nothing especial.
        return ret;
    }

    if (ret.distanceToWaypoint < config_.maxDistanceForTargetApproach &&
        std::abs(mrpt::math::wrapToPi(nextTargetWrtRobot.phi)) <
            config_.maxRelativeHeadingForTargetApproach)
    {  // yes:
        ret.aboutToReach = true;
    }

    return ret;
}

void TPS_Navigator::absoluteSpeedLimits(
    const mrpt::kinematics::CVehicleVelCmd::TVelCmdParams& newLimits)
{
    // TODO: anything else with current under-execution motion? Abort and
    // replan?
    absoluteSpeedLimits_ = newLimits;
}

void TPS_Navigator::merge_new_plan_if_better(const PathPlannerOutput& result)
{
    auto& _ = innerState_;

    // Check if the new plan is any better than the current one?
    const auto goal =
        _.waypointNavStatus.waypoints.at(_.pathPlannerTargetWpIdx.value())
            .targetAsPose();

    const double currentPlanDistToGoal =
        (goal - _.activePlanPath.rbegin()->pose).translation().norm();

    if (!result.po.bestNodeId.has_value())
    {
        // error: no good plan.
        MRPT_LOG_WARN("Dropping new path planning result, no bestNodeID.");
        return;
    }

    const double newPlanDistToGoal =
        (goal - result.po.motionTree.nodes().at(*result.po.bestNodeId).pose)
            .translation()
            .norm();

    if (currentPlanDistToGoal < config_.plannerParams.grid_resolution_xy ||
        newPlanDistToGoal > currentPlanDistToGoal * 0.99)
    {
        MRPT_LOG_INFO_STREAM(
            "Dropping new path planning result, improvement is not good "
            "enough. CurrentDistToGoal="
            << currentPlanDistToGoal
            << " NewPlanDistToGoal=" << newPlanDistToGoal);
        return;
    }

    MRPT_LOG_INFO_STREAM("Merging new path planning result...");

    // merge current under-execution path planning and the new
    // for-the-future segment that was just received:

    auto [newPath, newEdges] =
        result.po.motionTree.backtrack_path(*result.po.bestNodeId);

    // Correct PTG arguments according to the final actual poses.
    // Needed to correct for lattice approximations:
    refine_trajectory(newPath, newEdges, config_.ptgs);

    _.activePlanOutput = std::move(result);

    // Overwrite the plan, starting from the next node on:
    const auto formerEdgeIndex = *_.activePlanEdgeSentIndex;

    const auto formerEdgeInitOdometry =
        *_.activePlanInitOdometry + (_.activePlanPath.at(formerEdgeIndex).pose -
                                     _.activePlanPath.at(0).pose);

    _.active_plan_reset();

    /* Remap:
     *
     * Old edges:
     *  - [0,...,formerActiveEdgeIndex-1] => dissapear
     *  - [formerActiveEdgeIndex]         => new edge #0
     *  - [formerActiveEdgeIndex+1,...]   => dissapear

     * Old nodes:
     *  - [0,...,formerActiveEdgeIndex-1] => dissapear
     *  - formerActiveEdgeIndex           => new node #0
     *  - formerActiveEdgeIndex+1         => new node #1
     *  - [formerActiveEdgeIndex+2,...]   => dissapear
     *
     */
    std::vector<MotionPrimitivesTreeSE2::node_t> newPlanPath;
    std::vector<MotionPrimitivesTreeSE2::edge_t> newPathEdges;

    for (size_t i = 0; i <= formerEdgeIndex; i++)
        newPathEdges.push_back(_.activePlanPathEdges.at(i));

    // No need to add the last one, so it's "<" instead of "<=", since
    // that node is also duplicated as the new node list at position #0:
    for (size_t i = 0; i < formerEdgeIndex + 1; i++)
        newPlanPath.push_back(_.activePlanPath.at(formerEdgeIndex));

    _.activePlanPath      = std::move(newPlanPath);
    _.activePlanPathEdges = std::move(newPathEdges);

    for (const auto& node : newPath) _.activePlanPath.push_back(node);

    for (const auto& edge : newEdges) _.activePlanPathEdges.push_back(*edge);

    // Reconstruct current state:
    // We are waiting for the execution of the old "formerEdgeIndex", new
    // #0, edge motion:
    _.activePlanEdgeIndex     = formerEdgeIndex;
    _.activePlanEdgeSentIndex = formerEdgeIndex;
    for (size_t i = 0; i <= formerEdgeIndex; i++)
        _.activePlanEdgesSentOut.insert(i);
    _.activePlanInitOdometry = formerEdgeInitOdometry;
}

void TPS_Navigator::internal_mark_current_wp_as_reached()
{
    auto& _ = innerState_;

    // sanity checks:
    ASSERT_(_.pathPlannerTargetWpIdx.has_value());
    ASSERT_LT_(*_.pathPlannerTargetWpIdx, _.waypointNavStatus.waypoints.size());

    const waypoint_idx_t reachedIdx = *_.pathPlannerTargetWpIdx;

    // We are about to mark "reachedIdx" as reached.
    // First, go over the former ones, since the last "reached" and mark them as
    // skipped:
    {
        waypoint_idx_t lastReached = reachedIdx;
        while (lastReached > 0 &&
               !_.waypointNavStatus.waypoints.at(lastReached).reached)
        {  // go back:
            lastReached--;
        }
        // now, mark all in the range [lastReached+1, ..., reachedIdx-1] as
        // skipped:
        for (waypoint_idx_t i = lastReached + 1; i + 1 <= reachedIdx; i++)
        {
            // mark as skipped:
            _.waypointNavStatus.waypoints.at(i).skipped = true;

            // user callbacks:
            pendingEvents_.emplace_back([this, i]() {
                config_.vehicleMotionInterface->on_waypoint_reached(
                    i, false /* =skipped */);
            });
        }
    }

    // mark final one as reached:
    _.waypointNavStatus.waypoints.at(reachedIdx).reached = true;

    // user callbacks:
    pendingEvents_.emplace_back([this, reachedIdx]() {
        config_.vehicleMotionInterface->on_waypoint_reached(
            reachedIdx, true /* =reached */);
    });

    // clear statuses so we can launch a new plan in the next iteration:
    _.active_plan_reset(true);
}

bool TPS_Navigator::check_all_waypoints_are_done()
{
    auto& _ = innerState_;
    if (_.waypointNavStatus.waypoints.empty())
        return false;  // should never happen...

    // Was the last waypoint reached?
    return _.waypointNavStatus.waypoints.rbegin()->reached;
}
