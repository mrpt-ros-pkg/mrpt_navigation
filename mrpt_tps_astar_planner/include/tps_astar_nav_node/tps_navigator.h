/* -------------------------------------------------------------------------
 *   SelfDriving C++ library based on PTGs and mrpt-nav
 * Copyright (C) 2019-2022 Jose Luis Blanco, University of Almeria
 * See LICENSE for license information.
 * ------------------------------------------------------------------------- */

#pragma once

#include <mrpt/core/WorkerThreadsPool.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/poses/CPose2DInterpolator.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/system/CTimeLogger.h>
#include <mrpt/typemeta/TEnumType.h>
#include <selfdriving/algos/CostEvaluatorCostMap.h>
#include <selfdriving/algos/CostEvaluatorPreferredWaypoint.h>
#include <selfdriving/algos/TPS_Astar.h>
#include <selfdriving/data/PlannerInput.h>
#include <selfdriving/data/PlannerOutput.h>
#include <selfdriving/data/TrajectoriesAndRobotShape.h>
#include <selfdriving/data/Waypoints.h>
#include <selfdriving/interfaces/ObstacleSource.h>
#include <selfdriving/interfaces/TargetApproachController.h>
#include <selfdriving/interfaces/VehicleMotionInterface.h>

#include <functional>
#include <list>

namespace selfdriving
{
using namespace mrpt;

/** The different statuses for the navigation system. */
enum class NavStatus : uint8_t
{
    IDLE = 0,
    NAVIGATING,
    SUSPENDED,
    NAV_ERROR
};

/** Explains the reason for the navigation error. */
enum class NavError : uint8_t
{
    NONE = 0,
    EMERGENCY_STOP,
    CANNOT_REACH_TARGET,
    OTHER
};
struct NavErrorReason
{
    NavErrorReason()       = default;
    NavError    error_code = NavError::NONE;
    std::string error_msg;  //!< Human friendly description of the error
};

/** The top-level interface for users to control the vehicle navigation.
 *
 * This class is based and extends `mrpt::nav::CAbstractNavigator` with the
 * capability of following a list of waypoints. By default, waypoints are
 * followed one by one, but, refer to \c Waypoint for a discussion on
 * `allow_skip`.
 *
 * How to use:
 *  - \c initialize() must be called before running any actual navigation.
 *
 *  - Callbacks must be provided for interfacing the real (or simulated)
 * robot/vehicle and its sensors. They must be provided as std::function<>
 * instances within \c config_. Note they may be actual functions, or lambdas.
 *
 *  - \c navigation_step() must be called periodically in order to effectively
 * run the navigation. This method will internally call the callbacks to gather
 * sensor data and robot positioning data.
 *
 * This class implements the following state machine (see \c current_status()):
 * \dot
 *  digraph NavStatus {
 *      IDLE; NAVIGATING; SUSPENDED; NAV_ERROR;
 *      IDLE -> NAVIGATING [ label="request_navigation()" ];
 *      NAVIGATING -> IDLE [ label="Final target reached" ];
 *      NAVIGATING -> IDLE [ label="cancel()" ];
 *      NAVIGATING -> NAV_ERROR [ label="Upon sensor errors, timeout,..." ];
 *      NAVIGATING -> SUSPENDED [ label="suspend()" ];
 *      SUSPENDED -> NAVIGATING [ label="resume()" ];
 *      NAV_ERROR -> IDLE [ label="reset_nav_error()" ];
 *  }
 *  \enddot
 *
 * All methods are thread-safe, in the sense that mutexes are internally used
 * to ensure no undefined navigation state is possible if invoking an object of
 * this class from more than one thread.
 *
 */
class TPS_Navigator : public mrpt::system::COutputLogger
{
   public:
    /** ctor */
    TPS_Navigator() : mrpt::system::COutputLogger("TPS_Navigator") {}

    /** dtor */
    ~TPS_Navigator();

    /** \name Initialization and parameters
     * @{ */

    struct Configuration
    {
        Configuration() = default;

        /** @name Main configuration fields; must fill before initialize()
         *  @{ */
        VehicleMotionInterface::Ptr vehicleMotionInterface;

        /** Having at least one of these is mandatory */
        ObstacleSource::Ptr globalMapObstacleSource, localSensedObstacleSource;

        TrajectoriesAndRobotShape ptgs;

        TargetApproachController::Ptr targetApproachController;
        /** @} */

        /** @name Parameters
         *  @{ */

#if 0
        /** Default value=0, means use the "targetAllowedDistance" passed by the
         * user in the navigation request. */
        double dist_to_target_for_sending_event{0};


        /** (Default value=0.6) When closer than this distance, check if the
         * target is blocked to abort navigation with an error. */
        double dist_check_target_is_blocked{0.6};
#endif

        /** (Default=4.0) Hoy many meters to add at each side
         * (up,down,left,right) of the bbox enclosing the starting and goal
         * pose for each individual call to the A* planner. */
        double planner_bbox_margin = 4.0;

        double enqueuedActionsToleranceXY       = 0.05;
        double enqueuedActionsTolerancePhi      = 2.0_deg;
        double enqueuedActionsTimeoutMultiplier = 1.3;

        double lookAheadImmediateCollisionChecking = 1.0;  // [s]

        double maxDistanceForTargetApproach        = 1.5;  // [m]
        double maxRelativeHeadingForTargetApproach = 180.0_deg;  // [rad]

        /** Navigation timeout (seconds) [Default=30 sec]
         *  See description of VehicleMotionInterface::on_path_seems_blocked()
         */
        double timeoutNotGettingCloserGoal = 30;

        bool generateNavLogFiles = false;

        /** Actual files will be
         * `${navLogFilesPrefix}_${UNIQUE_ID}.reactivenavlog` */
        std::string navLogFilesPrefix = "./selfdriving";

        void                   loadFrom(const mrpt::containers::yaml& c);
        mrpt::containers::yaml saveTo() const;

        TPS_Astar_Parameters plannerParams;

        CostEvaluatorCostMap::Parameters           globalCostParameters;
        CostEvaluatorCostMap::Parameters           localCostParameters;
        CostEvaluatorPreferredWaypoint::Parameters preferWaypointsParameters;

        /** @} */

        /**  \name Visualization callbacks and methods
         *   @{ */

        std::function<void(void)>                    on_viz_pre_modify;
        std::shared_ptr<mrpt::opengl::CSetOfObjects> vizSceneToModify;
        std::function<void(void)>                    on_viz_post_modify;

        /** @} */
    };

    /** Must be called before any other navigation command, and after filling in
     *  all the required data into \c config_
     */
    virtual void initialize();

    Configuration config_;

    /** Read access to current absolute speed limits */
    const mrpt::kinematics::CVehicleVelCmd::TVelCmdParams& absoluteSpeedLimits()
        const
    {
        return absoluteSpeedLimits_;
    }

    /** Changes the current speed limits */
    void absoluteSpeedLimits(
        const mrpt::kinematics::CVehicleVelCmd::TVelCmdParams& newLimits);

    /** @} */

    /** \name Waypoint navigation control API
     * @{ */

    /** Waypoint navigation request. This immediately cancels any other previous
     * on-going navigation.
     */
    virtual void request_navigation(const WaypointSequence& navRequest);

    /** This method must be called periodically in order to effectively run the
     * navigation */
    virtual void navigation_step();

    /** Cancel current navegation. */
    virtual void cancel();

    /** Continues with suspended navigation. \sa suspend */
    virtual void resume();

    /** Suspend current navegation. \sa resume */
    virtual void suspend();

    /** Resets a `NAV_ERROR` state back to `IDLE` */
    virtual void reset_nav_error();

    /** Returns the current navigator status. */
    inline NavStatus current_status() const { return navigationStatus_; }

    /** In case of status=NAV_ERROR, this returns the reason for the error.
     * Error status is reseted every time a new navigation starts with
     * a call to navigate(), or when reset_nav_error() is called.
     */
    inline const NavErrorReason& error_reason() const
    {
        return navErrorReason_;
    }

    /** Get a copy of the control structure which describes the progress status
     * of the waypoint navigation. */
    WaypointStatusSequence waypoint_nav_status() const;

    /** Gets a write-enabled reference to the list of waypoints, simultaneously
     * acquiring the critical section mutex.
     * Caller must call endWaypointsAccess() when done editing the waypoints.
     */
    WaypointStatusSequence& beginWaypointsAccess()
    {
        navMtx_.lock();
        return innerState_.waypointNavStatus;
    }

    /** Must be called after beginWaypointsAccess() */
    void end_waypoints_access() { navMtx_.unlock(); }

    /** Publicly available time profiling object. Default: disabled */
    mrpt::system::CTimeLogger navProfiler_{true /*enabled*/, "TPS_Navigator"};

    /** @}*/

    struct PathPlannerOutput
    {
        PathPlannerOutput() = default;

        selfdriving::PlannerOutput po;

        /// A copy of the employed costs.
        std::vector<CostEvaluator::Ptr> costEvaluators;

        /// (See same name field in PathPlannerInput)
        std::optional<TNodeID> startingFromCurrentPlanNode;
        /// (See same name field in PathPlannerInput)
        std::optional<mrpt::math::TPose2D> startingFromCurrentPlanNodePose;
    };

    /** Use the callbacks above and render_tree() to update a visualization
     * with a given plan output */
    void send_planner_output_to_viz(const PathPlannerOutput& ppo);

    /** Update the GUI with a partial or final path only (no whole tree) */
    void send_path_to_viz_and_navlog(
        const MotionPrimitivesTreeSE2&         tree,
        const std::optional<TNodeID>&          finalNode,
        const PlannerInput&                    originalPlanInput,
        const std::vector<CostEvaluator::Ptr>& costEvaluators);

    /** Update current path plan visualization details in the GUI, or in the
     *  opengl object buffered to be writen to navlog files.
     */
    void send_current_state_to_viz_and_navlog();

   protected:
    /** Current and last internal state of navigator: */
    NavStatus      navigationStatus_    = NavStatus::IDLE;
    NavStatus      lastNavigationState_ = NavStatus::IDLE;
    NavErrorReason navErrorReason_;

    mrpt::system::output_logger_callback_t loggerToNavlog_;

    bool initialized_ = false;

    /** mutex for all navigation methods */
    std::recursive_mutex navMtx_;

    /** Current robot kinematic state; Updated in navigation_step() with a
     * minimum period of MIN_TIME_BETWEEN_POSE_UPDATES.
     */
    VehicleLocalizationState lastVehicleLocalization_;
    VehicleOdometryState     lastVehicleOdometry_;
    double                   lastVehiclePosRobotTime_ = 0;

    /** Events generated during navigation_step(), enqueued to be called at the
     * end of the method execution to avoid user code to change the navigator
     * state. */
    std::list<std::function<void(void)>> pendingEvents_;

    void dispatch_pending_nav_events();

    /** Call to the robot getCurrentPoseAndSpeeds() and updates members
     * m_curPoseVel accordingly.
     * If an error is returned by the user callback, first, it calls
     * robot.stop() ,then throws an std::runtime_error exception. */
    virtual void update_robot_kinematic_state();

    /** The actual action that happens inside navigation_step() for the
     * case of state being NAVIGATING.
     */
    virtual void impl_navigation_step();

    /** Implements the way to waypoint is free function in children classes:
     * `true` must be returned
     * if, according to the information gathered at the last navigation step,
     * there is a free path to
     * the given point; `false` otherwise: if way is blocked or there is
     * missing information, the point is out of range, etc. */
    //    virtual bool impl_waypoint_is_reachable(
    //        const mrpt::math::TPoint2D& wp_local_wrt_robot) const = 0;

    void internal_on_start_new_navigation();
    void internal_start_navlog_file();
    void internal_write_to_navlog_file();

    /// Created in internal_start_navlog_file()
    std::optional<mrpt::io::CFileGZOutputStream> navlog_output_file_;
    bool                                         navlogOutputFirstEntry_ = true;

    // Path planning in a parallel thread:
    mrpt::WorkerThreadsPool pathPlannerPool_{
        1 /*Single thread*/, mrpt::WorkerThreadsPool::POLICY_DROP_OLD,
        "path_planner"};

    struct PathPlannerInput
    {
        PathPlannerInput() = default;

        selfdriving::PlannerInput pi;

        /** If this is path refining plan request, this is the ID of the node
         * that acts as starting state, with the ID in the current activePlan.
         */
        std::optional<TNodeID> startingFromCurrentPlanNode;
        /** Like above, but with the SE(2) pose of that node. Used to tell from
         *  ambiguous situations where the node ID actually remains the same,
         * but after a path merging, so the node pose is different.
         */
        std::optional<mrpt::math::TPose2D> startingFromCurrentPlanNodePose;
    };

    // Argument is a copy instead of a const-ref intentionally.
    PathPlannerOutput path_planner_function(PathPlannerInput ppi);

    struct AlignStatus
    {
        bool is_aligning() const { return isAligning_; }
        bool is_aligning_after_overshoot() const { return isAfterOvershoot_; }
        mrpt::math::TPoint2D target_waypoint{.0, .0};
        void                 reset() { *this = AlignStatus(); }
        double               timeSinceLast() const
        {
            if (timeLastAlignCmd_ == INVALID_TIMESTAMP)
                return 1e6;
            else
                return mrpt::system::timeDifference(
                    timeLastAlignCmd_, mrpt::system::now());
        }
        void setAsAligningNow()
        {
            isAligning_       = true;
            timeLastAlignCmd_ = mrpt::system::now();
        }
        void setAsAligningNowAfterOvershoot()
        {
            setAsAligningNow();
            isAfterOvershoot_ = true;
        }

       private:
        mrpt::system::TTimeStamp timeLastAlignCmd_ = INVALID_TIMESTAMP;
        bool                     isAligning_       = false;
        bool                     isAfterOvershoot_ = false;
    };

    /** Everything that should be cleared upon a new navigation command. */
    struct CurrentNavInternalState
    {
        CurrentNavInternalState() = default;

        void clear() { *this = CurrentNavInternalState(); }

        /** The latest waypoints navigation command and the up-to-date control
         * status. */
        WaypointStatusSequence waypointNavStatus;

        /** Latest robot poses, updated in navigation_Step() */
        mrpt::poses::CPose2DInterpolator latestPoses, latestOdomPoses;

        std::future<PathPlannerOutput> pathPlannerFuture;

        /** The final waypoint of the currently under-optimization/already
         * finished path planning.
         */
        std::optional<waypoint_idx_t> pathPlannerTargetWpIdx;

        /// From check_immediate_collision(). For Debug visualization.
        std::optional<mrpt::math::TPose2D> collisionCheckingPosePrediction;

        /** Set by check_new_planner_output() */
        PathPlannerOutput                            activePlanOutput;
        std::vector<MotionPrimitivesTreeSE2::node_t> activePlanPath;
        std::vector<MotionPrimitivesTreeSE2::edge_t> activePlanPathEdges;

        /** A copy of the active queued condition, for viz purposes only
         *  (coordinates here are in the odometry frame)
         */
        std::optional<EnqueuedCondition> activeEnqueuedConditionForViz;

        /** A copy of the last odometry when an enqueued action was triggered,
         * for viz purposed only */
        std::optional<VehicleOdometryState> lastEnqueuedTriggerOdometry;

        void active_plan_reset(bool alsoClearComputedPath = false)
        {
            activePlanEdgeIndex.reset();
            activePlanEdgeSentIndex.reset();
            activePlanEdgesSentOut.clear();
            activePlanInitOdometry.reset();

            if (alsoClearComputedPath)
            {
                activePlanOutput = {};
                activePlanPath.clear();
                activePlanPathEdges.clear();
                pathPlannerTargetWpIdx.reset();
                lastDistanceToGoalTimestamp.reset();
                lastDistanceToGoal.reset();
            }
        }

        /** 0-based index of which edge in activePlanPathEdges[] is currently
         * being executed by the robot. Empty if the plan is new and no order
         * has been sent out to the robot yet.
         * The i-th edge is moving between nodes [i] and [i+1] in
         * activePlanPath.
         */
        std::optional<size_t> activePlanEdgeIndex;

        /** Will be equal to activePlanEdgeIndex once the command has been sent
         * out to the robot */
        std::optional<size_t> activePlanEdgeSentIndex;

        std::set<size_t> activePlanEdgesSentOut;

        /** The robot pose in the *odom* frame when the first motion edge is
         * executed from send_next_motion_cmd_or_nop() */
        std::optional<mrpt::math::TPose2D> activePlanInitOdometry;

        /** @name Data to be cleared upon each iteration
         *  @{ */
        /** Copy of sent-out cmd, for the log record */
        mrpt::kinematics::CVehicleVelCmd::Ptr sentOutCmdInThisIteration;
        mrpt::opengl::CSetOfObjects::Ptr      planVizForNavLog;
        mrpt::opengl::CSetOfObjects::Ptr      stateVizForNavLog;
        std::vector<std::string>              navlogDebugMessages;

        std::optional<double> lastNavigationStepEndTime;
        std::optional<double> timStartThisNavStep;

        void clearPerIterationData()
        {
            sentOutCmdInThisIteration.reset();
            planVizForNavLog.reset();
            stateVizForNavLog.reset();
            navlogDebugMessages.clear();
        }

        /** Values used to check against
         * Configuration::timeoutNotGettingCloserGoal
         */
        std::optional<double> lastDistanceToGoalTimestamp, lastDistanceToGoal;

        /** @} */

        /** For sending an alarm (error event) when it seems that we are not
         * approaching toward the target in a while... */
        double badNavAlarmMinDistTarget_ = std::numeric_limits<double>::max();
        mrpt::Clock::time_point badNavAlarmLastMinDistTime_ =
            mrpt::Clock::now();

        /** Will be false until the navigation end is sent. */
        bool navigationEndEventSent = false;
    };

    /** Navigation state variables, protected by navMtx_ */
    CurrentNavInternalState innerState_;

    /** Speed limits: If not defined, the values from the first PTG will be
     * copied here upon initialize() */
    mrpt::kinematics::CVehicleVelCmd::TVelCmdParams absoluteSpeedLimits_;

    /** Checks whether the current motion leads us into an obstacle */
    void check_immediate_collision();

    /** Checks whether we need to launch a new RRT* path planner */
    void check_have_to_replan();

    /** Checks whether the A* planner finished, then send a new active
     * trajectory to the path tracker */
    void check_new_planner_output();

    /** Checks and send next motion command, or NOP, if we are on track */
    void send_next_motion_cmd_or_nop();

    /** Finds the next waypt index up to which we should find a new RRT*
       plan */
    waypoint_idx_t find_next_waypoint_for_planner();

    /** Enqueues a task in pathPlannerPool_ running path_planner_function() and
     * saving future results into pathPlannerFuture.
     *
     * If this is a path refining, startingFrom and startingFromNodeID must be
     * supplied, with the latter being the nodeId of the the plan starting state
     * in activePlanOutput, activePlanPath, activePlanPathEdges.
     */
    void enqueue_path_planner_towards(
        const waypoint_idx_t             target,
        const selfdriving::SE2_KinState& startingFrom,
        const std::optional<TNodeID>&    startingFromNodeID = std::nullopt);

    /** Special behavior: if we are about to reach a WP with a stop condition,
     *  handle it specially if there's an obvious free path towards it.
     *  \return true if the special motion has been generated (or it's under
     *          execution). false if the regular path plan should go on.
     *  \note Call from within send_next_motion_cmd_or_nop()
     */
    bool approach_target_controller();

    void merge_new_plan_if_better(const PathPlannerOutput& result);

    void internal_mark_current_wp_as_reached();

    /** Returns true if all waypoints has been reached successfully. */
    bool check_all_waypoints_are_done();

    struct AboutToReachWpInfo
    {
        AboutToReachWpInfo() = default;

        bool   aboutToReach       = false;
        double distanceToWaypoint = std::numeric_limits<double>::max();
    };

    /**
     * @brief Checks whether the robot is within `maxDistanceForTargetApproach`
     * meters of the next non-skippable waypoint. This does not check for
     * potential obstacles, just the physical nearness.
     * @return See AboutToReachWpInfo.
     */
    AboutToReachWpInfo internal_check_about_to_reach_stop_wp();

#if 0
    bool checkHasReachedTarget(const double targetDist) const override;

    /** The waypoints-specific part of navigation_step() */
    virtual void waypoints_navigation_step();

    bool waypoints_isAligning() const { return m_is_aligning; }

    /** Whether the last timestep was "is_aligning" in a waypoint with heading
     */
    bool                     m_was_aligning{false};
    bool                     m_is_aligning{false};
    mrpt::system::TTimeStamp m_last_alignment_cmd;
#endif
};

}  // namespace selfdriving

MRPT_ENUM_TYPE_BEGIN(selfdriving::NavStatus)
MRPT_FILL_ENUM_MEMBER(selfdriving, NavStatus::IDLE);
MRPT_FILL_ENUM_MEMBER(selfdriving, NavStatus::NAVIGATING);
MRPT_FILL_ENUM_MEMBER(selfdriving, NavStatus::SUSPENDED);
MRPT_FILL_ENUM_MEMBER(selfdriving, NavStatus::NAV_ERROR);
MRPT_ENUM_TYPE_END()
