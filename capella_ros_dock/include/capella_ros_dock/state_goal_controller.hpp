// State-pattern docking goal controller.
// Replaces the monolithic switch-state-machine in simple_goal_controller.hpp.
// Keeps the same public API so the caller (DockingBehavior) only swaps the type.

#ifndef CAPELLA_ROS_DOCK__STATE_GOAL_CONTROLLER_HPP_
#define CAPELLA_ROS_DOCK__STATE_GOAL_CONTROLLER_HPP_

#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "boost/optional.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <time.h>
#include <rclcpp/time.hpp>
#include <capella_ros_msg/msg/velocities.hpp>
#include "rclcpp/rclcpp.hpp"
#include "capella_ros_dock/utils.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_util/line_iterator.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "capella_ros_dock/behaviors_scheduler.hpp"
#include "capella_ros_dock/simple_goal_controller.hpp"  // reused for the CmdPath type alias

namespace capella_ros_dock
{

struct GoalPoint
{
	double x;
	double y;
	double theta;
	float radius;
	bool drive_backwards;
};

class StateGoalController
{
public:
	// Same public API as SimpleGoalController.
	using CmdPath = SimpleGoalController::CmdPath;
	using optional_output_t = BehaviorsScheduler::optional_output_t;

	StateGoalController(
		rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
		rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
		rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
		rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
		motion_control_params * params_ptr);

	~StateGoalController();

	void initialize_goal(const CmdPath & cmd_path);
	void reset();

	optional_output_t get_velocity_for_position(
		const tf2::Transform & current_pose, const tf2::Transform & robot_pose_map,
		const tf2::Transform & charger_pose_map, bool sees_dock, bool is_docked, bool bluetooth_connected,
		nav_msgs::msg::Odometry odom_msg, std::string & state, std::string & infos, bool & b_timeout_current_state,
		nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision_checker,
		nav2_costmap_2d::Costmap2D costmap, std::vector<geometry_msgs::msg::Point> footprint_vec,
		rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_clear_entire_local_costmap);

private:
	// states are ordered by StateRank; index == rank
	enum class StateRank : int
	{
		INIT = 0,
		LOOKUP_MARKER,
		ANGLE_TO_BUFFER_POINT,
		MOVE_TO_BUFFER_POINT,
		ANGLE_TO_X_POSITIVE_ORIENTATION,
		ANGLE_TO_GOAL,
		GO_TO_GOAL_POSITION,
		GOAL_ANGLE,
		UNDOCK
	};

	/// Per-tick inputs and long-lived state shared with the state classes.
	struct StateCtx
	{
		// deps
		rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
		rclcpp::Logger logger_ {rclcpp::get_logger("state_goal_controller")};
		rclcpp::Clock::SharedPtr clock_;
		motion_control_params * params = nullptr;
		std::mutex mutex_;

		// tick inputs (bound by the controller at the start of each tick)
		tf2::Transform current_pose;
		tf2::Transform robot_pose_map;
		tf2::Transform charger_pose_map;
		bool sees_dock = false;
		bool is_docked = false;
		bool bluetooth_connected = false;
		nav_msgs::msg::Odometry odom_msg;
		double current_angle = 0.0;
		tf2::Vector3 current_position;

		nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision_checker;
		nav2_costmap_2d::Costmap2D costmap;
		std::vector<geometry_msgs::msg::Point> footprint_vec;
		rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_clear_entire_local_costmap;

		// session
		std::deque<GoalPoint> goal_points_;
		std::string state;
		std::string infos;
		bool b_timeout = false;

		// cached poses (saved from robot_pose_map / current_pose / charger_pose_map)
		bool marker_visible_ = false;
		tf2::Transform tf_robot_map_;
		double robot_x_map_ = 0.0, robot_y_map_ = 0.0, robot_yaw_map_ = 0.0;
		tf2::Transform tf_robot_charger_;
		double robot_x_charger_ = 0.0, robot_y_charger_ = 0.0, robot_yaw_charger_ = 0.0;
		tf2::Transform tf_charger_map_;
		double charger_x_map_ = 0.0, charger_y_map_ = 0.0, charger_yaw_map_ = 0.0;

		// timing
		double now_time_ = 0.0, pre_time_ = 0.0, delta_time_ = 0.0;
		double current_state_start_time_ = 0.0, current_state_timeout_ = 0.0;
		double thre_angle_diff = 0.30;

		// buffer-point / approach scratch
		double dist_buffer_point = 0.0;
		double dist_buffer_point_yaw = 0.0;
		double waiting_for_best_coord_start_time = 0.0;
		bool start_time_recorded = false;
		double buffer_goal_point_x = 0.0;
		double buffer_goal_point_y = 0.0;
		double robot_angle_to_buffer_point_yaw = 0.0;
		double robot_current_yaw = 0.0;
		double robot_current_yaw_positive = 0.0;
		bool drive_back = false;
		double theta_positive = 0.0, theta_negative = 0.0;
		double buffer_point2_x_map = 0.0, buffer_point2_y_map = 0.0;

		// marker-loss recovery
		rclcpp::Time last_time_cannot_see_dock = rclcpp::Time(0);
		rclcpp::Time now_time_cannot_see_dock = rclcpp::Time(0);
		bool first_cannot_see_dock = true;
		bool need_get_outof_charger_range = false;
		bool get_out_of_charger_range_completed = true;

		// rotation smoothing
		float last_rotation_speed_ = 0.0f;
		double last_rotation_speed_time_ = 0.0;
		bool first_pub_rotation_speed = true;

		// contact hold
		bool first_contacted = true;
		double first_contacted_time = 0.0;

		// undock
		bool undocking = false;
		double undock_dis_moved_ = 0.0;

		// goal-speed profile
		bool pose_x_init_recoreded_ = false;
		double pose_x_init_ = 0.0;

		// angle/move to buffer point geometry
		double theta_angle_to_buffer_point = 0.0;
		double dist_move_to_buffer_point = 0.0;
		tf2::Transform tf_before_angle_to_buffer_point;
		tf2::Transform tf_after_angle_to_buffer_point;
		tf2::Transform tf_after_move_to_buffer_point;

		// costmap-clear throttle
		double clear_time_last = 0.0, clear_time_now = 0.0, clear_time_delta = 0.0;

		// debug publishers
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_charger_pose_agent_pub_;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_charger_pose_apriltag_pub_;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_buffer_point2_pub_;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_undock_collision_line_pub_;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_undock_collision_point_pub_;
		rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_undock_point_loop_pub_;

		// helpers (defined in state_goal_controller.cpp)
		void save_all_poses_infos();
		double now_sec() const { return clock_->now().seconds(); }
		void update_time_smart();
		bool check_timeout();
		void bound_rotation(double & rotation_velocity, float min, float max) const;
		float generate_smooth_rotation_speed(float cur_rotation);
		double diff_angle(const GoalPoint & goal_pt);
		void clear_local_costmap();
		double get_cost_value(tf2::Transform tf_robot, bool rotation, double linear, double angular,
			double predict_time);
		double get_cost_value_undock(tf2::Transform tf_robot, double linear, double predict_time);
		void publish_marker(
			const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & pub, int32_t id,
			const std::string & ns, int32_t type, double scale, float r, float g, float b, double x, double y);
		bool converged_position_3cond(double theta, double base_link_y) const;
		bool skip_collision_near_charger(double & x_c2r, double & yaw_c2r_abs) const;
		void rotation_collision_block(geometry_msgs::msg::Twist & vel, double remaining_rot_angle, const char * state_name);
		void translation_collision_block(
			geometry_msgs::msg::Twist & vel, double remaining_dist, bool charger_dist_mode,
			const char * state_name);
	};

	/// Result of one state step: velocity to publish and optional transition request.
	struct StepResult
	{
		optional_output_t twist;
		StateRank next = StateRank::INIT;
		double next_timeout = 0.0;
		bool next_valid = false;

		static StepResult stay(const optional_output_t & vel) { return StepResult{vel, StateRank::INIT, 0.0, false}; }
		static StepResult go(const optional_output_t & vel, StateRank next_state, double timeout) {
			return StepResult{vel, next_state, timeout, true};
		}
	};

	/// Base state of the state machine.
	class DockState
	{
	public:
		virtual ~DockState() = default;
		virtual StateRank rank() const = 0;
		virtual const char * name() const = 0;
		// Called once on the first tick after the state becomes current (deferred enter).
		virtual StepResult on_enter(StateCtx & ctx) { (void)ctx; return StepResult::stay(optional_output_t()); }
		// Called every tick.
		virtual StepResult on_update(StateCtx & ctx) = 0;
		// Called on the last tick before leaving the state.
		virtual void on_exit(StateCtx & ctx) { (void)ctx; }
		// Called when the shared timeout guard fires while this state is current.
		virtual void on_timeout(StateCtx & ctx) { (void)ctx; }
		// Whether the shared timeout guard applies to this state (INIT never times out).
		virtual bool checks_timeout() const { return true; }
	};

	class InitState : public DockState
	{
	public:
		StateRank rank() const override { return StateRank::INIT; }
		const char * name() const override { return "INIT"; }
		StepResult on_enter(StateCtx & ctx) override;
		StepResult on_update(StateCtx & ctx) override;
		bool checks_timeout() const override { return false; }
	};
	class LookupMarkerState : public DockState
	{
	public:
		StateRank rank() const override { return StateRank::LOOKUP_MARKER; }
		const char * name() const override { return "LOOKUP_MARKER"; }
		StepResult on_update(StateCtx & ctx) override;
	};
	class AngleToBufferPointState : public DockState
	{
	public:
		StateRank rank() const override { return StateRank::ANGLE_TO_BUFFER_POINT; }
		const char * name() const override { return "ANGLE_TO_BUFFER_POINT"; }
		StepResult on_update(StateCtx & ctx) override;
	};
	class MoveToBufferPointState : public DockState
	{
	public:
		StateRank rank() const override { return StateRank::MOVE_TO_BUFFER_POINT; }
		const char * name() const override { return "MOVE_TO_BUFFER_POINT"; }
		StepResult on_update(StateCtx & ctx) override;
	};
	class AngleToXPositiveOrientationState : public DockState
	{
	public:
		StateRank rank() const override { return StateRank::ANGLE_TO_X_POSITIVE_ORIENTATION; }
		const char * name() const override { return "ANGLE_TO_X_POSITIVE_ORIENTATION"; }
		StepResult on_update(StateCtx & ctx) override;
	};
	class AngleToGoalState : public DockState
	{
	public:
		StateRank rank() const override { return StateRank::ANGLE_TO_GOAL; }
		const char * name() const override { return "ANGLE_TO_GOAL"; }
		StepResult on_update(StateCtx & ctx) override;
	};
	class GoToGoalPositionState : public DockState
	{
	public:
		StateRank rank() const override { return StateRank::GO_TO_GOAL_POSITION; }
		const char * name() const override { return "GO_TO_GOAL_POSITION"; }
		StepResult on_update(StateCtx & ctx) override;
	};
	class GoalAngleState : public DockState
	{
	public:
		StateRank rank() const override { return StateRank::GOAL_ANGLE; }
		const char * name() const override { return "GOAL_ANGLE"; }
		StepResult on_update(StateCtx & ctx) override;
	};
	class UndockState : public DockState
	{
	public:
		StateRank rank() const override { return StateRank::UNDOCK; }
		const char * name() const override { return "UNDOCK"; }
		StepResult on_update(StateCtx & ctx) override;
		void on_timeout(StateCtx & ctx) override { ctx.undocking = false; }
	};

	StateCtx ctx_;
	std::vector<std::unique_ptr<DockState>> states_;
	StateRank current_rank_ = StateRank::INIT;
	bool need_enter_ = true;

	DockState * current_state() { return states_[static_cast<int>(current_rank_)].get(); }
	void transition(StateRank target, double timeout_sec);
};

} // namespace capella_ros_dock

#endif // CAPELLA_ROS_DOCK__STATE_GOAL_CONTROLLER_HPP_