// State-pattern docking goal controller implementation.
// State transitions and per-state velocity generation, ported from
// simple_goal_controller.hpp's monolithic switch with repeated logic de-duplicated.

#include "capella_ros_dock/state_goal_controller.hpp"

#include <algorithm>
#include <unistd.h>
#include <utility>

using namespace std::chrono_literals;

namespace { // file-local helpers

inline float degree_to_radian(float degree)
{
	return degree / 180.0f * M_PI;
}

} // anonymous namespace

namespace capella_ros_dock
{

using optional_output_t = StateGoalController::optional_output_t;

// ---------------------------------------------------------------------------
// construction / public API
// ---------------------------------------------------------------------------

StateGoalController::StateGoalController(
	rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
	rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
	rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
	rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
	motion_control_params * params_ptr)
{
	ctx_.node_ = node;
	ctx_.logger_ = node_logging_interface->get_logger();
	ctx_.clock_ = node_clock_interface->get_clock();
	ctx_.params = params_ptr;

	ctx_.marker_charger_pose_agent_pub_ = rclcpp::create_publisher<visualization_msgs::msg::Marker>(
		node_topics_interface, "marker_charger_pose_agent",
		rclcpp::QoS(1).reliable().transient_local());
	ctx_.marker_charger_pose_apriltag_pub_ = rclcpp::create_publisher<visualization_msgs::msg::Marker>(
		node_topics_interface, "marker_charger_pose_apriltag",
		rclcpp::QoS(1).reliable().transient_local());
	ctx_.marker_buffer_point2_pub_ = rclcpp::create_publisher<visualization_msgs::msg::Marker>(
		node_topics_interface, "marker_buffer_point2",
		rclcpp::QoS(1).reliable().transient_local());
	ctx_.marker_undock_collision_line_pub_ = rclcpp::create_publisher<visualization_msgs::msg::Marker>(
		node_topics_interface, "marker_undock_collision_line",
		rclcpp::QoS(1).reliable().transient_local());
	ctx_.marker_undock_collision_point_pub_ = rclcpp::create_publisher<visualization_msgs::msg::Marker>(
		node_topics_interface, "marker_undock_collision_point",
		rclcpp::QoS(1).reliable().transient_local());
	ctx_.marker_undock_point_loop_pub_ = rclcpp::create_publisher<visualization_msgs::msg::Marker>(
		node_topics_interface, "marker_undock_point_loop",
		rclcpp::QoS(1).reliable().transient_local());

	// buffer goal point (docked, low_vel_dist, first_goal_dist, buffer_goal_dist)
	ctx_.buffer_goal_point_x = -(params_ptr->offset_last_docked_distance
		+ params_ptr->offset_low_speed + params_ptr->offset_second_goal + params_ptr->offset_buffer_goal);
	ctx_.buffer_goal_point_y = 0.0 + params_ptr->goal_y_correction;

	// camera geometry -> max allowed angle between robot heading and goal target
	float camera_horizontal_view = degree_to_radian(params_ptr->camera_horizontal_view);
	float marker_size = params_ptr->marker_size;
	float camera_baselink_dis = params_ptr->camera_baselink_dis;
	float goal_dis_x = params_ptr->offset_last_docked_distance + params_ptr->offset_low_speed
		+ params_ptr->offset_second_goal;
	RCLCPP_INFO(ctx_.logger_, "camera_horizontal_view: %.2f, marker_size: %.2f, camera_baselink_dis: %.2f, goal_dis_x: %.2f",
		camera_horizontal_view, marker_size, camera_baselink_dis, goal_dis_x);

	float d1, d2, d3, alpha;
	d1 = goal_dis_x;
	d2 = camera_baselink_dis;
	d3 = marker_size * 0.5;
	alpha = camera_horizontal_view * 0.5;
	float tan_alpha = std::tan(alpha);
	RCLCPP_INFO(ctx_.logger_, "d1: %.2f, d2: %.2f, d3: %.2f, alpha: %.2f", d1, d2, d3, alpha);
	float r = std::hypot(d1 * tan_alpha - d3, d1 + d3 * tan_alpha);
	float x1 = d2 * tan_alpha;
	float x2 = d1 * tan_alpha - d3;
	float beta_plus_theta = std::acos(x1 / r);
	float beta = std::acos(x2 / r);
	ctx_.thre_angle_diff = beta_plus_theta - beta;
	RCLCPP_INFO(ctx_.logger_, "r: %.2f, x1: %.2f, x2: %.2f, beta_plus_theta: %.2f, beta: %.2f",
		r, x1, x2, beta_plus_theta, beta);
	RCLCPP_INFO(ctx_.logger_, "thre_angle_diff: %.2f", ctx_.thre_angle_diff);

	states_.resize(static_cast<int>(StateRank::UNDOCK) + 1);
	states_[static_cast<int>(StateRank::INIT)] = std::make_unique<InitState>();
	states_[static_cast<int>(StateRank::LOOKUP_MARKER)] = std::make_unique<LookupMarkerState>();
	states_[static_cast<int>(StateRank::ANGLE_TO_BUFFER_POINT)] = std::make_unique<AngleToBufferPointState>();
	states_[static_cast<int>(StateRank::MOVE_TO_BUFFER_POINT)] = std::make_unique<MoveToBufferPointState>();
	states_[static_cast<int>(StateRank::ANGLE_TO_X_POSITIVE_ORIENTATION)] =
		std::make_unique<AngleToXPositiveOrientationState>();
	states_[static_cast<int>(StateRank::ANGLE_TO_GOAL)] = std::make_unique<AngleToGoalState>();
	states_[static_cast<int>(StateRank::GO_TO_GOAL_POSITION)] = std::make_unique<GoToGoalPositionState>();
	states_[static_cast<int>(StateRank::GOAL_ANGLE)] = std::make_unique<GoalAngleState>();
	states_[static_cast<int>(StateRank::UNDOCK)] = std::make_unique<UndockState>();
	current_rank_ = StateRank::INIT;
	need_enter_ = true;
	ctx_.current_state_start_time_ = ctx_.now_sec();
	ctx_.current_state_timeout_ = 1.0;
}

StateGoalController::~StateGoalController() = default;

void StateGoalController::initialize_goal(const CmdPath & cmd_path)
{
	RCLCPP_INFO(ctx_.logger_, "初始化cmd_path, 初始化当前state为: %s",
		magic_enum::enum_name(current_rank_).data());
	const std::lock_guard<std::mutex> lock(ctx_.mutex_);
	ctx_.goal_points_.clear();
	ctx_.goal_points_.resize(cmd_path.size());
	for (size_t i = 0; i < cmd_path.size(); ++i) {
		GoalPoint & gp = ctx_.goal_points_[i];
		const tf2::Vector3 & pt_position = cmd_path[i].pose.getOrigin();
		gp.x = pt_position.getX();
		gp.y = pt_position.getY();
		gp.theta = tf2::getYaw(cmd_path[i].pose.getRotation());
		gp.radius = cmd_path[i].radius;
		gp.drive_backwards = cmd_path[i].drive_backwards;
	}
	current_rank_ = StateRank::INIT;
	need_enter_ = true;
	ctx_.current_state_start_time_ = ctx_.now_sec();
	ctx_.current_state_timeout_ = 1.0;
}

void StateGoalController::reset()
{
	const std::lock_guard<std::mutex> lock(ctx_.mutex_);
	ctx_.goal_points_.clear();
}

void StateGoalController::transition(StateRank target, double timeout_sec)
{
	RCLCPP_INFO(ctx_.logger_, "change current state from %s to %s",
		magic_enum::enum_name(current_rank_).data(), magic_enum::enum_name(target).data());
	current_state()->on_exit(ctx_);
	current_rank_ = target;
	ctx_.current_state_start_time_ = ctx_.now_sec();
	ctx_.current_state_timeout_ = timeout_sec;
	ctx_.pre_time_ = ctx_.now_sec();
	need_enter_ = true;
}

optional_output_t StateGoalController::get_velocity_for_position(
	const tf2::Transform & current_pose, const tf2::Transform & robot_pose_map,
	const tf2::Transform & charger_pose_map, bool sees_dock, bool is_docked, bool bluetooth_connected,
	nav_msgs::msg::Odometry odom_msg, std::string & state, std::string & infos, bool & b_timeout_current_state,
	nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> collision_checker,
	nav2_costmap_2d::Costmap2D costmap, std::vector<geometry_msgs::msg::Point> footprint_vec,
	rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_clear_entire_local_costmap)
{
	// bind per-tick inputs
	ctx_.current_pose = current_pose;
	ctx_.robot_pose_map = robot_pose_map;
	ctx_.charger_pose_map = charger_pose_map;
	ctx_.sees_dock = sees_dock;
	ctx_.is_docked = is_docked;
	ctx_.bluetooth_connected = bluetooth_connected;
	ctx_.odom_msg = odom_msg;
	ctx_.collision_checker = collision_checker;
	ctx_.costmap = costmap;
	ctx_.footprint_vec = footprint_vec;
	ctx_.client_clear_entire_local_costmap = client_clear_entire_local_costmap;
	ctx_.b_timeout = false;
	ctx_.save_all_poses_infos();

	auto emit = [&](optional_output_t t) -> optional_output_t {
		state = ctx_.state;
		infos = ctx_.infos;
		b_timeout_current_state = ctx_.b_timeout;
		return t;
	};

	// impl undock (go to undock state)
	if (ctx_.goal_points_.size() > 0 && !(ctx_.goal_points_.front().drive_backwards)) {
		if (current_rank_ != StateRank::UNDOCK) {
			transition(StateRank::UNDOCK, ctx_.params->undock_timeout);
			ctx_.undock_dis_moved_ = 0.0;
		}
		sleep(0.5);  // wait for /charger/stop to execute.
		ctx_.undocking = true;
	} else {
		ctx_.undocking = false;
	}

	optional_output_t servo_vel;
	const std::lock_guard<std::mutex> lock(ctx_.mutex_);
	if (ctx_.is_docked && !ctx_.undocking) {
		if (ctx_.first_contacted) {
			ctx_.first_contacted = false;
			ctx_.first_contacted_time = ctx_.now_sec();
			RCLCPP_DEBUG(ctx_.logger_, "keep moving until %.2f expired.", ctx_.params->contacted_keep_move_time);
		} else {
			ctx_.now_time_ = ctx_.now_sec();
			if ((ctx_.now_time_ - ctx_.first_contacted_time) > ctx_.params->contacted_keep_move_time) {
				RCLCPP_INFO(ctx_.logger_, "*************** robot is docked *************");
				ctx_.goal_points_.clear();
				ctx_.start_time_recorded = false;
				ctx_.first_contacted = true;
			} else {
				RCLCPP_DEBUG(ctx_.logger_, "keep moving until %.2f expired, remaining %.2f seconds",
					ctx_.params->contacted_keep_move_time, ctx_.now_time_ - ctx_.first_contacted_time);
			}
		}
	}

	if (ctx_.goal_points_.size() == 0) {
		RCLCPP_INFO(ctx_.logger_, "*************** goal_points.size() = 0 *************");
		ctx_.state = std::string("goal_points.size() = 0");
		ctx_.infos = "Reason: goal_points.size() = 0 ==> stop ...";
		return emit(servo_vel);  // empty optional => done
	}

	ctx_.current_angle = tf2::getYaw(ctx_.current_pose.getRotation());
	ctx_.current_position = ctx_.current_pose.getOrigin();

	if (ctx_.sees_dock) {
		ctx_.first_cannot_see_dock = true;
	}

	const int rank_now = static_cast<int>(current_rank_);
	const int rank_x_positive = static_cast<int>(StateRank::ANGLE_TO_X_POSITIVE_ORIENTATION);

	if ((rank_now > rank_x_positive) && ctx_.need_get_outof_charger_range &&
		(ctx_.clock_->now().seconds() - ctx_.last_time_cannot_see_dock.seconds()) < (ctx_.params->time_sleep + 2)) {
		servo_vel = geometry_msgs::msg::Twist();
		servo_vel->linear.x = 0.15;
		ctx_.state = std::string("get_outof_charger_range");
		ctx_.infos = std::string("Reason: get_outof_charger_range executing ......");
		RCLCPP_INFO_THROTTLE(ctx_.logger_, *ctx_.clock_, 400, "get_outof_charger_range executing");
		return emit(servo_vel);
	}
	if (((ctx_.clock_->now().seconds() - ctx_.last_time_cannot_see_dock.seconds()) > (ctx_.params->time_sleep + 2)) &&
		(!ctx_.get_out_of_charger_range_completed)) {
		ctx_.need_get_outof_charger_range = false;
		ctx_.get_out_of_charger_range_completed = true;
		transition(StateRank::ANGLE_TO_X_POSITIVE_ORIENTATION, ctx_.params->timeout_angle_to_x_positive_orientation);
		servo_vel = geometry_msgs::msg::Twist();
		RCLCPP_INFO(ctx_.logger_, "get_outof_charge_range completed");
		ctx_.state = std::string(" get_outof_charge_range completed");
		ctx_.infos = std::string("Reason: get_outof_charger_range completed, go to state ANGLE_TO_X_POSITIVE_ORIENTATION");
		return emit(servo_vel);
	}

	if (!ctx_.sees_dock && rank_now > rank_x_positive && !ctx_.undocking) {
		if (ctx_.first_cannot_see_dock) {
			ctx_.last_time_cannot_see_dock = ctx_.clock_->now();
			ctx_.first_cannot_see_dock = false;
		}
		ctx_.now_time_cannot_see_dock = ctx_.clock_->now();
		if ((ctx_.now_time_cannot_see_dock.seconds() - ctx_.last_time_cannot_see_dock.seconds()) <
			ctx_.params->time_sleep) {
			servo_vel = geometry_msgs::msg::Twist();
			ctx_.state = std::string(" > ANGLE_TO_X_POSITIVE_ORIENTATION");
			ctx_.infos = std::string(
				"Reason: cannot see dock and navigate_state > ANGLE_TO_X_POSITIVE_ORIENTATION and stop time < time_sleep(default 5s) ==> stop");
			return emit(servo_vel);
		} else {
			if (std::abs(ctx_.current_position.getX()) > ctx_.params->robot_rotate_radius) {
				transition(StateRank::ANGLE_TO_X_POSITIVE_ORIENTATION, ctx_.params->timeout_angle_to_x_positive_orientation);
				servo_vel = geometry_msgs::msg::Twist();
				ctx_.state = std::string(" > ANGLE_TO_X_POSITIVE_ORIENTATION");
				ctx_.infos = std::string(
					"Reason: can not see marker more than time_sleep(default 5s) and navigate_state > ANGLE_TO_X_POSITIVE_ORIENTATION and robot.x > param robot_rotate_radius  ==> stop, change state to ANGLE_TO_X_POSITIVE_ORIENTATION");
				return emit(servo_vel);
			} else {
				ctx_.need_get_outof_charger_range = true;
				ctx_.get_out_of_charger_range_completed = false;
				RCLCPP_INFO_THROTTLE(ctx_.logger_, *ctx_.clock_, 1000,
					"robot cann't see the charger,but it is too linear to the charger , try to get robot out of charger range......");
				servo_vel = geometry_msgs::msg::Twist();
				ctx_.state = std::string(" > ANGLE_TO_X_POSITIVE_ORIENTATION");
				ctx_.infos = std::string(
					"Reason: can not see marker more than time_sleep(default 5s) and navigate_state > ANGLE_TO_X_POSITIVE_ORIENTATION and robot.x < param robot_rotate_radius  ==> stop and try to get robot out of charger range......");
				return emit(servo_vel);
			}
		}
	}

	// shared per-state timeout guard (INIT never times out)
	if (current_state()->checks_timeout() && ctx_.check_timeout()) {
		ctx_.b_timeout = true;
		current_state()->on_timeout(ctx_);
		transition(StateRank::INIT, 1.0);
		servo_vel = geometry_msgs::msg::Twist();
		return emit(servo_vel);
	}

	// deferred enter: run the state body once on its first tick
	StepResult result;
	if (need_enter_) {
		need_enter_ = false;
		result = current_state()->on_enter(ctx_);
		if (result.next_valid) {
			// on_enter transitioned immediately (e.g. INIT), no update this tick
			transition(result.next, result.next_timeout);
			return emit(result.twist);
		}
	}
	result = current_state()->on_update(ctx_);
	if (result.next_valid) {
		transition(result.next, result.next_timeout);
	}
	return emit(result.twist);
}

// ---------------------------------------------------------------------------
// StateCtx helpers
// ---------------------------------------------------------------------------

void StateGoalController::StateCtx::save_all_poses_infos()
{
	marker_visible_ = sees_dock;
	tf_robot_map_ = robot_pose_map;
	tf_robot_charger_ = current_pose;
	tf_charger_map_ = charger_pose_map;

	robot_x_map_ = tf_robot_map_.getOrigin().getX();
	robot_y_map_ = tf_robot_map_.getOrigin().getY();
	robot_yaw_map_ = tf2::getYaw(tf_robot_map_.getRotation());

	robot_x_charger_ = tf_robot_charger_.getOrigin().getX();
	robot_y_charger_ = tf_robot_charger_.getOrigin().getY();
	robot_yaw_charger_ = tf2::getYaw(tf_robot_charger_.getRotation());

	charger_x_map_ = tf_charger_map_.getOrigin().getX();
	charger_y_map_ = tf_charger_map_.getOrigin().getY();
	charger_yaw_map_ = tf2::getYaw(tf_charger_map_.getRotation());
}

void StateGoalController::StateCtx::update_time_smart()
{
	now_time_ = now_sec();
	delta_time_ = now_time_ - pre_time_;
	pre_time_ = now_time_;
	if (std::abs(delta_time_) > 1.5 / params->cmd_vel_hz) {
		RCLCPP_WARN(logger_, "error occurs, real delta_time: %.2f, hz is %d", delta_time_, params->cmd_vel_hz);
		delta_time_ = 1.0 / params->cmd_vel_hz;
	}
}

bool StateGoalController::StateCtx::check_timeout()
{
	now_time_ = now_sec();
	bool ret = (now_time_ - current_state_start_time_) > current_state_timeout_;
	RCLCPP_DEBUG(logger_, "now_time: %.2f, start_time: %.2f", now_time_, current_state_start_time_);
	RCLCPP_DEBUG(logger_, "delta_time: %.2f, timeout: %.2f", now_time_ - current_state_start_time_, current_state_timeout_);
	return ret;
}

void StateGoalController::StateCtx::bound_rotation(double & rotation_velocity, float min, float max) const
{
	double abs_rot = std::abs(rotation_velocity);
	if (abs_rot > max) {
		rotation_velocity = std::copysign(max, rotation_velocity);
	} else if (abs_rot < min) {
		rotation_velocity = std::copysign(min, rotation_velocity);
	}
}

float StateGoalController::StateCtx::generate_smooth_rotation_speed(float cur_rotation)
{
	float new_rotation_speed, rotation_max_change_abs, rotation_cur_change_abs;
	float acc = params->speed_rotation_acceleration;
	double cur_time = now_sec();
	double delta_time = cur_time - last_rotation_speed_time_;
	if (delta_time > 0.13) {  // 10hz
		first_pub_rotation_speed = true;
	} else {
		first_pub_rotation_speed = false;
	}

	if (first_pub_rotation_speed) {
		RCLCPP_DEBUG(logger_, "first pub rotation speed.");
		new_rotation_speed = std::copysign(params->speed_rotation_init_abs, cur_rotation);
	} else {
		rotation_max_change_abs = std::abs(delta_time * acc);
		rotation_cur_change_abs = std::abs(cur_rotation - last_rotation_speed_);
		if (rotation_cur_change_abs > rotation_max_change_abs) {
			new_rotation_speed = last_rotation_speed_ +
				std::copysign(rotation_max_change_abs, cur_rotation - last_rotation_speed_);
		} else {
			new_rotation_speed = cur_rotation;
		}
		RCLCPP_DEBUG(logger_, "last_rotation          : %.2f", last_rotation_speed_);
		RCLCPP_DEBUG(logger_, "cur_rotation           : %.2f", cur_rotation);
		RCLCPP_DEBUG(logger_, "delta_time             : %.2f", delta_time);
		RCLCPP_DEBUG(logger_, "rotation_max_change_abs: %.2f", rotation_max_change_abs);
		RCLCPP_DEBUG(logger_, "rotation_cur_change_abs: %.2f", rotation_cur_change_abs);
		RCLCPP_DEBUG(logger_, "new_rotation_speed     : %.2f", new_rotation_speed);
	}
	last_rotation_speed_ = new_rotation_speed;
	last_rotation_speed_time_ = cur_time;
	return new_rotation_speed;
}

double StateGoalController::StateCtx::diff_angle(const GoalPoint & goal_pt)
{
	double y = goal_pt.y - current_position.getY();
	double x = goal_pt.x - current_position.getX();
	double atan2_value = std::atan2(y, x);
	double result = angles::shortest_angular_distance(current_angle, atan2_value);
	RCLCPP_DEBUG(logger_, "dist    => %.2f", result);
	return result;
}

void StateGoalController::StateCtx::clear_local_costmap()
{
	bool ret = false;
	clear_time_now = now_sec();
	clear_time_delta = clear_time_now - clear_time_last;

	RCLCPP_DEBUG(logger_, "/local_costmap/clear_entirely_local_costmap time_now  : %.2f", clear_time_now);
	RCLCPP_DEBUG(logger_, "/local_costmap/clear_entirely_local_costmap time_last : %.2f", clear_time_last);
	RCLCPP_DEBUG(logger_, "/local_costmap/clear_entirely_local_costmap time_delta: %.2f", clear_time_delta);

	if (clear_time_delta > params->timout_clear_local_costmap) {
		auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
		ret = client_clear_entire_local_costmap->wait_for_service(0.05s);
		if (!ret) {
			RCLCPP_INFO(logger_, "/local_costmap/clear_entirely_local_costmap service not online.");
		} else {
			clear_time_last = clear_time_now;
			RCLCPP_INFO(logger_, "call service: /local_costmap/clear_entirely_local_costmap. (delta_time: %.2f, timout: %.2f)",
				clear_time_delta, params->timout_clear_local_costmap);
			client_clear_entire_local_costmap->async_send_request(request);
		}
	}
}

double StateGoalController::StateCtx::get_cost_value(
	tf2::Transform tf_robot, bool rotation, double linear, double angular, double predict_time)
{
	double cost_value = 0.0;
	double x, y, theta;
	tf2::Transform tf_offset;
	tf2::Transform tf_new;
	tf_offset.setIdentity();
	int counts_number = std::floor(predict_time * params->cmd_vel_hz);

	if (rotation) {
		double vel_angular = angular * params->odom_twist_scale;
		tf2::Quaternion q;
		for (int i = 0; i < counts_number; i++) {
			tf_offset.setOrigin(tf2::Vector3(0, 0, 0));
			double yaw = 1.0 / params->cmd_vel_hz * vel_angular * i;
			q.setRPY(0, 0, yaw);
			tf_offset.setRotation(q);
			tf_new = tf_robot * tf_offset;
			x = tf_new.getOrigin().getX();
			y = tf_new.getOrigin().getY();
			theta = tf2::getYaw(tf_new.getRotation());
			double cost_value_tmp = collision_checker.footprintCostAtPose(x, y, theta, footprint_vec);
			cost_value = std::max(cost_value, cost_value_tmp);
			if (cost_value >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
				return cost_value;
			}
		}
	} else {
		double vel_linear = linear * params->odom_twist_scale;
		for (int i = 0; i < counts_number; i++) {
			tf_offset.setOrigin(tf2::Vector3(1.0 / params->cmd_vel_hz * vel_linear * i, 0.0, 0.0));
			tf_new = tf_robot * tf_offset;
			x = tf_new.getOrigin().getX();
			y = tf_new.getOrigin().getY();
			theta = tf2::getYaw(tf_new.getRotation());
			double cost_value_tmp = collision_checker.footprintCostAtPose(x, y, theta, footprint_vec);
			cost_value = std::max(cost_value, cost_value_tmp);
			if (cost_value >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
				return cost_value;
			}
		}
		cost_value = vel_linear;
	}

	return cost_value;
}

double StateGoalController::StateCtx::get_cost_value_undock(
	tf2::Transform tf_robot, double linear, double predict_time)
{
	double x, y, theta;
	tf2::Transform tf_offset;
	tf2::Transform tf_new;
	tf_offset.setIdentity();
	int counts_number = std::floor(predict_time * params->cmd_vel_hz);
	RCLCPP_DEBUG(logger_, "counts: %d, predict_time: %.2f, hz: %d", counts_number, predict_time, params->cmd_vel_hz);
	double vel_linear = linear * params->odom_twist_scale;
	double footprint_cost = 0.0;

	// 生成不包含机器人后边的三条边
	std::vector<std::pair<geometry_msgs::msg::Point, geometry_msgs::msg::Point>> otherEdges;
	for (size_t i = 0; i < footprint_vec.size(); i++) {
		auto p1 = footprint_vec[i];
		auto p2 = footprint_vec[(i + 1) % footprint_vec.size()];
		if (params->undock_collision_check_front_only) {
			// 只保留机器人footprint中前边的边
			if (p1.x > 0 && p2.x > 0) {
				otherEdges.push_back(std::make_pair(p1, p2));
				RCLCPP_DEBUG(logger_, "edge %zu: Point(%.2f, %.2f) to Point(%.2f, %.2f)", i, p1.x, p1.y, p2.x, p2.y);
			}
		} else {
			double midX = (p1.x + p2.x) / 2.0;
			if (midX > 0) {
				otherEdges.push_back(std::make_pair(p1, p2));
				RCLCPP_DEBUG(logger_, "edge %zu: Point(%.2f, %.2f) to Point(%.2f, %.2f)", i, p1.x, p1.y, p2.x, p2.y);
			}
		}
	}

	for (int i = 0; i < counts_number; i++) {
		tf_offset.setOrigin(tf2::Vector3(1.0 / params->cmd_vel_hz * vel_linear * i, 0.0, 0.0));
		tf_new = tf_robot * tf_offset;
		x = tf_new.getOrigin().getX();
		y = tf_new.getOrigin().getY();
		theta = tf2::getYaw(tf_new.getRotation());
		for (size_t j = 0; j < otherEdges.size(); j++) {
			auto edge = otherEdges[j];
			auto p1 = edge.first;
			auto p2 = edge.second;
			geometry_msgs::msg::Point p1_transformed;
			geometry_msgs::msg::Point p2_transformed;
			p1_transformed.x = p1.x * cos(theta) - p1.y * sin(theta) + x;
			p1_transformed.y = p1.x * sin(theta) + p1.y * cos(theta) + y;
			p2_transformed.x = p2.x * cos(theta) - p2.y * sin(theta) + x;
			p2_transformed.y = p2.x * sin(theta) + p2.y * cos(theta) + y;
			unsigned int x0, x1, y0, y1;
			if (!collision_checker.worldToMap(p1_transformed.x, p1_transformed.y, x0, y0)) {
				return static_cast<double>(nav2_costmap_2d::NO_INFORMATION);
			}
			if (!collision_checker.worldToMap(p2_transformed.x, p2_transformed.y, x1, y1)) {
				return static_cast<double>(nav2_costmap_2d::NO_INFORMATION);
			}

			double cost_value_edge = 0.0;
			int collision_x = 0, collision_y = 0;
			double collision_x_map = 0.0, collision_y_map = 0.0;
			{
				double line_cost = 0.0;
				double point_cost = -1.0;
				for (nav2_util::LineIterator line(x0, y0, x1, y1); line.isValid(); line.advance()) {
					point_cost = collision_checker.pointCost(line.getX(), line.getY());
					// pub /marker_undock_point_loop
					double point_loop_x_map = 0.0, point_loop_y_map = 0.0;
					costmap.mapToWorld(line.getX(), line.getY(), point_loop_x_map, point_loop_y_map);
					visualization_msgs::msg::Marker marker_point;
					marker_point.header.frame_id = "map";
					marker_point.header.stamp = clock_->now();
					marker_point.ns = "point_loop";
					marker_point.id = 5;
					marker_point.type = visualization_msgs::msg::Marker::CUBE;
					marker_point.action = visualization_msgs::msg::Marker::ADD;
					marker_point.scale.x = 0.1;
					marker_point.scale.y = 0.1;
					marker_point.scale.z = 0.1;
					marker_point.color.r = 1.0;
					marker_point.color.g = 1.0;
					marker_point.color.b = 0.0;
					marker_point.color.a = 1.0;
					marker_point.pose.position.x = point_loop_x_map;
					marker_point.pose.position.y = point_loop_y_map;
					marker_undock_point_loop_pub_->publish(marker_point);

					if (point_cost == static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
						line_cost = point_cost;
						collision_x = line.getX();
						collision_y = line.getY();
						break;
					}
					if (line_cost < point_cost) {
						line_cost = point_cost;
					}
				}
				cost_value_edge = line_cost;
			}

			RCLCPP_DEBUG(logger_, "edge %zu cost_value: %.2f", j, cost_value_edge);
			footprint_cost = std::max(footprint_cost, cost_value_edge);
			if (footprint_cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) {
				// pub /marker_undock_collision_line
				visualization_msgs::msg::Marker marker_line;
				marker_line.header.frame_id = "map";
				marker_line.header.stamp = clock_->now();
				marker_line.ns = "collision_edges";
				marker_line.id = 3;
				marker_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
				marker_line.action = visualization_msgs::msg::Marker::ADD;
				marker_line.points.push_back(p1_transformed);
				marker_line.points.push_back(p2_transformed);
				marker_line.scale.x = 0.05;
				marker_line.color.r = 1.0;
				marker_line.color.g = 0.0;
				marker_line.color.b = 0.0;
				marker_line.color.a = 1.0;
				marker_undock_collision_line_pub_->publish(marker_line);

				// pub /marker_undock_collision_point
				visualization_msgs::msg::Marker marker_point;
				marker_point.header.frame_id = "map";
				marker_point.header.stamp = clock_->now();
				marker_point.ns = "collision_point";
				marker_point.id = 4;
				marker_point.type = visualization_msgs::msg::Marker::CUBE;
				marker_point.action = visualization_msgs::msg::Marker::ADD;
				marker_point.scale.x = 0.2;
				marker_point.scale.y = 0.2;
				marker_point.scale.z = 0.2;
				marker_point.color.r = 0.0;
				marker_point.color.g = 1.0;
				marker_point.color.b = 0.0;
				marker_point.color.a = 1.0;
				costmap.mapToWorld(collision_x, collision_y, collision_x_map, collision_y_map);
				marker_point.pose.position.x = collision_x_map;
				marker_point.pose.position.y = collision_y_map;
				marker_undock_collision_point_pub_->publish(marker_point);

				return footprint_cost;
			}
		}
	}
	return footprint_cost;
}

void StateGoalController::StateCtx::publish_marker(
	const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr & pub, int32_t id,
	const std::string & ns, int32_t type, double scale, float r, float g, float b, double x, double y)
{
	visualization_msgs::msg::Marker msg_marker;
	msg_marker.header.frame_id = "map";
	msg_marker.header.stamp = clock_->now();
	msg_marker.ns = ns;
	msg_marker.id = id;
	msg_marker.type = type;
	msg_marker.action = visualization_msgs::msg::Marker::ADD;
	msg_marker.scale.x = scale;
	msg_marker.scale.y = scale;
	msg_marker.scale.z = scale;
	msg_marker.color.r = r;
	msg_marker.color.g = g;
	msg_marker.color.b = b;
	msg_marker.color.a = 1.0;
	msg_marker.pose.position.x = x;
	msg_marker.pose.position.y = y;
	pub->publish(msg_marker);
}

bool StateGoalController::StateCtx::converged_position_3cond(double theta, double base_link_y) const
{
	return theta < thre_angle_diff &&
		std::abs(base_link_y) < params->base_link_y_thr &&
		std::abs(robot_x_charger_) > (params->offset_last_docked_distance + params->offset_low_speed +
			params->offset_second_goal + params->deviate_second_goal_x);
}

bool StateGoalController::StateCtx::skip_collision_near_charger(double & x_c2r, double & yaw_c2r_abs) const
{
	bool need_check_collision = true;
	auto tf_charger_to_robot = charger_pose_map.inverse() * robot_pose_map;
	auto translation_charger_to_robot = tf_charger_to_robot.getOrigin();
	x_c2r = std::abs(translation_charger_to_robot.getX());
	yaw_c2r_abs = std::abs(tf2::getYaw(tf_charger_to_robot.getRotation()));
	RCLCPP_DEBUG(logger_, "x_charge_to_robot: %.2f, dock_valid_obstacle_x: %.2f", x_c2r, params->dock_valid_obstacle_x);
	RCLCPP_DEBUG(logger_, "yaw_charger_to_robot: %.2f, throttle: %.2f", yaw_c2r_abs, M_PI * 0.5);
	if ((yaw_c2r_abs < M_PI * 0.5) && (x_c2r < params->dock_valid_obstacle_x)) {
		need_check_collision = false;
	}
	RCLCPP_DEBUG(logger_, "need_check_collision: %s", need_check_collision ? "true" : "false");
	return need_check_collision;
}

void StateGoalController::StateCtx::rotation_collision_block(
	geometry_msgs::msg::Twist & vel, double remaining_rot_angle, const char * state_name)
{
	double remaining_rotation_time = std::abs(remaining_rot_angle / vel.angular.z);
	double predict_time = std::min(double(params->collision_predict_time), remaining_rotation_time);
	RCLCPP_DEBUG(logger_, "predict_time: %.2f", predict_time);
	if (params->collision_check) {
		double cost_value = get_cost_value(robot_pose_map, true, 0.0, vel.angular.z, predict_time);
		if (cost_value >= nav2_costmap_2d::LETHAL_OBSTACLE) {
			RCLCPP_DEBUG(logger_, "cost value: %.2f >= %.2f", cost_value,
				static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));
			vel.angular.z = 0.0;
			RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "stop for collision check, when  %s", state_name);
			if (params->enable_clear_local_costmap) {
				clear_local_costmap();
			}
		}
	} else {
		RCLCPP_DEBUG(logger_, "collision_check: %s", params->collision_check ? "true" : "false");
	}
}

void StateGoalController::StateCtx::translation_collision_block(
	geometry_msgs::msg::Twist & vel, double remaining_dist, bool charger_dist_mode, const char * state_name)
{
	bool need_check_collision = true;
	double x_c2r = 0.0, yaw_c2r_abs = 0.0;
	if (params->collision_check) {
		need_check_collision = skip_collision_near_charger(x_c2r, yaw_c2r_abs);
	}

	double remaining_time;
	if (charger_dist_mode) {
		remaining_time = std::abs((x_c2r - params->dock_valid_obstacle_x) / vel.linear.x);
	} else {
		remaining_time = std::abs(remaining_dist / vel.linear.x);
	}
	double predict_time = std::min(double(params->collision_predict_time), remaining_time);
	RCLCPP_DEBUG(logger_, "predict_time: %.2f", predict_time);

	if (params->collision_check && need_check_collision) {
		double cost_value = get_cost_value(robot_pose_map, false, vel.linear.x, 0.0, predict_time);
		if (cost_value >= nav2_costmap_2d::LETHAL_OBSTACLE) {
			RCLCPP_DEBUG(logger_, "cost value: %.2f >= %.2f", cost_value,
				static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));
			vel.linear.x = 0.0;
			RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "stop for collision check, when %s", state_name);
			if (params->enable_clear_local_costmap) {
				clear_local_costmap();
			}
		}
	} else {
		RCLCPP_DEBUG(logger_, "collision_check: %s", params->collision_check ? "true" : "false");
		RCLCPP_DEBUG(logger_, "need_check_collision: %s", need_check_collision ? "true" : "false");
		RCLCPP_DEBUG(logger_, "yaw_c2r_abs: %.2f", yaw_c2r_abs);
		RCLCPP_DEBUG(logger_, "x_c2r: %.2f, dock_valid_obstacle_x: %.2f", x_c2r, params->dock_valid_obstacle_x);
	}
}

// ---------------------------------------------------------------------------
// state classes
// ---------------------------------------------------------------------------

StateGoalController::StepResult StateGoalController::InitState::on_enter(StateCtx & ctx)
{
	RCLCPP_DEBUG(ctx.logger_, "--------------- INIT ---------------");
	geometry_msgs::msg::Twist servo_vel;

	// pub agent发出的 /charger/pose位姿
	ctx.publish_marker(ctx.marker_charger_pose_agent_pub_, 1, "",
		visualization_msgs::msg::Marker::CUBE, 0.2, 0.0, 1.0, 0.0, ctx.charger_x_map_, ctx.charger_y_map_);
	RCLCPP_INFO(ctx.logger_, "publish topic /marker_charger_pose_agent");

	double dist_robot_to_charger = std::hypot(
		ctx.robot_x_map_ - ctx.charger_x_map_, ctx.robot_y_map_ - ctx.charger_y_map_);
	RCLCPP_INFO(ctx.logger_, "dist_robot_to_charger: %.2f", dist_robot_to_charger);

	if (ctx.params->garage_test && dist_robot_to_charger > 2.5) {
		RCLCPP_INFO(ctx.logger_, "garage_test mode trigger");
		ctx.drive_back = true;
		tf2::Transform tf_charger_to_buffer_point2;
		tf_charger_to_buffer_point2.setIdentity();
		tf_charger_to_buffer_point2.setOrigin(tf2::Vector3(
			ctx.params->offset_buffer_goal2_x, ctx.params->offset_buffer_goal2_y, 0.0));
		tf2::Transform tf_map_to_buffer_point2 = ctx.tf_charger_map_ * tf_charger_to_buffer_point2;

		ctx.buffer_point2_x_map = tf_map_to_buffer_point2.getOrigin().getX();
		ctx.buffer_point2_y_map = tf_map_to_buffer_point2.getOrigin().getY();
		RCLCPP_INFO(ctx.logger_, "buffer point2: (%.2f, %.2f)", ctx.buffer_point2_x_map, ctx.buffer_point2_y_map);

		ctx.publish_marker(ctx.marker_buffer_point2_pub_, 2, "",
			visualization_msgs::msg::Marker::SPHERE, 0.2, 1.0, 0.0, 0.0,
			ctx.buffer_point2_x_map, ctx.buffer_point2_y_map);
		RCLCPP_INFO(ctx.logger_, "publish topic /marker_buffer_point2");

		ctx.dist_buffer_point = std::hypot(
			ctx.robot_x_map_ - ctx.buffer_point2_x_map, ctx.robot_y_map_ - ctx.buffer_point2_y_map);
		double theta_buffer_point2_to_robot = std::atan2(
			ctx.robot_y_map_ - ctx.buffer_point2_y_map, ctx.robot_x_map_ - ctx.buffer_point2_x_map);
		ctx.dist_buffer_point_yaw = angles::shortest_angular_distance(
			ctx.robot_yaw_map_, theta_buffer_point2_to_robot);

		RCLCPP_INFO(ctx.logger_, "dist_buffer_point: %.2f, dis_buffer_point_yaw: %.2f",
			ctx.dist_buffer_point, ctx.dist_buffer_point_yaw);
		return StepResult::go(boost::make_optional(servo_vel),
			StateRank::ANGLE_TO_BUFFER_POINT, ctx.params->timeout_angle_to_buffer_point);
	}

	// garage_test = false
	return StepResult::go(boost::make_optional(servo_vel),
		StateRank::LOOKUP_MARKER, ctx.params->timeout_lookup_marker);
}

StateGoalController::StepResult StateGoalController::InitState::on_update(StateCtx & ctx)
{
	(void)ctx; // on_enter transitions immediately; this fallback is unreachable in normal flow
	geometry_msgs::msg::Twist servo_vel;
	return StepResult::stay(boost::make_optional(servo_vel));
}

StateGoalController::StepResult StateGoalController::LookupMarkerState::on_update(StateCtx & ctx)
{
	RCLCPP_DEBUG(ctx.logger_, "--------------- LOOKUP_MARKER ---------------");
	geometry_msgs::msg::Twist servo_vel;
	ctx.update_time_smart();

	auto angle_robot = tf2::getYaw(ctx.robot_pose_map.getRotation());
	double x_robot = ctx.robot_pose_map.getOrigin()[0];
	double y_robot = ctx.robot_pose_map.getOrigin()[1];
	double x_charger = ctx.charger_pose_map.getOrigin()[0];
	double y_charger = ctx.charger_pose_map.getOrigin()[1];
	auto angle_charger_to_robot = std::atan2(y_robot - y_charger, x_robot - x_charger);
	auto dist_angle = angles::shortest_angular_distance(angle_robot, angle_charger_to_robot);

	if (!ctx.sees_dock) {
		RCLCPP_DEBUG(ctx.logger_, "x_robot_map: %.2f, y_robot_map: %.2f", x_robot, y_robot);
		RCLCPP_DEBUG(ctx.logger_, "x_charger_map: %.2f, y_charger_map: %.2f", x_charger, y_charger);
		RCLCPP_DEBUG(ctx.logger_, "angle_robot: %.2f", angle_robot);
		RCLCPP_DEBUG(ctx.logger_, "angle_charger_to_robot: %.2f", angle_charger_to_robot);
		RCLCPP_DEBUG(ctx.logger_, "dist_angle: %.2f", dist_angle);

		ctx.start_time_recorded = false;
		RCLCPP_DEBUG(ctx.logger_, "Need rotate robot for it can see the marker.");

		if (std::abs(dist_angle) > ctx.params->tolerance_angle) {
			double dist_angle_copy = dist_angle;
			ctx.bound_rotation(dist_angle_copy, ctx.params->min_rotation, ctx.params->max_rotation);
			servo_vel.angular.z = dist_angle_copy;
			RCLCPP_DEBUG(ctx.logger_, "angular_z: %.2f", servo_vel.angular.z);
		} else {
			servo_vel.angular.z = 0.0;
			ctx.state = std::string("LOOKUP_MARKER");
			ctx.infos = std::string(
				"Reason: The robot has rotated towards the charging station direction, but it can not see the marker!");
			RCLCPP_WARN_THROTTLE(ctx.logger_, *ctx.clock_, 3000, "%s", ctx.infos.c_str());
		}

		// before actually begin rotation, collision_check first
		ctx.rotation_collision_block(servo_vel, dist_angle, name());

		ctx.state = std::string("LOOKUP_MARKER");
		ctx.infos = std::string("Reason: camera's orientation not towards charger's marker ==> rotate robot");
	} else {
		RCLCPP_DEBUG(ctx.logger_, "waiting for find the best coordinate for robot in marker frame");
		double x_, y_, theta_;
		x_ = ctx.current_pose.getOrigin()[0];
		y_ = ctx.current_pose.getOrigin()[1];
		theta_ = tf2::getYaw(ctx.current_pose.getRotation());
		RCLCPP_DEBUG(ctx.logger_, "robot_x: %.2f", x_);
		RCLCPP_DEBUG(ctx.logger_, "robot_y: %.2f", y_);
		RCLCPP_DEBUG(ctx.logger_, "robot_theta: %.2f", theta_);

		if (!ctx.start_time_recorded) {
			ctx.start_time_recorded = true;
			ctx.waiting_for_best_coord_start_time = ctx.now_sec();
			RCLCPP_DEBUG(ctx.logger_, "waiting_for_best_coord_start_time : %.2f",
				ctx.waiting_for_best_coord_start_time);
			ctx.state = std::string("LOOKUP_MARKER");
			ctx.infos = std::string("Reason: camera's orientation first towards charger's marker ==> record the start time.");
		} else {
			ctx.now_time_ = ctx.now_sec();
			double time_wating = ctx.now_time_ - ctx.waiting_for_best_coord_start_time;

			if (time_wating < ctx.params->localization_converged_time) {
				RCLCPP_DEBUG(ctx.logger_, "coords not converged, wait %.2f seconds.", time_wating);
				ctx.state = std::string("LOOKUP_MARKER");
				ctx.infos = std::string("Reason: coords not converged ==> just wait.");
			} else {
				RCLCPP_DEBUG(ctx.logger_, "coords converged, change state.");
				ctx.start_time_recorded = false;

				float distance_tmp = ctx.params->offset_last_docked_distance
					+ ctx.params->offset_low_speed
					+ ctx.params->offset_second_goal;
				double theta = std::atan2(
					std::abs(ctx.robot_y_charger_), std::abs(ctx.robot_x_charger_) - distance_tmp);
				RCLCPP_DEBUG(ctx.logger_, "robot_x_charger: %.2f", ctx.robot_x_charger_);
				RCLCPP_DEBUG(ctx.logger_, "robot_y_charger: %.2f", ctx.robot_y_charger_);
				RCLCPP_DEBUG(ctx.logger_, "robot_yaw_charger: %.2f", ctx.robot_yaw_charger_);

				double base_link_y, base_link_x;
				base_link_y = ctx.robot_y_charger_ - ctx.params->base_link_dummy_dis * std::sin(ctx.robot_yaw_charger_);
				base_link_x = ctx.robot_x_charger_ - ctx.params->base_link_dummy_dis * std::cos(ctx.robot_yaw_charger_);

				RCLCPP_DEBUG(ctx.logger_, "base_link_x: %.2f", base_link_x);
				RCLCPP_DEBUG(ctx.logger_, "base_link_y: %.2f", base_link_y);
				RCLCPP_DEBUG(ctx.logger_, "theta: %.2f", theta);
				RCLCPP_DEBUG(ctx.logger_, "thre_angle_diff: %.2f", ctx.thre_angle_diff);
				RCLCPP_DEBUG(ctx.logger_, "std::abs(base_link_y): %.2f", std::abs(base_link_y));
				RCLCPP_DEBUG(ctx.logger_, " params_ptr->base_link_y_thr: %.2f", ctx.params->base_link_y_thr);
				RCLCPP_DEBUG(ctx.logger_, "std::abs(robot_x_charger_): %.2f", std::abs(ctx.robot_x_charger_));
				RCLCPP_DEBUG(ctx.logger_, "distance_tmp + params_ptr->deviate_second_goal_x: %.2f",
					distance_tmp + ctx.params->deviate_second_goal_x);

				if (ctx.converged_position_3cond(theta, base_link_y)) {
					RCLCPP_DEBUG(ctx.logger_, "robot change state to angle_to_goal");
					ctx.state = std::string("LOOKUP_MARKER");
					ctx.infos = std::string("Reason: robot's position converged ==> directly change state to ANGLE_TO_GOAL");
					return StepResult::go(boost::make_optional(servo_vel),
						StateRank::ANGLE_TO_GOAL, ctx.params->timeout_angle_to_goal);
				} else {
					RCLCPP_DEBUG(ctx.logger_, "robot change state to angle_to_buffer_point");
					RCLCPP_DEBUG(ctx.logger_, "buffer_goal_point_x: %.2f, buffer_goal_point_y: %.2f",
						ctx.buffer_goal_point_x, ctx.buffer_goal_point_y);

					ctx.tf_before_angle_to_buffer_point = ctx.robot_pose_map;

					double buffer_goal_point_x_base_link = ctx.buffer_goal_point_x - ctx.params->base_link_dummy_dis;
					double buffer_goal_point_y_base_link = ctx.buffer_goal_point_y;
					RCLCPP_DEBUG(ctx.logger_, "buffer_goal_point_x_base_link: %.2f, buffer_goal_point_y_base_link: %.2f",
						buffer_goal_point_x_base_link, buffer_goal_point_y_base_link);

					ctx.dist_buffer_point = std::hypot(base_link_x - buffer_goal_point_x_base_link,
						base_link_y - buffer_goal_point_y_base_link);
					ctx.dist_move_to_buffer_point = ctx.dist_buffer_point;

					ctx.robot_angle_to_buffer_point_yaw = std::atan2(
						buffer_goal_point_y_base_link - base_link_y,
						buffer_goal_point_x_base_link - base_link_x);

					ctx.robot_current_yaw = ctx.robot_yaw_charger_;
					ctx.theta_positive = angles::shortest_angular_distance(
						angles::normalize_angle(ctx.robot_current_yaw + M_PI),
						ctx.robot_angle_to_buffer_point_yaw);
					ctx.theta_negative = angles::shortest_angular_distance(
						ctx.robot_current_yaw, ctx.robot_angle_to_buffer_point_yaw);
					if (std::abs(ctx.theta_positive) < std::abs(ctx.theta_negative)) {
						ctx.drive_back = false;
						ctx.dist_buffer_point_yaw = ctx.theta_positive;
					} else {
						ctx.drive_back = true;
						ctx.dist_buffer_point_yaw = ctx.theta_negative;
					}

					ctx.theta_angle_to_buffer_point = ctx.dist_buffer_point_yaw;

					RCLCPP_DEBUG(ctx.logger_, "robot_current_yaw: %.2f", ctx.robot_current_yaw);
					RCLCPP_DEBUG(ctx.logger_, "robot_angle_to_buffer_point_yaw: %.2f", ctx.robot_angle_to_buffer_point_yaw);
					RCLCPP_DEBUG(ctx.logger_, "theta_negative: %.2f", ctx.theta_negative);
					RCLCPP_DEBUG(ctx.logger_, "theta_positive: %.2f", ctx.theta_positive);
					RCLCPP_DEBUG(ctx.logger_, "dist_buffer_point: %.2f", ctx.dist_buffer_point);
					RCLCPP_DEBUG(ctx.logger_, "dist_buffer_point_yaw: %.2f", ctx.dist_buffer_point_yaw);

					ctx.state = std::string("LOOKUP_MARKER");
					ctx.infos = std::string("Reason: robot's position not converged ==> directly change state to ANGLE_TO_BUFFER_POINT");
					return StepResult::go(boost::make_optional(servo_vel),
						StateRank::ANGLE_TO_BUFFER_POINT, ctx.params->timeout_angle_to_buffer_point);
				}
			}
		}
	}
	return StepResult::stay(boost::make_optional(servo_vel));
}

StateGoalController::StepResult StateGoalController::AngleToBufferPointState::on_update(StateCtx & ctx)
{
	RCLCPP_DEBUG(ctx.logger_, "--------------- ANGLE_TO_BUFFER_POINT ---------------");
	geometry_msgs::msg::Twist servo_vel;
	ctx.update_time_smart();

	RCLCPP_DEBUG(ctx.logger_, "delta_time: %.2f", ctx.delta_time_);
	RCLCPP_DEBUG(ctx.logger_, "angular.z: %.2f", ctx.odom_msg.twist.twist.angular.z);
	RCLCPP_DEBUG(ctx.logger_, "delta_angular: %.2f", ctx.odom_msg.twist.twist.angular.z * ctx.delta_time_);
	RCLCPP_DEBUG(ctx.logger_, "dist_buffer_point_yaw pre: %.2f", ctx.dist_buffer_point_yaw);
	ctx.dist_buffer_point_yaw -= ctx.odom_msg.twist.twist.angular.z * ctx.delta_time_;
	ctx.robot_current_yaw += ctx.odom_msg.twist.twist.angular.z * ctx.delta_time_;
	RCLCPP_DEBUG(ctx.logger_, "dist_buffer_point_yaw now: %.2f", ctx.dist_buffer_point_yaw);

	double angle_dist = ctx.dist_buffer_point_yaw;
	RCLCPP_DEBUG(ctx.logger_, "angle_dist: %.2f", angle_dist);
	RCLCPP_DEBUG(ctx.logger_, "robot_map_yaw: %.2f", tf2::getYaw(ctx.robot_pose_map.getRotation()));

	if (std::abs(angle_dist) < ctx.params->tolerance_angle) {
		RCLCPP_DEBUG(ctx.logger_, "change state to move_to_buffer_point.");
		ctx.tf_after_angle_to_buffer_point = ctx.robot_pose_map;
		double theta_delta_map = tf2::getYaw(
			(ctx.tf_before_angle_to_buffer_point.inverse() * ctx.tf_after_angle_to_buffer_point).getRotation());
		RCLCPP_DEBUG(ctx.logger_, "dist_angle_to_buffer_point: %.2f, theta_delta: %.2f",
			ctx.theta_angle_to_buffer_point, theta_delta_map);
		ctx.state = std::string("ANGLE_TO_BUFFER_POINT");
		ctx.infos = std::string("Reason: ANGLE_TO_BUFFER_POINT converged ==> change state to MOVE_TO_BUFFER_POINT");
		return StepResult::go(boost::make_optional(servo_vel),
			StateRank::MOVE_TO_BUFFER_POINT, ctx.params->timeout_move_to_buffer_point);
	}

	// static footprint check (distinct from the ray-cast block below)
	double x = ctx.robot_pose_map.getOrigin().getX();
	double y = ctx.robot_pose_map.getOrigin().getY();
	double theta = tf2::getYaw(ctx.robot_pose_map.getRotation());
	double cost = ctx.collision_checker.footprintCostAtPose(x, y, theta, ctx.footprint_vec);
	RCLCPP_DEBUG(ctx.logger_, "x: %.2f, y: %.2f, theta: %.2f", x, y, theta);
	if ((cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) && ctx.params->collision_check) {
		RCLCPP_DEBUG(ctx.logger_, "cost value: %.2f >= %.2f", cost,
			static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));
		servo_vel.angular.z = 0.0;
		return StepResult::stay(boost::make_optional(servo_vel));
	} else {
		RCLCPP_DEBUG(ctx.logger_, "cost value: %.2f, go on ......", cost);
		RCLCPP_DEBUG(ctx.logger_, "collision_check: %s", ctx.params->collision_check ? "true" : "false");
	}

	ctx.bound_rotation(angle_dist, ctx.params->min_rotation, ctx.params->max_rotation);
	if (std::abs(angle_dist) < ctx.params->min_rotation) {
		angle_dist = std::copysign(ctx.params->min_rotation, angle_dist);
	}
	servo_vel.angular.z = angle_dist;
	RCLCPP_DEBUG(ctx.logger_, "angular.z: %.2f", angle_dist);

	// before actually begin rotation, collision_check first
	ctx.rotation_collision_block(servo_vel, ctx.dist_buffer_point_yaw, name());

	ctx.state = std::string("ANGLE_TO_BUFFER_POINT");
	ctx.infos = std::string("Reason: ANGLE_TO_BUFFER_POINT not converged ==> keep on rotating robot");
	return StepResult::stay(boost::make_optional(servo_vel));
}

StateGoalController::StepResult StateGoalController::MoveToBufferPointState::on_update(StateCtx & ctx)
{
	RCLCPP_DEBUG(ctx.logger_, "--------------- MOVE_TO_BUFFER_POINT ---------------");
	geometry_msgs::msg::Twist servo_vel;
	ctx.update_time_smart();

	ctx.dist_buffer_point -= ctx.delta_time_ * std::abs(ctx.odom_msg.twist.twist.linear.x);
	double dist_y = ctx.dist_buffer_point;
	RCLCPP_DEBUG(ctx.logger_, "odom_msg.linear_x: %.2f", ctx.odom_msg.twist.twist.linear.x);
	RCLCPP_DEBUG(ctx.logger_, "delta_time: %.2f", ctx.delta_time_);
	RCLCPP_DEBUG(ctx.logger_, "dist_buffer_point now: %.2f", ctx.dist_buffer_point);
	RCLCPP_DEBUG(ctx.logger_, "dist_y: %.2f", dist_y);
	RCLCPP_DEBUG(ctx.logger_, "robot_map_x: %.2f, robot_map_y: %.2f",
		ctx.robot_pose_map.getOrigin().getX(), ctx.robot_pose_map.getOrigin().getY());

	if (std::abs(dist_y) < ctx.params->tolerance_r) {
		RCLCPP_DEBUG(ctx.logger_, "change state to angle_to_x_positive_orientation");
		ctx.tf_after_move_to_buffer_point = ctx.robot_pose_map;
		tf2::Transform tf_ = ctx.tf_after_angle_to_buffer_point.inverse() * ctx.tf_after_move_to_buffer_point;
		double dist_move_to_buffer_point_delta = std::hypot(tf_.getOrigin().getX(), tf_.getOrigin().getY());
		RCLCPP_DEBUG(ctx.logger_, "dist_move_to_buffer_point: %.2f, dist_delta: %.2f",
			ctx.dist_move_to_buffer_point, dist_move_to_buffer_point_delta);
		ctx.state = std::string("MOVE_TO_BUFFER_POINT");
		ctx.infos = std::string("Reason: MOVE_TO_BUFFER_POINT converged ==> change state to ANGLE_TO_X_POSITIVE_ORIENTATION");
		return StepResult::go(boost::make_optional(servo_vel),
			StateRank::ANGLE_TO_X_POSITIVE_ORIENTATION, ctx.params->timeout_angle_to_x_positive_orientation);
	}

	double translate_velocity = dist_y;
	if (ctx.drive_back) {
		translate_velocity *= -1;
	}
	if (std::abs(translate_velocity) > ctx.params->max_translation) {
		translate_velocity = std::copysign(ctx.params->max_translation, translate_velocity);
	}
	if (std::abs(translate_velocity) < ctx.params->min_translation) {
		translate_velocity = std::copysign(ctx.params->min_translation, translate_velocity);
	}
	servo_vel.linear.x = translate_velocity;
	RCLCPP_DEBUG(ctx.logger_, "linear.x: : %.2f", translate_velocity);

	if (ctx.params->garage_test && ctx.dist_buffer_point > 0.2) {
		auto theta_buffer_point2_to_robot_current = std::atan2(
			ctx.robot_y_map_ - ctx.buffer_point2_y_map, ctx.robot_x_map_ - ctx.buffer_point2_x_map);
		auto dist_buffer_point_yaw_now = angles::shortest_angular_distance(
			ctx.robot_yaw_map_, theta_buffer_point2_to_robot_current);
		ctx.bound_rotation(dist_buffer_point_yaw_now,
			ctx.params->go_to_goal_rotation_min, ctx.params->go_to_goal_rotation_max);
		servo_vel.angular.z = dist_buffer_point_yaw_now;
	}

	// before actually begin moving, collision_check first
	ctx.translation_collision_block(servo_vel, ctx.dist_buffer_point, false, name());

	ctx.state = std::string("MOVE_TO_BUFFER_POINT");
	ctx.infos = std::string("Reason: MOVE_TO_BUFFER_POINT not converged ==> keep on moving");
	return StepResult::stay(boost::make_optional(servo_vel));
}

StateGoalController::StepResult
	StateGoalController::AngleToXPositiveOrientationState::on_update(StateCtx & ctx)
{
	RCLCPP_DEBUG(ctx.logger_, "--------------- ANGLE_TO_X_POSITIVE_ORIENTATION ---------------");
	geometry_msgs::msg::Twist servo_vel;
	ctx.update_time_smart();

	double theta_charger_to_robot = std::atan2(
		ctx.robot_y_map_ - ctx.charger_y_map_, ctx.robot_x_map_ - ctx.charger_x_map_);
	double dist_yaw_map = angles::shortest_angular_distance(ctx.robot_yaw_map_, theta_charger_to_robot);

	RCLCPP_DEBUG(ctx.logger_, "robot_yaw_map_: %.2f", ctx.robot_yaw_map_);
	RCLCPP_DEBUG(ctx.logger_, "theta_charger_to_robot: %.2f", theta_charger_to_robot);
	RCLCPP_DEBUG(ctx.logger_, "dist_yaw_map: %.2f", dist_yaw_map);
	RCLCPP_DEBUG(ctx.logger_, "delta_time: %.2f", ctx.delta_time_);
	RCLCPP_DEBUG(ctx.logger_, "marker_visible: %s", ctx.sees_dock ? "true" : "false");

	if (std::abs(dist_yaw_map) < ctx.params->tolerance_angle) {
		if (ctx.marker_visible_) {
			float distance_tmp = ctx.params->offset_last_docked_distance
				+ ctx.params->offset_low_speed
				+ ctx.params->offset_second_goal;
			double theta = std::atan2(
				std::abs(ctx.robot_y_charger_), std::abs(ctx.robot_x_charger_) - distance_tmp);
			RCLCPP_DEBUG(ctx.logger_, "robot_x_charger: %.2f", ctx.robot_x_charger_);
			RCLCPP_DEBUG(ctx.logger_, "robot_y_charger: %.2f", ctx.robot_y_charger_);
			RCLCPP_DEBUG(ctx.logger_, "robot_yaw_charger_: %.2f", ctx.robot_yaw_charger_);

			double base_link_y = ctx.robot_y_charger_ -
				ctx.params->base_link_dummy_dis * std::sin(ctx.robot_yaw_charger_);
			RCLCPP_DEBUG(ctx.logger_, "base_link_y: %.2f", base_link_y);
			RCLCPP_DEBUG(ctx.logger_, "theta: %.2f", theta);
			RCLCPP_DEBUG(ctx.logger_, "thre_angle_diff: %.2f", ctx.thre_angle_diff);
			RCLCPP_DEBUG(ctx.logger_, "std::abs(base_link_y): %.2f", std::abs(base_link_y));
			RCLCPP_DEBUG(ctx.logger_, " params_ptr->base_link_y_thr: %.2f", ctx.params->base_link_y_thr);
			RCLCPP_DEBUG(ctx.logger_, "std::abs(robot_x_charger_): %.2f", std::abs(ctx.robot_x_charger_));
			RCLCPP_DEBUG(ctx.logger_, "distance_tmp + params_ptr->deviate_second_goal_x: %.2f",
				distance_tmp + ctx.params->deviate_second_goal_x);

			if (ctx.converged_position_3cond(theta, base_link_y)) {
				RCLCPP_INFO(ctx.logger_, "converged ==>Change state to ANGLE_TO_GOAL");
				ctx.state = std::string("ANGLE_TO_X_POSITIVE_ORIENTATION => ANGLE_TO_GOAL");
				ctx.infos = std::string("Reason: ANGLE_TO_X_POSITIVE_ORIENTATION converged ==> change state to ANGLE_TO_GOAL");
				return StepResult::go(boost::make_optional(servo_vel),
					StateRank::ANGLE_TO_GOAL, ctx.params->timeout_angle_to_goal);
			} else {
				RCLCPP_INFO(ctx.logger_, "To re-execute ANGLE_TO_BUFFER_POINT, change state to LOOKUP_MARKER");
				ctx.state = std::string("ANGLE_TO_X_POSITIVE_ORIENTATION => LOOKUP_MARKER");
				ctx.infos = std::string(
					"Reason: ANGLE_TO_X_POSITIVE_ORIENTATION not converged ==> change state to LOOKUP_MARKER");
				return StepResult::go(boost::make_optional(servo_vel),
					StateRank::LOOKUP_MARKER, ctx.params->timeout_lookup_marker);
			}
		} else {  // marker_visible: false
			RCLCPP_INFO_THROTTLE(ctx.logger_, *ctx.clock_, 1000,
				"current_ state: %s, can not see the marker, just waiting ...", name());
			return StepResult::stay(boost::make_optional(servo_vel));
		}
	}

	// std::abs(dist_yaw_map) >= tolerance_angle
	ctx.bound_rotation(dist_yaw_map, ctx.params->min_rotation, ctx.params->max_rotation);
	servo_vel.angular.z = dist_yaw_map;

	// before actually begin rotation, collision_check first
	ctx.rotation_collision_block(servo_vel, dist_yaw_map, name());

	RCLCPP_DEBUG(ctx.logger_, "angular.z: %.2f", servo_vel.angular.z);
	return StepResult::stay(boost::make_optional(servo_vel));
}

StateGoalController::StepResult StateGoalController::AngleToGoalState::on_update(StateCtx & ctx)
{
	RCLCPP_DEBUG(ctx.logger_, "--------------- ANGLE_TO_GOAL ---------------");
	geometry_msgs::msg::Twist servo_vel;
	ctx.update_time_smart();

	const GoalPoint & gp = ctx.goal_points_.front();

	RCLCPP_DEBUG(ctx.logger_, "goal =>  x: %.2f, y: %.2f, yaw: %.2f", gp.x, gp.y, gp.theta);
	RCLCPP_DEBUG(ctx.logger_, "robot =>  x: %.2f, y: %.2f, yaw: %.2f",
		ctx.current_position.getX(), ctx.current_position.getY(), ctx.current_angle);

	double delta_y, delta_x;
	delta_y = std::abs(gp.y - ctx.current_position.getY());
	delta_x = std::abs(gp.x - ctx.current_position.getX());
	double dist_to_goal = std::hypot(delta_x, delta_y);
	if (dist_to_goal <= gp.radius || delta_y < ctx.params->dist_error_y_1) {
		ctx.pose_x_init_recoreded_ = false;
		ctx.state = std::string("ANGLE_TO_GOAL");
		ctx.infos = std::string("Reason: ANGLE_TO_GOAL converged ==> change state to GO_TO_GOAL_POSITION");
		return StepResult::go(boost::make_optional(servo_vel),
			StateRank::GO_TO_GOAL_POSITION, ctx.params->timeout_go_to_goal_position);
	}

	double ang = ctx.diff_angle(gp);
	double ang_save = ang;
	RCLCPP_DEBUG(ctx.logger_, "diff angle: %.2f", ang);
	ctx.bound_rotation(ang, 0.05, 0.10);
	RCLCPP_DEBUG(ctx.logger_, "bound angle: %.2f", ang);

	// fix bug when robot had angle to marker but y coord error or odom data error,  10 degree(0.174533)
	if (std::abs(ang_save) < ctx.params->angle_to_goal_angle_converged ||
		(ctx.sees_dock && std::abs(std::abs(ctx.current_angle) - M_PI) < 0.174533)) {
		ctx.pose_x_init_recoreded_ = false;
		RCLCPP_DEBUG(ctx.logger_, " ******** change to state GO_TO_GOAL_POSITION ******** ");
		ctx.state = std::string("ANGLE_TO_GOAL");
		ctx.infos = std::string("Reason: ANGLE_TO_GOAL converged ==> change state to GO_TO_GOAL_POSITION");
		return StepResult::go(boost::make_optional(servo_vel),
			StateRank::GO_TO_GOAL_POSITION, ctx.params->timeout_go_to_goal_position);
	}

	servo_vel.angular.z = ang;
	ctx.state = std::string("ANGLE_TO_GOAL");
	ctx.infos = std::string("Reason: ANGLE_TO_GOAL not converged ==> keep on rotating");
	return StepResult::stay(boost::make_optional(servo_vel));
}

StateGoalController::StepResult StateGoalController::GoToGoalPositionState::on_update(StateCtx & ctx)
{
	RCLCPP_DEBUG(ctx.logger_, "--------------- GO_TO_GOAL_POSITION ---------------");
	geometry_msgs::msg::Twist servo_vel;
	ctx.update_time_smart();

	GoalPoint gp = ctx.goal_points_.front();
	if (ctx.goal_points_.size() > 1) {
		RCLCPP_DEBUG(ctx.logger_, "not the first goal.");
	} else {
		RCLCPP_DEBUG(ctx.logger_, "the first goal.");
		gp.x = -(ctx.params->offset_last_docked_distance - 0.02);
	}

	if (!ctx.pose_x_init_recoreded_) {
		ctx.pose_x_init_ = ctx.current_pose.getOrigin().getX();
		ctx.pose_x_init_recoreded_ = true;
		RCLCPP_DEBUG(ctx.logger_, "recored the pose_x: %.2f", ctx.pose_x_init_);
	}

	RCLCPP_DEBUG(ctx.logger_, "goal =>  x: %.2f, y: %.2f, yaw: %.2f", gp.x, gp.y, gp.theta);
	RCLCPP_DEBUG(ctx.logger_, "robot =>  x: %.2f, y: %.2f, yaw: %.2f degree.",
		ctx.current_pose.getOrigin().getX(), ctx.current_pose.getOrigin().getY(), ctx.current_angle / 3.1415926 * 180.0);

	double delta_y, delta_x;
	delta_y = std::abs(gp.y - ctx.current_position.getY());
	delta_x = std::abs(gp.x - ctx.current_position.getX());
	double dist_to_goal = std::hypot(delta_x, delta_y);
	double ang = ctx.diff_angle(gp);

	double abs_ang = std::abs(ang);
	double translate_velocity = ctx.params->go_to_goal_translation_max;

	auto robot_abs_x = std::abs(ctx.current_position.getX());
	auto dist_low_speed = ctx.params->offset_last_docked_distance + ctx.params->offset_low_speed;
	auto dist_speed_down_length = (ctx.params->go_to_goal_translation_max + ctx.params->go_to_goal_translation_min) / 2.0 *
		((ctx.params->go_to_goal_translation_max - ctx.params->go_to_goal_translation_min) /
			ctx.params->go_to_goal_linear_acc);
	auto dist_speed_down_range = ctx.params->go_to_goal_translation_max - ctx.params->go_to_goal_translation_min;
	auto dist_speed_down = dist_low_speed + dist_speed_down_length;

	if (robot_abs_x >= std::abs(ctx.pose_x_init_)) {  // linear_low_speed(0.05)
		translate_velocity = ctx.params->go_to_goal_translation_min;
	}
	if ((robot_abs_x > dist_speed_down) && (robot_abs_x < std::abs(ctx.pose_x_init_))) {
		double max_linear_speed = ctx.params->go_to_goal_translation_max;
		translate_velocity = std::min(ctx.params->go_to_goal_translation_min +
			(std::abs(ctx.pose_x_init_) - robot_abs_x) / dist_speed_down_length * dist_speed_down_range,
			max_linear_speed);
	}
	if ((robot_abs_x <= dist_speed_down) && robot_abs_x >= dist_low_speed) {
		translate_velocity = ctx.params->go_to_goal_translation_max -
			(dist_speed_down - robot_abs_x) / dist_speed_down_length * dist_speed_down_range;
	}
	if (robot_abs_x < dist_low_speed) {  // linear_low_speed(0.05)
		translate_velocity = ctx.params->go_to_goal_translation_min;
	}

	// If robot is close enough to goal, move to final stage
	if (dist_to_goal < ctx.goal_points_.front().radius || std::abs(ctx.current_position.getX()) < std::abs(gp.x)) {
		RCLCPP_DEBUG(ctx.logger_, " ******** change to state GOAL_ANGLE ******** ");
		servo_vel.linear.x = gp.drive_backwards ? -translate_velocity : translate_velocity;
		RCLCPP_DEBUG(ctx.logger_, "linear_x: %.2f", servo_vel.linear.x);
		ctx.state = std::string("GO_TO_GOAL_POSITION");
		ctx.infos = std::string("GO_TO_GOAL_POSITION converged ==> change state to GOAL_ANGLE");
		return StepResult::go(boost::make_optional(servo_vel),
			StateRank::GOAL_ANGLE, ctx.params->timeout_goal_angle);
	}

	if (gp.drive_backwards) {
		translate_velocity *= -1;
	}

	if (std::abs(ctx.current_position.getX()) <
		(ctx.params->offset_last_docked_distance + ctx.params->offset_low_speed)) {
		RCLCPP_DEBUG(ctx.logger_, "low speed mode ");
		if (!ctx.bluetooth_connected) {
			RCLCPP_INFO_THROTTLE(ctx.logger_, *ctx.clock_, 2000,
				"bluetooth disconnected, waiting ......");
			ctx.state = std::string("GO_TO_GOAL_POSITION");
			ctx.infos = std::string("Reason: bluetooth disconnected ==> stop");
			return StepResult::stay(boost::make_optional(servo_vel));
		}

		servo_vel.linear.x = translate_velocity;

		if (std::abs(ctx.current_position.getX()) <
			(ctx.params->offset_last_docked_distance + ctx.params->last_goal_angle_to_x_positive_dis)) {
			double ang2 = angles::shortest_angular_distance(ctx.current_angle, 0);
			RCLCPP_DEBUG(ctx.logger_, "ang2: %.2f", ang2);
			if (ang2 < 0 && std::abs(ang2) > ctx.params->go_to_goal_apply_rotation_angle &&
				ctx.current_position.getY() > -ctx.params->last_goal_angle_to_x_positive_y) {
				RCLCPP_DEBUG(ctx.logger_, "ang2: %.2f, y: %.2f, angle_to_x_positive direction", ang2, ctx.current_position.getY());
				servo_vel.linear.x = -0.05;
				ang = ang2;
			} else if (ang2 > 0 && std::abs(ang2) > ctx.params->go_to_goal_apply_rotation_angle &&
				ctx.current_position.getY() < ctx.params->last_goal_angle_to_x_positive_y) {
				RCLCPP_DEBUG(ctx.logger_, "ang2: %.2f, y: %.2f, angle_to_x_positive direction", ang2, ctx.current_position.getY());
				servo_vel.linear.x = -0.05;
				ang = ang2;
			}
		}
		ctx.bound_rotation(ang, ctx.params->go_to_goal_rotation_min, ctx.params->go_to_goal_rotation_max);
		ang = ctx.generate_smooth_rotation_speed(ang);
		servo_vel.angular.z = ang;
		ctx.last_rotation_speed_ = ang;

		ctx.state = std::string("GO_TO_GOAL_POSITION");
		ctx.infos = std::string("GO_TO_GOAL_POSITION (low speed mode) ==> keep on moving");
	} else {
		RCLCPP_DEBUG(ctx.logger_, "normal speed mode ");
		RCLCPP_DEBUG(ctx.logger_, "diff angle_to_goal: %.2f", ang);
		RCLCPP_DEBUG(ctx.logger_, "abs_angle: %.2f", abs_ang);
		RCLCPP_DEBUG(ctx.logger_, "thre: %.2f", ctx.params->go_to_goal_apply_rotation_angle);
		if (abs_ang > ctx.params->go_to_goal_apply_rotation_angle) {
			RCLCPP_DEBUG(ctx.logger_, "Need adjust direction.");
			ctx.bound_rotation(ang, ctx.params->go_to_goal_rotation_min, ctx.params->go_to_goal_rotation_max);
			ang = ctx.generate_smooth_rotation_speed(ang);
			servo_vel.angular.z = ang;
			ctx.last_rotation_speed_ = ang;
			ctx.state = std::string("GO_TO_GOAL_POSITION");
			ctx.infos = std::string("GO_TO_GOAL_POSITION (normal speed mode) ==> keep on moving");
		} else {
			RCLCPP_DEBUG(ctx.logger_, "Don't need adjust direction.");
		}
		servo_vel.linear.x = translate_velocity;
	}

	RCLCPP_DEBUG(ctx.logger_, "linear_x: %.2f", servo_vel.linear.x);
	RCLCPP_DEBUG(ctx.logger_, "angular.z: %.2f", servo_vel.angular.z);

	// currently moving towards goal, collision_check first (charger-distance predict time)
	ctx.translation_collision_block(servo_vel, ctx.dist_buffer_point, true, name());

	return StepResult::stay(boost::make_optional(servo_vel));
}

StateGoalController::StepResult StateGoalController::GoalAngleState::on_update(StateCtx & ctx)
{
	RCLCPP_DEBUG(ctx.logger_, "***********************************");
	RCLCPP_DEBUG(ctx.logger_, "***********************************");
	RCLCPP_DEBUG(ctx.logger_, "--------------- GOAL_ANGLE ---------------");
	geometry_msgs::msg::Twist servo_vel;
	ctx.update_time_smart();

	const GoalPoint & gp = ctx.goal_points_.front();
	RCLCPP_DEBUG(ctx.logger_, "goal =>  x: %.2f, y: %.2f, yaw: %.2f", gp.x, gp.y, gp.theta);
	RCLCPP_DEBUG(ctx.logger_, "robot =>  x: %.2f, y: %.2f, yaw: %.2f",
		ctx.current_pose.getOrigin().getX(), ctx.current_pose.getOrigin().getY(), ctx.current_angle);

	double ang = angles::shortest_angular_distance(ctx.current_angle, gp.theta);
	ctx.bound_rotation(ang, ctx.params->go_to_goal_rotation_min, ctx.params->go_to_goal_rotation_max);
	RCLCPP_DEBUG(ctx.logger_, "diff angle: %.2f", ang);

	double translate_velocity = ctx.params->go_to_goal_translation_max;
	if (gp.drive_backwards) {
		translate_velocity *= -1.0;
	}

	if (std::abs(ang) > ctx.params->goal_angle_converged) {
		servo_vel.angular.z = ang;
	}
	ctx.goal_points_.pop_front();
	RCLCPP_DEBUG(ctx.logger_, "============ pop goal============");
	if (ctx.goal_points_.size() > 0) {
		RCLCPP_DEBUG(ctx.logger_, "******** change to state GO_TO_GOAL_POSITION ******** ");
		ctx.state = std::string("GOAL_ANGLE");
		ctx.infos = std::string("GOAL_ANGLE  ==> keep on rotating");
		return StepResult::go(boost::make_optional(servo_vel),
			StateRank::GO_TO_GOAL_POSITION, ctx.params->timeout_go_to_goal_position);
	}
	ctx.state = std::string("GOAL_ANGLE");
	ctx.infos = std::string("GOAL_ANGLE  ==> keep on rotating");
	return StepResult::stay(boost::make_optional(servo_vel));
}

StateGoalController::StepResult StateGoalController::UndockState::on_update(StateCtx & ctx)
{
	RCLCPP_DEBUG(ctx.logger_, "--------------- UNDOCK ---------------");
	geometry_msgs::msg::Twist servo_vel;
	ctx.update_time_smart();

	ctx.undock_dis_moved_ += ctx.odom_msg.twist.twist.linear.x * ctx.delta_time_;
	RCLCPP_INFO_THROTTLE(ctx.logger_, *ctx.clock_, 500,
		"undock cost time: %.2f, dis_moved: %.2f, total: %.2f",
		(ctx.now_sec() - ctx.current_state_start_time_), ctx.undock_dis_moved_, ctx.params->undock_dis);

	if (ctx.undock_dis_moved_ < ctx.params->undock_dis) {
		servo_vel.linear.x = ctx.params->undock_speed;

		double predict_time = std::min(
			(ctx.current_state_timeout_ - ctx.delta_time_), double(ctx.params->collision_predict_time));
		if (ctx.params->collision_check) {
			double cost_value = ctx.get_cost_value_undock(ctx.robot_pose_map, servo_vel.linear.x, predict_time);
			if (cost_value >= nav2_costmap_2d::LETHAL_OBSTACLE) {
				RCLCPP_DEBUG(ctx.logger_, "cost value: %.2f >= %.2f", cost_value,
					static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));
				servo_vel.linear.x = 0.0;
				RCLCPP_INFO_THROTTLE(ctx.logger_, *ctx.clock_, 1000,
					"stop for collision check, when %s", name());
				if (ctx.params->enable_clear_local_costmap) {
					ctx.clear_local_costmap();
				}
			}
		}
	} else {
		ctx.goal_points_.clear();
		RCLCPP_INFO(ctx.logger_, "x_charger: %.2f, y_charger: %.2f", ctx.robot_x_charger_, ctx.robot_y_charger_);
		RCLCPP_INFO(ctx.logger_, "undock succeed.");
		ctx.undocking = false;
	}

	return StepResult::stay(boost::make_optional(servo_vel));
}

} // namespace capella_ros_dock