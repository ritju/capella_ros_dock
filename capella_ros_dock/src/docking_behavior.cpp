

#include "capella_ros_dock/docking_behavior.hpp"

#include <memory>

namespace capella_ros_dock
{

using namespace std::placeholders;

DockingBehavior::DockingBehavior(
	rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
	rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
	rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
	rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
	rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
	motion_control_params* params_ptr,
	std::shared_ptr<BehaviorsScheduler> behavior_scheduler)
	: clock_(node_clock_interface->get_clock()),
	logger_(node_logging_interface->get_logger()),
	max_action_runtime_(rclcpp::Duration(std::chrono::seconds(params_ptr->max_action_runtime)))
{
	RCLCPP_INFO(logger_, "DockingBehavior constructor.");
	behavior_scheduler_ = behavior_scheduler;
	last_feedback_time_ = clock_->now();
	this->params_ptr = params_ptr;
	goal_controller_ = std::make_shared<SimpleGoalController>(params_ptr);
	// RCLCPP_INFO_STREAM(logger_, "max_dock_action_run_time: " << params_ptr->max_dock_action_run_time << " seconds.");

	undock_state_pub_ = rclcpp::create_publisher<std_msgs::msg::Bool>(
		node_topics_interface,
		"is_undocking_state",
		5
	);

	dock_visible_sub_ = rclcpp::create_subscription<capella_ros_service_interfaces::msg::ChargeMarkerVisible>(
		node_topics_interface,
		"/marker_visible",
		5,
		std::bind(&DockingBehavior::dock_visible_callback, this, _1)
		);

	charge_state_sub_ = rclcpp::create_subscription<capella_ros_service_interfaces::msg::ChargeState>(
		node_topics_interface,
		"charger/state",
		rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
		std::bind(&DockingBehavior::charge_state_callback, this, _1)
		);

	robot_pose_sub_ = rclcpp::create_subscription<aruco_msgs::msg::PoseWithId>(
		node_topics_interface,
		"/pose_with_id_optimize",
		rclcpp::SensorDataQoS(),
		std::bind(&DockingBehavior::robot_pose_callback, this, _1));

	raw_vel_sub_ = rclcpp::create_subscription<capella_ros_msg::msg::Velocities>(
		node_topics_interface,
		"/raw_vel",
		rclcpp::SensorDataQoS(),
		std::bind(&DockingBehavior::raw_vel_sub_callback, this, _1)
		);
	odom_sub_ = rclcpp::create_subscription<nav_msgs::msg::Odometry>(
		node_topics_interface,
		"/odom",
		rclcpp::SensorDataQoS(),
		std::bind(&DockingBehavior::odom_sub_callback, this, _1)
		);
	laserScan_sub_ = rclcpp::create_subscription<sensor_msgs::msg::LaserScan>(
		node_topics_interface,
		"/scan",
		rclcpp::SensorDataQoS(),
		std::bind(&DockingBehavior::laserScan_sub_callback, this, _1)
		);
	
	charger_id_sub_ = rclcpp::create_subscription<std_msgs::msg::String>(
		node_topics_interface,
		"/charger/id",
		rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
		std::bind(&DockingBehavior::charger_id_callback, this, _1)
		);
	marker_and_mac_sub_ = rclcpp::create_subscription<aruco_msgs::msg::MarkerAndMacVector>(
		node_topics_interface,
		"/id_mac",
		rclcpp::QoS(rclcpp::KeepLast(30)),
		std::bind(&DockingBehavior::marker_and_mac_callback, this, _1)
		);
	footprint_sub_ = rclcpp::create_subscription<geometry_msgs::msg::PolygonStamped>(
		node_topics_interface,
		"/local_costmap/published_footprint",
		rclcpp::QoS(rclcpp::KeepLast(1)),
		std::bind(&DockingBehavior::footprint_sub_callback_, this, _1)
	);
	local_costmap_sub_ = rclcpp::create_subscription<nav_msgs::msg::OccupancyGrid>(
		node_topics_interface,
		"/local_costmap/costmap",
		rclcpp::QoS(rclcpp::KeepLast(1)),
		std::bind(&DockingBehavior::local_costmap_sub_callback_, this, _1)
	);

	docking_action_server_ = rclcpp_action::create_server<capella_ros_dock_msgs::action::Dock>(
		node_base_interface,
		node_clock_interface,
		node_logging_interface,
		node_waitables_interface,
		"dock",
		std::bind(&DockingBehavior::handle_dock_servo_goal, this, _1, _2),
		std::bind(&DockingBehavior::handle_dock_servo_cancel, this, _1),
		std::bind(&DockingBehavior::handle_dock_servo_accepted, this, _1));

	undocking_action_server_ = rclcpp_action::create_server<capella_ros_service_interfaces::action::Undock>(
		node_base_interface,
		node_clock_interface,
		node_logging_interface,
		node_waitables_interface,
		"undock",
		std::bind(&DockingBehavior::handle_undock_goal, this, _1, _2),
		std::bind(&DockingBehavior::handle_undock_cancel, this, _1),
		std::bind(&DockingBehavior::handle_undock_accepted, this, _1));
	// Give poses default value, will be over-written by subscriptions
	last_robot_pose_.setIdentity();
	last_dock_pose_.setIdentity();
	tf2::Quaternion dock_rotation;
	dock_rotation.setRPY(0, 0, 0);
	last_dock_pose_.setRotation(dock_rotation);
	// Set number from observation, but will repopulate on undock with calibrated value
	last_docked_distance_offset_ = 0.32;
	action_start_time_ = clock_->now();

	this->footprint_collision_checker_ =  nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>();
}

void DockingBehavior::local_costmap_sub_callback_(const nav_msgs::msg::OccupancyGrid & msg)
{
	this->local_costmap_ = msg;
	costmap2d_ = nav2_costmap_2d::Costmap2D(this->local_costmap_);
	footprint_collision_checker_.setCostmap(&(this->costmap2d_));
}

void DockingBehavior::footprint_sub_callback_(const geometry_msgs::msg::PolygonStamped &msg)
{
	this->footprint_ = msg;

	std::vector<geometry_msgs::msg::Point>().swap(this->footprint_vec_);
	for (size_t i = 0; i < footprint_.polygon.points.size(); i++)
	{
		geometry_msgs::msg::Point point;
		point.x = footprint_.polygon.points[i].x;
		point.y = footprint_.polygon.points[i].y;
		point.z = footprint_.polygon.points[i].z;
		footprint_vec_.push_back(point);
	}
	
}

void DockingBehavior::raw_vel_sub_callback(capella_ros_msg::msg::Velocities raw_vel)
{
	this->raw_vel_msg = raw_vel;
}

void DockingBehavior::odom_sub_callback(nav_msgs::msg::Odometry odom)
{
	this->odom_msg = odom;
}



bool DockingBehavior::docking_behavior_is_done()
{
	return !running_dock_action_;
}

bool DockingBehavior::undocking_behavior_is_done()
{
	return !running_undock_action_;
}

rclcpp_action::GoalResponse DockingBehavior::handle_dock_servo_goal(
	const rclcpp_action::GoalUUID & /*uuid*/,
	std::shared_ptr<const capella_ros_dock_msgs::action::Dock::Goal>/*goal*/)
{
	RCLCPP_INFO(logger_, "Received new dock servo goal");

	// if (!docking_behavior_is_done()) {
	// 	RCLCPP_WARN(logger_, "A docking behavior is already running, reject");
	// 	return rclcpp_action::GoalResponse::REJECT;
	// }

	if (is_docked_) {
		RCLCPP_WARN(logger_, "Robot already docked, reject");
		return rclcpp_action::GoalResponse::REJECT;
	}
	if (!sees_dock_) {
		RCLCPP_INFO(logger_, "Robot doesn't see dock, begin roation ");
		// return rclcpp_action::GoalResponse::REJECT;
	}
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DockingBehavior::handle_dock_servo_cancel(
	const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<capella_ros_dock_msgs::action::Dock> >/*goal_handle*/)
{
	RCLCPP_INFO(logger_, "Received request to cancel dock servo goal");
	return rclcpp_action::CancelResponse::ACCEPT;
}

void DockingBehavior::handle_dock_servo_accepted(
	const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<capella_ros_dock_msgs::action::Dock> > goal_handle)
{
	// Create new Docking state machine
	running_dock_action_ = true;
	action_start_time_ = clock_->now();

	const auto goal = goal_handle->get_goal();
	charger_id_ = goal->mac;

	for(size_t i = 0; i < marker_and_mac_vector.marker_and_mac_vector.size(); i++)
	{
		if(charger_id_.compare(marker_and_mac_vector.marker_and_mac_vector[i].bluetooth_mac) == 0)
		{
			marker_id_ = marker_and_mac_vector.marker_and_mac_vector[i].marker_id;
		}
	}

	// Generate point offset from dock facing dock then point at dock
	SimpleGoalController::CmdPath dock_path;
	tf2::Transform robot_pose(tf2::Transform::getIdentity());
	{
		const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
		robot_pose = last_robot_pose_;
		auto position = robot_pose.getOrigin();
		auto yaw = tf2::getYaw(robot_pose.getRotation());
		RCLCPP_INFO(logger_, "robot_pose => x: %f, y: %f, angular: %f", position.getX(), position.getY(), yaw);
	}
	tf2::Transform dock_pose(tf2::Transform::getIdentity());
	{
		const std::lock_guard<std::mutex> lock(dock_pose_mutex_);
		dock_pose = last_dock_pose_;
		auto position = dock_pose.getOrigin();
		auto yaw = tf2::getYaw(dock_pose.getRotation());
		RCLCPP_INFO(logger_, "dock_pose  => x: %f, y: %f, angular: %f", position.getX(), position.getY(), yaw);
	}
	// If robot is farther than 0.5 from dock, put offset point 0.5 in front of dock,
	// otherwise put in line with robot's current distance away from the dock
	// const tf2::Vector3 & robot_position = robot_pose.getOrigin();
	// const tf2::Vector3 & dock_position = dock_pose.getOrigin();
	// double dist_offset = std::hypot(
	// 	dock_position.getX() - robot_position.getX(),
	// 	dock_position.getY() - robot_position.getY());
	// RCLCPP_INFO(logger_, "dist_offset: %f", dist_offset);
	MAX_DOCK_INTERMEDIATE_GOAL_OFFSET = params_ptr->distance_low_speed + params_ptr->second_goal_distance;
	last_docked_distance_offset_ = params_ptr->last_docked_distance_offset_;
	const double max_goal_offset = MAX_DOCK_INTERMEDIATE_GOAL_OFFSET + last_docked_distance_offset_ + params_ptr->first_goal_distance;
	// if (dist_offset > max_goal_offset) {
	double dist_offset = max_goal_offset;
	// }
	RCLCPP_DEBUG(logger_, "dist_offset: %f", dist_offset);
	tf2::Transform dock_offset(tf2::Transform::getIdentity());
	tf2::Quaternion dock_rotation;


	// dock_rotation.setRPY(0, 0, 0);
	// dock_offset.setOrigin(tf2::Vector3(-dist_offset , params_ptr->goal_y_correction, 0));
	// dock_offset.setRotation(dock_rotation);
	// dock_path.emplace_back(dock_pose * dock_offset, 0.10, true); // third goal

	// dock_rotation.setRPY(0, 0, 0);
	// dock_offset.setOrigin(tf2::Vector3(-(params_ptr->last_docked_distance_offset_ + 0.10), params_ptr->goal_y_correction, 0));
	// dock_offset.setRotation(dock_rotation);
	// dock_path.emplace_back(dock_pose * dock_offset, 0.01, true); // second goal

	float dx = 0.05;
	float start_point_x = -(dist_offset + params_ptr->buffer_goal_distance);
	float end_point_x = -params_ptr->last_docked_distance_offset_;
	int size = std::floor(std::abs(end_point_x - start_point_x) / dx);
	for (int i = 1; i <= size; i++)
	{
		float x_coord = start_point_x + dx * i;
		RCLCPP_INFO(logger_, "goal_x: %f", x_coord);
		dock_rotation.setRPY(0, 0, 0);
		dock_offset.setOrigin(tf2::Vector3(x_coord, params_ptr->goal_y_correction, 0));
		dock_offset.setRotation(dock_rotation);
		dock_path.emplace_back(dock_pose * dock_offset, 0.01, true); 
	}

	// dock_rotation.setRPY(0, 0, 0);
	// dock_offset.setOrigin(tf2::Vector3(-params_ptr->first_goal_distance, params_ptr->goal_y_correction, 0));
	// dock_offset.setRotation(dock_rotation);
	// dock_path.emplace_back(dock_pose * dock_offset, 0.01, true); // first goal

	goal_controller_->initialize_goal(dock_path);
	// Setup behavior to override other commanded motion
	BehaviorsScheduler::BehaviorsData data;
	data.run_func = std::bind(&DockingBehavior::execute_dock_servo, this, goal_handle, _1);
	data.is_done_func = std::bind(&DockingBehavior::docking_behavior_is_done, this);
	data.stop_on_new_behavior = true;
	data.apply_backup_limits = false;

	const bool ret = behavior_scheduler_->set_behavior(data);
	RCLCPP_DEBUG(logger_, "set behavior: %s", ret ? "true" : "false");
	if (!ret) {
		// for some reason we couldn't set the new behavior, treat this as a goal being cancelled
		RCLCPP_WARN(logger_, "Dock Servo behavior failed to start");
		auto result = std::make_shared<capella_ros_dock_msgs::action::Dock::Result>();
		result->is_docked = is_docked_;
		goal_handle->abort(result);
		running_dock_action_ = false;
	}
	last_feedback_time_ = clock_->now();
}

BehaviorsScheduler::optional_output_t DockingBehavior::execute_dock_servo(
	const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<capella_ros_dock_msgs::action::Dock> > goal_handle,
	const RobotState & current_state)
{
	BehaviorsScheduler::optional_output_t servo_cmd;
	// Handle if goal is cancelling
	if (goal_handle->is_canceling()) {
		RCLCPP_INFO(logger_, "Cancelling the goal.");
		auto result = std::make_shared<capella_ros_dock_msgs::action::Dock::Result>();
		result->is_docked = is_docked_;
		goal_handle->canceled(result);
		goal_controller_->reset();
		running_dock_action_ = false;
		return servo_cmd;
	}

	bool exceeded_runtime = false;
	if (clock_->now() - action_start_time_ > max_action_runtime_) {
		RCLCPP_INFO(logger_, "Dock Servo Goal Exceeded Runtime");
		exceeded_runtime = true;
	}
	// Get next command
	tf2::Transform robot_pose(tf2::Transform::getIdentity());
	{
		const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
		robot_pose = last_robot_pose_;
	}
	auto hazards = current_state.hazards;
	servo_cmd = goal_controller_->get_velocity_for_position(robot_pose, current_state.pose, current_state.charger_pose, sees_dock_, is_docked_,
	 bluetooth_connected,  odom_msg, clock_, logger_, params_ptr, hazards, state, infos, footprint_collision_checker_, footprint_vec_);
	if(this->is_docked_)
	{
		RCLCPP_DEBUG(logger_, "zero cmd time => sec: %f", this->clock_.get()->now().seconds());
	}

	// tmp for test clean_robot
	// double x,y,r;
	// x = this->last_robot_pose_.getOrigin().getX();
	// y = this->last_robot_pose_.getOrigin().getY();
	// r = std::hypot(x,y);
	// if (r < (this->params_ptr->last_docked_distance_offset_ + this->params_ptr->distance_low_speed))
	// {
	// 	RCLCPP_INFO(logger_, "distance: %f,  return success for testing.", r);
	// 	auto result = std::make_shared<capella_ros_dock_msgs::action::Dock::Result>();
	// 	result->is_docked = true;
	// 	RCLCPP_INFO(logger_, "Dock Servo Goal Succeeded\n");
	// 	goal_handle->succeed(result);

	// 	goal_controller_->reset();
	// 	running_dock_action_ = false;
	// 	return servo_cmd;
	// }

	if (!servo_cmd || exceeded_runtime) {
		auto result = std::make_shared<capella_ros_dock_msgs::action::Dock::Result>();
		if (is_docked_) {
			result->is_docked = true;
			RCLCPP_INFO(logger_, "Dock Servo Goal Succeeded\n");
			goal_handle->succeed(result);
		} else {
			result->is_docked = false;
			RCLCPP_INFO(logger_, "Dock Servo Goal Aborted\n");
			goal_handle->abort(result);
		}
		goal_controller_->reset();
		running_dock_action_ = false;
		return servo_cmd;
	}

	rclcpp::Time current_time = clock_->now();
	auto time_since_feedback = current_time - last_feedback_time_;
	if (time_since_feedback > report_feedback_interval_) {
		// Publish feedback
		auto feedback = std::make_shared<capella_ros_dock_msgs::action::Dock::Feedback>();
		feedback->sees_dock = sees_dock_;
		feedback->state = state;
		feedback->infos = infos;
		goal_handle->publish_feedback(feedback);
		last_feedback_time_ = current_time;
	}

	return servo_cmd;
}

rclcpp_action::GoalResponse DockingBehavior::handle_undock_goal(
	const rclcpp_action::GoalUUID & /*uuid*/,
	std::shared_ptr<const capella_ros_service_interfaces::action::Undock::Goal>/*goal*/)
{
	RCLCPP_INFO(logger_, "Received new undock goal");
	
	if (!sees_dock_)
	{
		RCLCPP_WARN(logger_, "Robot cannot see the aruco marker, reject");
		return rclcpp_action::GoalResponse::REJECT;
	}
	auto current_pose = last_robot_pose_.getOrigin();
	double x;
	x = current_pose.getX();
	if (std::abs(x) > (params_ptr->last_docked_distance_offset_ + 0.5))
	{
		RCLCPP_WARN(logger_, "Robot had undocked, reject");
		return rclcpp_action::GoalResponse::REJECT;
	}

	if (!undocking_behavior_is_done()) {
		RCLCPP_WARN(logger_, "An un_docking behavior is already running, reject");
		return rclcpp_action::GoalResponse::REJECT;
	}

	if (undock_has_obstacle_)
	{
		RCLCPP_WARN(logger_, "There are some obstacles in front of robot, reject");
		return rclcpp_action::GoalResponse::REJECT;
	}


	// if (!is_docked_) {
	// 	RCLCPP_WARN(logger_, "Robot already undocked, reject");
	// 	return rclcpp_action::GoalResponse::REJECT;
	// }
	return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DockingBehavior::handle_undock_cancel(
	const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<capella_ros_service_interfaces::action::Undock> >/*goal_handle*/)
{
	RCLCPP_INFO(logger_, "Received request to cancel undock goal");
	return rclcpp_action::CancelResponse::ACCEPT;
}

void DockingBehavior::handle_undock_accepted(
	const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<capella_ros_service_interfaces::action::Undock> > goal_handle)
{
	// Create new Docking Action
	running_undock_action_ = true;
	action_start_time_ = clock_->now();

	SimpleGoalController::CmdPath undock_path;
	// Generate path with point offset from robot pose,
	// have robot drive backwards to offset then turn 180
	tf2::Transform robot_pose(tf2::Transform::getIdentity());
	{
		const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
		robot_pose = last_robot_pose_;
	}
	if (!calibrated_offset_) {
		tf2::Transform dock_pose(tf2::Transform::getIdentity());
		{
			const std::lock_guard<std::mutex> lock(dock_pose_mutex_);
			dock_pose = last_dock_pose_;
		}
		calibrate_docked_distance_offset(robot_pose, dock_pose);
	}
	tf2::Transform dock_offset(tf2::Transform::getIdentity());
	dock_offset.setOrigin(tf2::Vector3(-UNDOCK_GOAL_OFFSET, 0, 0));
	tf2::Transform undock_offset = robot_pose * dock_offset;
	undock_path.emplace_back(undock_offset, 0.05, false);
	tf2::Transform face_away_dock(tf2::Transform::getIdentity());
	tf2::Quaternion undock_rotation;
	undock_rotation.setRPY(0, 0, M_PI);
	face_away_dock.setRotation(undock_rotation);
	tf2::Transform undocked_goal = undock_offset * face_away_dock;
	undock_path.emplace_back(undocked_goal, 0.05, false);
	goal_controller_->initialize_goal(undock_path);

	BehaviorsScheduler::BehaviorsData data;
	data.run_func = std::bind(&DockingBehavior::execute_undock, this, goal_handle, _1);
	data.is_done_func = std::bind(&DockingBehavior::undocking_behavior_is_done, this);
	data.stop_on_new_behavior = true;
	data.apply_backup_limits = false;

	const bool ret = behavior_scheduler_->set_behavior(data);
	if (!ret) {
		// for some reason we couldn't set the new behavior, treat this as a goal being cancelled
		RCLCPP_WARN(logger_, "Undock behavior failed to start");
		auto result = std::make_shared<capella_ros_service_interfaces::action::Undock::Result>();
		result->is_docked = is_docked_;
		result->sees_charger = sees_dock_;
		result->success = false;
		goal_handle->abort(result);
		goal_controller_->reset();
		running_undock_action_ = false;
	}
	last_feedback_time_ = clock_->now();
}

BehaviorsScheduler::optional_output_t DockingBehavior::execute_undock(
	const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<capella_ros_service_interfaces::action::Undock> > goal_handle,
	const RobotState & current_state)
{
	BehaviorsScheduler::optional_output_t servo_cmd;
	// Handle if goal is cancelling
	if (goal_handle->is_canceling()) {
		auto result = std::make_shared<capella_ros_service_interfaces::action::Undock::Result>();
		result->is_docked = is_docked_;
		result->sees_charger = sees_dock_;
		result->success = false;
		goal_handle->canceled(result);
		goal_controller_->reset();
		running_undock_action_ = false;
		return BehaviorsScheduler::optional_output_t();
	}
	// Get next command
	tf2::Transform robot_pose(tf2::Transform::getIdentity());
	{
		const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
		robot_pose = last_robot_pose_;
	}
	auto hazards = current_state.hazards;
	servo_cmd = goal_controller_->get_velocity_for_position(robot_pose, current_state.pose, current_state.charger_pose, sees_dock_,
	                                                        is_docked_, bluetooth_connected, odom_msg, clock_, logger_, params_ptr,
								 hazards, state, infos, footprint_collision_checker_, footprint_vec_);

	
	auto msg = std_msgs::msg::Bool();
	msg.data = true;
	undock_state_pub_->publish(msg);
	bool exceeded_runtime = false;
	if ((clock_->now() - action_start_time_) > rclcpp::Duration(std::chrono::seconds((int)(params_ptr->undock_time) + 2))) {
		RCLCPP_INFO(logger_, "Undock Goal Exceeded Runtime");
		exceeded_runtime = true;
	}

	if (!servo_cmd || exceeded_runtime) {
		auto result = std::make_shared<capella_ros_service_interfaces::action::Undock::Result>();
		result->is_docked = is_docked_;
		result->sees_charger = sees_dock_;
		if (!is_docked_) {
			RCLCPP_INFO(logger_, "Undock Goal Succeeded");
			result->success = true;
			goal_handle->succeed(result);
		} else {
			result->success = false;
			RCLCPP_INFO(logger_, "Undock Goal Aborted");
			goal_handle->abort(result);
		}
		goal_controller_->reset();
		running_undock_action_ = false;
		return BehaviorsScheduler::optional_output_t();
	}

	return servo_cmd;
}

void DockingBehavior::dock_visible_callback(capella_ros_service_interfaces::msg::ChargeMarkerVisible::ConstSharedPtr msg)
{
	this->sees_dock_ = msg->marker_visible;
	// RCLCPP_INFO(logger_, "sees_dock: %d", sees_dock_.load());
}

void DockingBehavior::charge_state_callback(capella_ros_service_interfaces::msg::ChargeState::ConstSharedPtr msg)
{
	// if(!this->is_docked_ && msg->has_contact)
	// {
	// 	rclcpp::Time contact_time = this->clock_.get()->now();
	// 	RCLCPP_DEBUG(logger_, "First receive contact msg => sec: %f, nanosec: %ld ",
	// 	            contact_time.seconds(), contact_time.nanoseconds() % 1000000000);
	// 	RCLCPP_DEBUG(logger_, "First send contact msg    => sec: %d, nanosec: %d ",
	// 	            msg->stamp.sec, msg->stamp.nanosec);
	// } // 调试 需临时更改msg消息类型定义
	this->is_docked_ = msg->has_contact;
	if (this->is_docked_ != this->contact_state_last_)
	{
		RCLCPP_INFO(logger_, "motion_control node => contact state changed from %s to %s", 
			this->contact_state_last_?"true":"false",
			this->is_docked_?"true":"false");
		this->contact_state_last_ = this->is_docked_;
	}
	this->bluetooth_connected = !(msg->pid.compare("") == 0);
}

void DockingBehavior::robot_pose_callback(aruco_msgs::msg::PoseWithId::ConstSharedPtr msg)
{
	const std::lock_guard<std::mutex> lock(robot_pose_mutex_);
	if(msg->marker_id == this->marker_id_)
	{
		tf2::convert(msg->pose.pose, last_robot_pose_);
		// this->sees_dock_ = true;
	}
	else
	{
		RCLCPP_DEBUG_THROTTLE(logger_, *clock_, 5000, "topic /pose_with_id's marker_id: %d, node's marker_id_: %d", (int)(msg->marker_id), marker_id_);
	}
}

// void DockingBehavior::dock_pose_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg)
// {
//   const std::lock_guard<std::mutex> lock(dock_pose_mutex_);
//   tf2::convert(msg->pose, last_dock_pose_);
// }

void DockingBehavior::calibrate_docked_distance_offset(
	const tf2::Transform & docked_robot_pose,
	const tf2::Transform & dock_pose)
{
	tf2::Vector3 pos_diff = docked_robot_pose.getOrigin() - dock_pose.getOrigin();
	last_docked_distance_offset_ = std::hypot(pos_diff.getX(), pos_diff.getY());
	calibrated_offset_ = true;
	RCLCPP_DEBUG(logger_, "Setting robot dock offset to %f", last_docked_distance_offset_);
}

void DockingBehavior::laserScan_sub_callback(sensor_msgs::msg::LaserScan msg)
{
	if (!has_table)
	{
		laser_min_angle = msg.angle_min;
		laser_data_size = msg.ranges.size();
		laser_angle_increament = msg.angle_increment;
		RCLCPP_INFO(logger_, "min_angle: %f", laser_min_angle);
		RCLCPP_INFO(logger_, "laser_data_size: %d", laser_data_size);
		RCLCPP_INFO(logger_, "laser_angle_increament: %f", laser_angle_increament);
		generate_sin_cos_table(laser_min_angle, laser_angle_increament, laser_data_size);
		has_table = true;
		undock_has_obstacle_ = check_undock_has_obstale(msg);
	}
	else
	{
		undock_has_obstacle_ = check_undock_has_obstale(msg);
	}

	// debug results
	if(undock_has_obstacle_)
	{
		// RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 1000, "undock, has obstale.");
	}
	else
	{
		// RCLCPP_DEBUG_STREAM_THROTTLE(logger_, *clock_, 1000, "undock, free space.");
	}	
}

void DockingBehavior::charger_id_callback(std_msgs::msg::String msg)
{
	charger_id_ = msg.data;
	for(size_t i = 0; i < marker_and_mac_vector.marker_and_mac_vector.size(); i++)
	{
		if(charger_id_.compare(marker_and_mac_vector.marker_and_mac_vector[i].bluetooth_mac) == 0)
		{
			marker_id_ = marker_and_mac_vector.marker_and_mac_vector[i].marker_id;
		}
	}
}

void DockingBehavior::marker_and_mac_callback(aruco_msgs::msg::MarkerAndMacVector msg)
{
	this->marker_and_mac_vector = msg;
}

void DockingBehavior::generate_sin_cos_table(float theta_min, float angle_increament, int size)
{
	sin_table.resize(size);
	cos_table.resize(size);
	for(int i = 0; i < size; i++)
	{
		sin_table[i] = sin(theta_min + angle_increament * i);
		cos_table[i] = cos(theta_min + angle_increament * i);
	}
}

bool DockingBehavior::check_undock_has_obstale(sensor_msgs::msg::LaserScan msg)
{
	bool ret = false;
	float pos_lr, pos_front;
	// sin(x) => left/right, cos(x) => front
	float range;
	for(int i = 0; i < laser_data_size; i++)
	{
		range = msg.ranges[i];
		if(isinf(range) || range < msg.range_min || range > msg.range_max)
		{
			continue;
		}
		else
		{
			pos_lr = std::abs(range * sin_table[i]);
			pos_front = std::abs(range * cos_table[i]);
			// RCLCPP_DEBUG(logger_, "range: %f", range);
			// RCLCPP_DEBUG(logger_, "sin: %f", sin_table[i]);
			// RCLCPP_DEBUG(logger_, "cos: %f", cos_table[i]);
			// RCLCPP_DEBUG(logger_, "pos_lr: %f", pos_lr);
			// RCLCPP_DEBUG(logger_, "pos_front: %f", pos_front);
			
			if (pos_lr < params_ptr->undock_obstacle_lr && pos_front < params_ptr->undock_obstacle_front)
			{
				ret = true;
				break;
			}
			else
			{
				continue;
			}
		}
		
	}
	return ret;
}

}  // namespace capella_ros_dock
