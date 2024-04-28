

#include "capella_ros_dock/motion_control_node.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace capella_ros_dock
{
using namespace std::placeholders;
using namespace std::chrono_literals;

MotionControlNode::MotionControlNode(const rclcpp::NodeOptions & options)
	: rclcpp::Node("motion_control", options)
{
	RCLCPP_INFO(this->get_logger(), "motion_control node constructor.");

	tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
	tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

	// only compatible for other behaviors
	current_state_.pose.setIdentity();
	current_state_.charger_pose.setIdentity();
	current_state_.hazards = capella_ros_dock_msgs::msg::HazardDetectionVector();

	init_params();

	// create subscription to hazards
	auto options_hazard_detection = rclcpp::SubscriptionOptions();
	cb_group_hazards_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	options_hazard_detection.callback_group = cb_group_hazards_;
	sub_hazards_ = this->create_subscription<capella_ros_dock_msgs::msg::HazardDetectionVector>
	                       ("hazard_detection", rclcpp::SensorDataQoS(), std::bind(&MotionControlNode::cb_hazard_detection, this, _1), options_hazard_detection);

	// create subscription to /charger/pose
	auto options_charger_pose = rclcpp::SubscriptionOptions();
	cb_group_charger_pose_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	options_charger_pose.callback_group = cb_group_charger_pose_;
	sub_charger_pose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>
	                       ("/charger/pose", rclcpp::QoS(5).transient_local().reliable(), std::bind(&MotionControlNode::cb_charger_pose, this, _1), options_charger_pose);

	// Create behaviors scheduler
	scheduler_ = std::make_shared<BehaviorsScheduler>();
	// Create Docking Behavior manager
	docking_behavior_ = std::make_shared<DockingBehavior>(
		this->get_node_base_interface(),
		this->get_node_clock_interface(),
		this->get_node_logging_interface(),
		this->get_node_topics_interface(),
		this->get_node_waitables_interface(),
		&(this->params),
		scheduler_);

	cmd_vel_out_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
		"/cmd_vel", rclcpp::SystemDefaultsQoS());

	// Create timer to periodically execute behaviors and control the robot
	constexpr auto control_period = std::chrono::duration<double>(1.0 / 40);
	start_control_timer = rclcpp::create_timer(
		this,
		this->get_clock(),
		rclcpp::Duration(control_period),
		std::bind(&MotionControlNode::start_control_timer_callback, this));
}

void MotionControlNode::init_params()
{
	this->declare_parameter<int>("max_dock_action_run_time", 180);
	this->declare_parameter<float>("min_rotation", 0.15);
	this->declare_parameter<float>("max_rotation", 0.3);
	this->declare_parameter<float>("min_translation", 0.02);
	this->declare_parameter<float>("max_translation", 0.1);
	this->declare_parameter<float>("angle_to_goal_angle_converged", 0.10);
	this->declare_parameter<float>("go_to_goal_angle_too_far", 0.15);
	this->declare_parameter<float>("go_to_goal_apply_rotation_angle", 0.10);
	this->declare_parameter<float>("goal_angle_converged", 0.15);
	this->declare_parameter<float>("dist_goal_converged", 0.10);
	this->declare_parameter<float>("last_docked_distance_offset_", 0.32);
	this->declare_parameter<float>("distance_low_speed", 0.10);
	this->declare_parameter<float>("translate_low_speed", 0.02);
	this->declare_parameter<float>("rotation_low_speed", 0.05);
	this->declare_parameter<float>("first_goal_distance", 0.0);
	this->declare_parameter<float>("second_goal_distance", 0.5);
	this->declare_parameter<float>("buffer_goal_distance", 1.0);
	this->declare_parameter<float>("camera_horizontal_view", 64.0);
	this->declare_parameter<float>("localization_converged_time", 2);
	this->declare_parameter<float>("tolerance_angle", 0.05);
	this->declare_parameter<float>("tolerance_r", 0.1);
	this->declare_parameter<float>("deviate_second_goal_x", 0.2);
	this->declare_parameter<float>("dist_error_y_1", 0.05);
	this->declare_parameter<float>("dist_error_x_and_y", 0.3);
	this->declare_parameter<std::string>("motion_control_log_level", "info");
	this->declare_parameter<int>("cmd_vel_hz", 10);
	this->declare_parameter<float>("angle_delta", 0.1);
	this->declare_parameter<float>("marker_size", 0.2);
	this->declare_parameter<float>("dock_valid_obstacle_x", 0.4);
	this->declare_parameter<float>("time_sleep", 3.0);
	this->declare_parameter<float>("go_to_goal_rotation_min", 0.05);
	this->declare_parameter<float>("go_to_goal_rotation_max", 0.10);
	this->declare_parameter<float>("speed_rotation_acceleration", 0.20);
	this->declare_parameter<float>("speed_rotation_init_abs", 0.02);
	this->declare_parameter<int>("max_action_runtime", 180);
	this->declare_parameter<float>("contacted_keep_move_time", 0.1);
	this->declare_parameter<float>("undock_speed", 0.15);
	this->declare_parameter<float>("undock_time", 0.20);
	this->declare_parameter<float>("undock_obstacle_lr", 0.25);
	this->declare_parameter<float>("undock_obstacle_front", 0.50);

	params.max_dock_action_run_time = this->get_parameter_or<int>("max_dock_action_run_time", 180);
	params.min_rotation = this->get_parameter_or<float>("min_rotation", 0.15);
	params.max_rotation = this->get_parameter_or<float>("max_rotation", 0.30);
	params.min_translation = this->get_parameter_or<float>("min_translation", 0.02);
	params.max_translation = this->get_parameter_or<float>("max_translation", 0.10);
	params.angle_to_goal_angle_converged = this->get_parameter_or<float>("angle_to_goal_angle_converged", 0.10);
	params.go_to_goal_angle_too_far = this->get_parameter_or<float>("go_to_goal_angle_too_far", 0.15);
	params.go_to_goal_apply_rotation_angle = this->get_parameter_or<float>("go_to_goal_apply_rotation_angle", 0.10);
	params.goal_angle_converged = this->get_parameter_or<float>("goal_angle_converged", 0.15);
	params.dist_goal_converged = this->get_parameter_or<float>("dist_goal_converged", 0.10);
	params.last_docked_distance_offset_ = this->get_parameter_or<float>("last_docked_distance_offset_", 0.32);
	params.distance_low_speed = this->get_parameter_or<float>("distance_low_speed", 0.10);
	params.translate_low_speed = this->get_parameter_or<float>("translate_low_speed", 0.02);
	params.rotation_low_speed = this->get_parameter_or<float>("rotation_low_speed", 0.05);
	params.first_goal_distance = this->get_parameter_or<float>("first_goal_distance", 0.0);
	params.second_goal_distance = this->get_parameter_or<float>("second_goal_distance", 0.50);
	params.buffer_goal_distance = this->get_parameter_or<float>("buffer_goal_distance", 1.0);
	params.camera_horizontal_view = this->get_parameter_or<float>("camera_horizontal_view", 64.0);
	params.localization_converged_time = this->get_parameter_or<float>("localization_converged_time", 2);
	params.tolerance_angle = this->get_parameter_or<float>("tolerance_angle", 0.05);
	params.tolerance_r = this->get_parameter_or<float>("tolerance_r", 0.1);
	params.deviate_second_goal_x = this->get_parameter_or<float>("deviate_second_goal_x", 0.2);
	params.dist_error_y_1 = this->get_parameter_or<float>("dist_error_y_1", 0.05);
	params.dist_error_x_and_y = this->get_parameter_or<float>("dist_error_x_and_y", 0.3);
	params.motion_control_log_level = this->get_parameter_or<std::string>("motion_control_log_level", "info");
	params.cmd_vel_hz = this->get_parameter("cmd_vel_hz").get_value<int>();
	params.angle_delta = this->get_parameter("angle_delta").get_value<float>();
	params.marker_size = this->get_parameter("marker_size").get_value<float>();
	params.dock_valid_obstacle_x = this->get_parameter("dock_valid_obstacle_x").get_value<float>();
	params.time_sleep = this->get_parameter("time_sleep").get_value<float>();
	params.go_to_goal_rotation_min = this->get_parameter("go_to_goal_rotation_min").get_value<float>();
	params.go_to_goal_rotation_max = this->get_parameter("go_to_goal_rotation_max").get_value<float>();
	params.speed_rotation_acceleration = this->get_parameter("speed_rotation_acceleration").get_value<float>();
	params.speed_rotation_init_abs = this->get_parameter("speed_rotation_init_abs").get_value<float>();
	params.max_action_runtime = this->get_parameter("max_action_runtime").get_value<int>();
	params.contacted_keep_move_time = this->get_parameter("contacted_keep_move_time").get_value<float>();
	params.undock_speed = this->get_parameter("undock_speed").get_value<float>();
	params.undock_time = this->get_parameter("undock_time").get_value<float>();
	params.undock_obstacle_lr = this->get_parameter("undock_obstacle_lr").get_value<float>();
	params.undock_obstacle_front = this->get_parameter("undock_obstacle_front").get_value<float>();
	
	// RCLCPP_INFO_STREAM(this->get_logger(), "max_dock_action_run_time: " << params.max_dock_action_run_time << " s.");
}

void MotionControlNode::cb_hazard_detection(capella_ros_dock_msgs::msg::HazardDetectionVector::SharedPtr msg)
{
	std::lock_guard<std::mutex> lock(current_state_mutex_);
	current_state_.hazards = *msg;

}

void MotionControlNode::cb_charger_pose(geometry_msgs::msg::PoseWithCovarianceStamped msg)
{
	RCLCPP_INFO(get_logger(), "Receive a new charger/pose.");
	tf2::Stamped<tf2::Transform> stamped_tf;
	geometry_msgs::msg::TransformStamped msg_tf_stamped;
	msg_tf_stamped.header = msg.header;
	msg_tf_stamped.child_frame_id = std::string("charger");
	msg_tf_stamped.transform.translation.x = msg.pose.pose.position.x;
	msg_tf_stamped.transform.translation.y = msg.pose.pose.position.y;
	msg_tf_stamped.transform.translation.z = msg.pose.pose.position.z;
	msg_tf_stamped.transform.rotation = msg.pose.pose.orientation;
	tf2::fromMsg(msg_tf_stamped, stamped_tf);
	current_state_.charger_pose = static_cast<tf2::Transform>(stamped_tf);
}

bool MotionControlNode::getTransform(
	const std::string & refFrame, const std::string & childFrame,
	geometry_msgs::msg::TransformStamped & transform)
{
	std::string errMsg;

	if (!tf_buffer_->canTransform(
		    refFrame, childFrame, tf2::TimePointZero,
		    tf2::durationFromSec(0.5), &errMsg))
	{
		RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to get pose from TF: " << errMsg);
		return false;
	} else {
		try {
			transform = tf_buffer_->lookupTransform(
				refFrame, childFrame, tf2::TimePointZero, tf2::durationFromSec(
					0.5));
		} catch (const tf2::TransformException & e) {
			RCLCPP_ERROR_STREAM(
				this->get_logger(),
				"Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
			return false;
		}
	}
	return true;
}

void MotionControlNode::start_control_timer_callback()
{
	if(scheduler_->has_behavior())
	{
		if(!control_timer_)
		{
			auto control_period = std::chrono::duration<double>(1.0 / params.cmd_vel_hz);
			control_timer_ = rclcpp::create_timer(
				this,
				this->get_clock(),
				rclcpp::Duration(control_period),
				std::bind(&MotionControlNode::control_robot, this));
		}
	}
	else
	{
		control_timer_.reset();
	}
}

void MotionControlNode::control_robot()
{
	// Handle behaviors
	BehaviorsScheduler::optional_output_t command;
	if (scheduler_->has_behavior()) {
		const std::lock_guard<std::mutex> lock(current_state_mutex_);
		geometry_msgs::msg::TransformStamped transform_stamped;
		tf2::Stamped<tf2::Transform> stamped_map2baselink;
		stamped_map2baselink.setIdentity();
		if (!getTransform(std::string("map"), std::string("base_link"), transform_stamped))
		{
			RCLCPP_INFO(get_logger(), "Failed to get tf from map to base_link.");
		}
		tf2::fromMsg(transform_stamped, stamped_map2baselink);
		current_state_.pose = static_cast<tf2::Transform>(stamped_map2baselink);

		command = scheduler_->run_behavior(current_state_);
	}

	// Create a null command if we don't have anything.
	if (!command) {
		command = geometry_msgs::msg::Twist();
	}
	auto cmd_out_msg = std::make_unique<geometry_msgs::msg::Twist>();
	*cmd_out_msg = *command;
	cmd_vel_out_pub_->publish(std::move(cmd_out_msg));
}
}  // namespace capella_ros_dock

RCLCPP_COMPONENTS_REGISTER_NODE(capella_ros_dock::MotionControlNode)
