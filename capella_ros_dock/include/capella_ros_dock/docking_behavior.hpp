

#ifndef CAPELLA_ROS_DOCK__DOCKING_BEHAVIOR_HPP_
#define CAPELLA_ROS_DOCK__DOCKING_BEHAVIOR_HPP_

#include <stdint.h>

#include <atomic>
#include <memory>

#include "capella_ros_dock_msgs/action/dock.hpp"
#include "capella_ros_dock_msgs/action/undock.hpp"
#include "capella_ros_dock/behaviors_scheduler.hpp"
#include "capella_ros_dock/simple_goal_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "capella_ros_service_interfaces/msg/charge_state.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "capella_ros_service_interfaces/msg/charge_marker_visible.hpp"
#include "capella_ros_msg/msg/velocities.hpp"
#include "rclcpp/qos.hpp"
#include "capella_ros_dock/utils.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "capella_ros_service_interfaces/action/undock.hpp"
#include "aruco_msgs/msg/pose_with_id.hpp"
#include "aruco_msgs/msg/marker_and_mac.hpp"
#include "std_msgs/msg/string.hpp"
#include "aruco_msgs/msg/marker_and_mac_vector.hpp"
#include "capella_ros_dock_msgs/msg/charger_contact_condition_type.hpp"

#include "std_msgs/msg/bool.hpp"

namespace capella_ros_dock
{


/**
 * @brief This class allows to create and manage Docking and Undocking action
 * servers.
 */
class DockingBehavior
{
public:
DockingBehavior(
	rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_base_interface,
	rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
	rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
	rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
	rclcpp::node_interfaces::NodeWaitablesInterface::SharedPtr node_waitables_interface,
  motion_control_params *params_ptr,
	std::shared_ptr<BehaviorsScheduler> behavior_scheduler);
~DockingBehavior() = default;

private:
bool docking_behavior_is_done();
bool undocking_behavior_is_done();

void calibrate_docked_distance_offset(
	const tf2::Transform & docked_robot_pose,
	const tf2::Transform & dock_pose);


// callback
void robot_pose_callback(aruco_msgs::msg::PoseWithId::ConstSharedPtr msg);
// void dock_pose_callback(geometry_msgs::msg::PoseStamped::ConstSharedPtr msg);
void dock_visible_callback(capella_ros_service_interfaces::msg::ChargeMarkerVisible::ConstSharedPtr msg);
void charge_state_callback(capella_ros_service_interfaces::msg::ChargeState::ConstSharedPtr msg);



rclcpp_action::GoalResponse handle_dock_servo_goal(
	const rclcpp_action::GoalUUID & uuid,
	std::shared_ptr<const capella_ros_dock_msgs::action::Dock::Goal> goal);

rclcpp_action::CancelResponse handle_dock_servo_cancel(
	const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<capella_ros_dock_msgs::action::Dock> > goal_handle);

void handle_dock_servo_accepted(
	const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<capella_ros_dock_msgs::action::Dock> > goal_handle);

BehaviorsScheduler::optional_output_t execute_dock_servo(
	const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<capella_ros_dock_msgs::action::Dock> > goal_handle,
	const RobotState & current_state);

rclcpp_action::GoalResponse handle_undock_goal(
	const rclcpp_action::GoalUUID & uuid,
	std::shared_ptr<const capella_ros_service_interfaces::action::Undock::Goal> goal);

rclcpp_action::CancelResponse handle_undock_cancel(
	const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<capella_ros_service_interfaces::action::Undock> > goal_handle);

void handle_undock_accepted(
	const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<capella_ros_service_interfaces::action::Undock> > goal_handle);

BehaviorsScheduler::optional_output_t execute_undock(
	const std::shared_ptr<
		rclcpp_action::ServerGoalHandle<capella_ros_service_interfaces::action::Undock> > goal_handle,
	const RobotState & current_state);

rclcpp_action::Server<capella_ros_dock_msgs::action::Dock>::SharedPtr docking_action_server_;
rclcpp_action::Server<capella_ros_service_interfaces::action::Undock>::SharedPtr undocking_action_server_;


rclcpp::Subscription<capella_ros_service_interfaces::msg::ChargeMarkerVisible>::SharedPtr dock_visible_sub_;
rclcpp::Subscription<capella_ros_service_interfaces::msg::ChargeState>::SharedPtr charge_state_sub_;
rclcpp::Subscription<aruco_msgs::msg::PoseWithId>::SharedPtr robot_pose_sub_;
rclcpp::Subscription<capella_ros_msg::msg::Velocities>::SharedPtr raw_vel_sub_;
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laserScan_sub_;

rclcpp::Subscription<std_msgs::msg::String>::SharedPtr charger_id_sub_;
rclcpp::Subscription<aruco_msgs::msg::MarkerAndMacVector>::SharedPtr marker_and_mac_sub_;

rclcpp::Clock::SharedPtr clock_;
rclcpp::Logger logger_;
std::shared_ptr<BehaviorsScheduler> behavior_scheduler_;
std::atomic<bool> is_docked_ {false};
std::atomic<bool> sees_dock_ {false};
std::atomic<bool> running_dock_action_ {false};
std::atomic<bool> running_undock_action_ {false};
std::shared_ptr<SimpleGoalController> goal_controller_;
std::mutex robot_pose_mutex_;
tf2::Transform last_robot_pose_;
std::mutex dock_pose_mutex_;
tf2::Transform last_dock_pose_;
rclcpp::Time action_start_time_;
const rclcpp::Duration max_action_runtime_;
double last_docked_distance_offset_ {0.32};
bool calibrated_offset_ {false};
double MAX_DOCK_INTERMEDIATE_GOAL_OFFSET {0.6};   // 0.5 + 0.1
double UNDOCK_GOAL_OFFSET {0.5};
rclcpp::Time last_feedback_time_;
const rclcpp::Duration report_feedback_interval_ {std::chrono::seconds(1)};
capella_ros_msg::msg::Velocities raw_vel_msg;
nav_msgs::msg::Odometry odom_msg;
motion_control_params *params_ptr;

rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr undock_state_pub_;
rclcpp::Publisher<capella_ros_dock_msgs::msg::ChargerContactConditionType>::SharedPtr charger_contact_condition_type_pub_;

void raw_vel_sub_callback(capella_ros_msg::msg::Velocities);
void odom_sub_callback(nav_msgs::msg::Odometry);

void laserScan_sub_callback(sensor_msgs::msg::LaserScan);

void charger_id_callback(std_msgs::msg::String);
void marker_and_mac_callback(aruco_msgs::msg::MarkerAndMacVector);

bool undock_has_obstacle_ = false;
// sin/cos table generated or not
bool has_table = false;
float laser_min_angle;
float laser_angle_increament;
int laser_data_size;
std::vector<float> sin_table;
std::vector<float> cos_table;

int marker_id_;
std::string charger_id_;
aruco_msgs::msg::MarkerAndMacVector marker_and_mac_vector;
bool bluetooth_connected{false};

std::string state;
std::string infos;

int charger_contact_condition_type = 0; // default 0 => only use bluetooth data


void generate_sin_cos_table(float theta_min, float increament, int size);
bool check_undock_has_obstale(sensor_msgs::msg::LaserScan);
};

}  // namespace capella_ros_dock
#endif  // CAPELLA_ROS_DOCK__DOCKING_BEHAVIOR_HPP_
