

#ifndef CAPELLA_ROS_DOCK__MOTION_CONTROL_NODE_HPP_
#define CAPELLA_ROS_DOCK__MOTION_CONTROL_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "capella_ros_dock/docking_behavior.hpp"
#include "rclcpp/rclcpp.hpp"
#include "capella_ros_dock_msgs/msg/hazard_detection_vector.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

namespace capella_ros_dock
{

class MotionControlNode : public rclcpp::Node
{
public:
/// @brief Constructor
explicit MotionControlNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
/// \brief Function to centralize velocity command for system
void control_robot();

///  @brief Function to start and stop timer for control_robot()
void start_control_timer_callback();

/// @brief declare and get parameters
void init_params();

/// @brief callback for hazard_detection topic
void cb_hazard_detection(capella_ros_dock_msgs::msg::HazardDetectionVector::SharedPtr msg);

/// @brief callback for /charger/pose topic
void cb_charger_pose(geometry_msgs::msg::PoseWithCovarianceStamped msg);


/// @brief subscription to hazards
rclcpp::Subscription<capella_ros_dock_msgs::msg::HazardDetectionVector>::SharedPtr sub_hazards_;
rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_charger_pose_;
rclcpp::CallbackGroup::SharedPtr cb_group_hazards_;
rclcpp::CallbackGroup::SharedPtr cb_group_charger_pose_;

rclcpp::TimerBase::SharedPtr control_timer_ {nullptr};
rclcpp::TimerBase::SharedPtr start_control_timer {nullptr};
std::shared_ptr<BehaviorsScheduler> scheduler_ {nullptr};
std::shared_ptr<DockingBehavior> docking_behavior_ {nullptr};
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_out_pub_;
std::mutex current_state_mutex_;
motion_control_params params;
RobotState current_state_;

std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

bool getTransform(
	const std::string & refFrame, const std::string & childFrame,
	geometry_msgs::msg::TransformStamped & transform);
};

}  // namespace capella_ros_dock

#endif  //  CAPELLA_ROS_DOCK__MOTION_CONTROL_NODE_HPP_
