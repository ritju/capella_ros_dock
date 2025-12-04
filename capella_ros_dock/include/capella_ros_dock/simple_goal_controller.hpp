

#ifndef CAPELLA_ROS_DOCK__SIMPLE_GOAL_CONTROLLER_HPP_
#define CAPELLA_ROS_DOCK__SIMPLE_GOAL_CONTROLLER_HPP_

#include <deque>
#include <functional>
#include <mutex>
#include <vector>

#include "angles/angles.h"
#include "boost/optional.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "capella_ros_dock/behaviors_scheduler.hpp"
#include "capella_ros_dock/behaviors_scheduler.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <chrono>
#include <time.h>
#include <rclcpp/time.hpp>
#include <capella_ros_msg/msg/velocities.hpp>
#include "rclcpp/rclcpp.hpp"
#include "capella_ros_dock/utils.hpp"
#include <inttypes.h>
#include <nav_msgs/msg/odometry.hpp>
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include <chrono>
#include <magic_enum.hpp>


using namespace std;
using namespace chrono_literals;

namespace capella_ros_dock
{


/**
 * @brief This class provides an API to give velocity commands given a goal and robot position.
 */
class SimpleGoalController
{
public:
SimpleGoalController(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_,
                     rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
                     rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
                     motion_control_params *params_ptr)
	: logger_(node_logging_interface->get_logger()),
	clock_(node_clock_interface->get_clock())
{
	this->node_ = node_;
	this->params_ptr = params_ptr;
	init(params_ptr);
}

void init(motion_control_params* params_ptr)
{
	robot_info_ = RobotInfo();
	buffer_goal_point_x = -(params_ptr->offset_last_docked_distance
	                        + params_ptr->offset_low_speed
	                        + params_ptr->offset_seconde_goal
	                        + params_ptr->offset_buffer_goal);
	buffer_goal_point_y = 0.0 + params_ptr->goal_y_correction;

	float camera_horizontal_view, marker_size, camera_baselink_dis, goal_dis_x;
	camera_horizontal_view = degree_to_radian(params_ptr->camera_horizontal_view);
	marker_size = params_ptr->marker_size;
	camera_baselink_dis = params_ptr->camera_baselink_dis;
	goal_dis_x = params_ptr->offset_last_docked_distance + params_ptr->offset_low_speed + params_ptr->offset_seconde_goal;
	RCLCPP_INFO(rclcpp::get_logger("simple_goal_controller"), "camera_horizontal_view: %.2f, marker_size: %.2f, camera_baselink_dis: %.2f, goal_dis_x: %.2f",
	            camera_horizontal_view, marker_size, camera_baselink_dis, goal_dis_x);

	float d1, d2, d3, alpha;
	d1 = goal_dis_x;
	d2 = camera_baselink_dis;
	d3 = marker_size * 0.5;
	alpha = camera_horizontal_view * 0.5;
	float tan_alpha = std::tan(alpha);
	RCLCPP_INFO(rclcpp::get_logger("simple_goal_controller"), "d1: %.2f, d2: %.2f, d3: %.2f, alpha: %.2f", d1, d2, d3, alpha);
	float r = std::hypot(d1 * tan_alpha - d3, d1 + d3 * tan_alpha);
	float x1 = d2 * tan_alpha;
	float x2 = d1 * tan_alpha - d3;
	float beta_plus_theta = std::acos(x1 / r);
	float beta = std::acos(x2 / r);
	thre_angle_diff = beta_plus_theta - beta;
	RCLCPP_INFO(rclcpp::get_logger("simple_goal_controller"), "r: %.2f, x1: %.2f, x2: %.2f, beta_plus_theta: %.2f, beta: %.2f", r, x1, x2, beta_plus_theta, beta);
	RCLCPP_INFO(rclcpp::get_logger("simple_goal_controller"), "thre_angle_diff: %.2f", thre_angle_diff);

	// 计算 marker是否在相机的视野范围内，不再使用该方法，已经计算了机器人朝向目标点的方向的最大允许角度thre_angle_diff
	// 	if (sees_dock)
	// 	{
	// 		auto robot_pose = current_pose.getOrigin();
	// 		float x, y, theta;
	// 		x = robot_pose[0], y = robot_pose[1];
	// 		theta = tf2::getYaw(current_pose.getRotation());
	// 		float goal_x, goal_y;
	// 		goal_x = -goal_dis_x;
	// 		goal_y = 0;
	// 		float theta_to_goal;
	// 		theta_to_goal = std::atan2(goal_y - y, goal_x - x);
	// 		if (camera_horizontal_view * 0.5 < std::abs(theta_to_goal))
	// 		{
	// 			RCLCPP_INFO(logger_, "************failed************");
	// 			RCLCPP_INFO(logger_, "x: %.2f, y: %.2f, theta: %.2f, theta_to_goal: %.2f", x, y, theta, theta_to_goal);
	// 			RCLCPP_INFO(logger_, "camera_horizontal_view/2: %.2f< theta_to_goal: %.2f",
	// 				camera_horizontal_view * 0.5, std::abs(theta_to_goal));
	// 		}
	// 		else
	// 		{
	// 			float y_coord = camera_horizontal_view_y_coord(std::abs(theta_to_goal), camera_horizontal_view, camera_baselink_dis, goal_dis_x);
	// 			if (y_coord > marker_size * 0.5)
	// 			{
	// 				RCLCPP_INFO(logger_, "============success============");
	// 				RCLCPP_INFO(logger_, "x: %.2f, y: %.2f, theta: %.2f, theta_to_goal: %.2f", x, y, theta, theta_to_goal);
	// 				RCLCPP_INFO(logger_, "y_coord: %.2f", y_coord);
	// 			}
	// 			else
	// 			{
	// 				RCLCPP_INFO(logger_, "************failed************");
	// 				RCLCPP_INFO(logger_, "x: %.2f, y: %.2f, theta: %.2f, theta_to_goal: %.2f", x, y, theta, theta_to_goal);
	// 				RCLCPP_INFO(logger_, "y_coord: %.2f", y_coord);
	// 			}
	// 		}

	// 	}
	// 	else
	// 	{
	// 		RCLCPP_INFO(logger_, "can not see marker.");
	// 	}
}

/// \brief Structure to keep information for each point in commanded path
//  including pose with position and orientation of point
//  radius that is considered close enough to achieving the point
//  drive_backwards whether the robot should drive backwards towards the point (for docking)
struct CmdPathPoint
{
	CmdPathPoint(tf2::Transform p, float r, bool db)
		: pose(p), radius(r), drive_backwards(db) {
	}
	tf2::Transform pose;
	float radius;
	bool drive_backwards;
};
using CmdPath = std::vector<CmdPathPoint>;

/// \brief Set goal path for controller along with max rotation and translation speed
void initialize_goal(const CmdPath & cmd_path)
{
	RCLCPP_INFO(logger_, "初始化cmd_path, 初始化当前state为: %s", magic_enum::enum_name(current_state_).data());
	const std::lock_guard<std::mutex> lock(mutex_);
	// Convert path points to goal points
	goal_points_.clear();
	goal_points_.resize(cmd_path.size());
	for (size_t i = 0; i < cmd_path.size(); ++i) {
		GoalPoint & gp = goal_points_[i];
		const tf2::Vector3 & pt_position = cmd_path[i].pose.getOrigin();
		gp.x = pt_position.getX();
		gp.y = pt_position.getY();
		gp.theta = tf2::getYaw(cmd_path[i].pose.getRotation());
		gp.radius = cmd_path[i].radius;
		gp.drive_backwards = cmd_path[i].drive_backwards;
	}
	current_state_ = NavigateStates::INIT;
}

/// \brief Clear goal
void reset()
{
	const std::lock_guard<std::mutex> lock(mutex_);
	goal_points_.clear();
}

// \brief Generate velocity based on current position and next goal point looking for convergence
// with goal point based on radius.
// \return empty optional if no goal or velocity command to get to next goal point
BehaviorsScheduler::optional_output_t get_velocity_for_position(
	const tf2::Transform & current_pose, const tf2::Transform & robot_pose_map, const tf2::Transform & charger_pose_map, bool sees_dock, bool is_docked, bool bluetooth_connected,
	nav_msgs::msg::Odometry odom_msg, capella_ros_dock_msgs::msg::HazardDetectionVector hazards, std::string & state, std::string & infos, bool& b_timeout_current_state,
	nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>  collision_checker, std::vector<geometry_msgs::msg::Point> footprint_vec, rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_clear_entire_local_costmap)
{
	save_all_poses_infos(robot_pose_map, current_pose, charger_pose_map, sees_dock);

	// impl undock (go to undock state)
	if (goal_points_.size() >0 && !(goal_points_.front().drive_backwards))
	{
		if (current_state_ != NavigateStates::UNDOCK)
		{
			change_state(current_state_, NavigateStates::UNDOCK, clock_->now().seconds(), params_ptr->undock_timeout);
			undock_dis_moved_ = 0.0;
		}
		sleep(0.5); // wait for /charger/stop to execute.
		undocking = true;
	}
	else
	{
		undocking = false;
	}

	time_start = std::chrono::high_resolution_clock::now();
	BehaviorsScheduler::optional_output_t servo_vel;
	const std::lock_guard<std::mutex> lock(mutex_);
	if (is_docked && !undocking)
	{
		if(first_contacted)
		{
			first_contacted = false;
			first_contacted_time = clock_->now().seconds();
			RCLCPP_DEBUG(logger_, "keep moving until %.2f expired.", params_ptr->contacted_keep_move_time);
		}
		else
		{
			now_time_ = clock_->now().seconds();
			if ((now_time_ - first_contacted_time) > params_ptr->contacted_keep_move_time)
			{
				RCLCPP_INFO(logger_, "*************** robot is docked *************");
				goal_points_.clear();
				start_time_recorded = false;
				first_contacted = true;
			}
		}
	}
	if (goal_points_.size() == 0) {
		RCLCPP_INFO(logger_, "*************** goal_points.size() = 0 *************");
		state = std::string("goal_points.size() = 0");
		infos = "Reason: goal_points.size() = 0 ==> stop ...";
		return servo_vel;
	}

	double current_angle;
	tf2::Vector3 current_position;
	// if (current_state_ >= NavigateStates::ANGLE_TO_GOAL)
	// {
	current_angle = tf2::getYaw(current_pose.getRotation());
	current_position = current_pose.getOrigin();
	// }

	// stop when has valid hazards
	if(hazards_valid(current_pose, hazards) && current_state_ > NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION)
	{
		last_time_cannot_see_dock = clock_->now();
		RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "stop for hazards.");
		auto distance = std::abs(current_pose.getOrigin().getX());
		RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "distance: %.2f", distance);
		RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "throttle: %.2f", params_ptr->dock_valid_obstacle_x);
		servo_vel = geometry_msgs::msg::Twist();
		state = std::string(" > ANGLE_TO_X_POSITIVE_ORIENTATION");
		infos = std::string("Reason: have valid hazards and navigate_state > ANGLE_TO_X_POSITIVE_ORIENTATION  ==> stop ...");
		return servo_vel;
	}
	if (sees_dock)
	{
		first_cannot_see_dock = true;
	}


	if ((current_state_ > NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION) && need_get_outof_charger_range && (clock_->now().seconds() - last_time_cannot_see_dock.seconds()) < (params_ptr->time_sleep + 2))
	{
		servo_vel = geometry_msgs::msg::Twist();
		servo_vel->linear.x = 0.15;
		state = std::string("get_outof_charger_range");
		infos = std::string("Reason: get_outof_charger_range executing ......");
		RCLCPP_INFO_THROTTLE(logger_, *clock_, 400, "get_outof_charger_range executing");
		return servo_vel;
	}
	if (((clock_->now().seconds() - last_time_cannot_see_dock.seconds()) > (params_ptr->time_sleep + 2)) && (!get_out_of_charger_range_completed))
	{
		need_get_outof_charger_range = false;
		get_out_of_charger_range_completed = true;
		change_state(current_state_, NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION, clock_->now().seconds(), params_ptr->timeout_angle_to_x_positive_orientation);
		servo_vel = geometry_msgs::msg::Twist();
		RCLCPP_INFO(logger_, "get_outof_charge_range completed");
		state = std::string(" get_outof_charge_range completed");
		infos = std::string("Reason: get_outof_charger_range completed, go to state ANGLE_TO_X_POSITIVE_ORIENTATION");
		return servo_vel;
	}

	if(!sees_dock && current_state_ > NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION && !undocking)
	{
		if (first_cannot_see_dock)
		{
			last_time_cannot_see_dock = clock_->now();
			first_cannot_see_dock = false;
		}
		now_time_cannot_see_dock = clock_->now();
		if ((now_time_cannot_see_dock.seconds() - last_time_cannot_see_dock.seconds()) < params_ptr->time_sleep)
		{
			servo_vel = geometry_msgs::msg::Twist();
			state = std::string(" > ANGLE_TO_X_POSITIVE_ORIENTATION");
			infos = std::string("Reason: cannot see dock and navigate_state > ANGLE_TO_X_POSITIVE_ORIENTATION and stop time < time_sleep(default 5s) ==> stop");
			return servo_vel;
		}
		else
		{
			// RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "stop until can see dock.");
			if (std::abs(current_position.getX()) > params_ptr->robot_rotate_radius)
			{
				change_state(current_state_, NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION, clock_->now().seconds(), params_ptr->timeout_angle_to_x_positive_orientation);
				servo_vel = geometry_msgs::msg::Twist();
				state = std::string(" > ANGLE_TO_X_POSITIVE_ORIENTATION");
				infos = std::string("Reason: can not see marker more than time_sleep(default 5s) and navigate_state > ANGLE_TO_X_POSITIVE_ORIENTATION and robot.x > param robot_rotate_radius  ==> stop, change state to ANGLE_TO_X_POSITIVE_ORIENTATION");
				return servo_vel;
			}
			else
			{
				need_get_outof_charger_range = true;
				get_out_of_charger_range_completed = false;
				RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "robot cann't see the charger,but it is too linear to the charger , try to get robot out of charger range......");
				servo_vel = geometry_msgs::msg::Twist();
				state = std::string(" > ANGLE_TO_X_POSITIVE_ORIENTATION");
				infos = std::string("Reason: can not see marker more than time_sleep(default 5s) and navigate_state > ANGLE_TO_X_POSITIVE_ORIENTATION and robot.x < param robot_rotate_radius  ==> stop and try to get robot out of charger range......");
				return servo_vel;
			}
		}
	}

	// Generate velocity based on current position and next goal point looking for convergence
	// with goal point based on radius.
	switch (current_state_) {
	case NavigateStates::INIT:
	{
		print_current_state_debug(current_state_);
		servo_vel = geometry_msgs::msg::Twist();
		change_state(current_state_, NavigateStates::LOOKUP_MARKER, clock_->now().seconds(), 1.0); // INIT 状态只执行一次，不涉及超时问题，为保持一致，timout参数设置为1.0
		break;
	}
	case NavigateStates::LOOKUP_MARKER:
	{
		print_current_state_debug(current_state_);
		servo_vel = geometry_msgs::msg::Twist();

		b_timeout_current_state = check_current_state_timeout();
		if (b_timeout_current_state)
		{
			return servo_vel;		
		}
		
		auto angle_robot = tf2::getYaw(robot_pose_map.getRotation());
		double x_charger, y_charger, x_robot, y_robot;
		x_robot = robot_pose_map.getOrigin()[0];
		y_robot = robot_pose_map.getOrigin()[1];
		x_charger = charger_pose_map.getOrigin()[0];
		y_charger = charger_pose_map.getOrigin()[1];
		auto angle_charger_to_robot = std::atan2(y_robot - y_charger, x_robot - x_charger);
		auto dist_angle = angles::shortest_angular_distance(angle_robot, angle_charger_to_robot);

		if (!sees_dock)
		{
			RCLCPP_DEBUG(logger_, "x_robot_map: %.2f, y_robot_map: %.2f", x_robot, y_robot);
			RCLCPP_DEBUG(logger_, "x_charger_map: %.2f, y_charger_map: %.2f", x_charger, y_charger);

			RCLCPP_DEBUG(logger_, "angle_robot: %.2f", angle_robot);
			RCLCPP_DEBUG(logger_, "angle_charger_to_robot: %.2f", angle_charger_to_robot);
			RCLCPP_DEBUG(logger_, "dist_angle: %.2f", dist_angle);

			start_time_recorded = false;
			RCLCPP_DEBUG(logger_, "Need rotate robot for it can see the marker.");
			
			if (std::abs(dist_angle) > params_ptr->tolerance_angle)
			{
				double dist_angle_copy = dist_angle;
				bound_rotation(dist_angle_copy, params_ptr->min_rotation, params_ptr->max_rotation);
				servo_vel->angular.z = dist_angle_copy;
				RCLCPP_DEBUG(logger_, "angular_z: %.2f", servo_vel->angular.z);
			}
			else
			{
				servo_vel->angular.z = 0.0;
				state = std::string("LOOKUP_MARKER");
				infos = std::string("Reason: The robot has rotated towards the charging station direction, but it can not see the marker!"); 
				RCLCPP_WARN_THROTTLE(logger_, *clock_, 3000, "%s", infos.c_str());
			}

			// before actually begin rotation, collision_check first
			// current_state: LOOKUP_MARKER
			double remaining_rotation_time = std::abs(dist_angle / servo_vel->angular.z);
			double predict_time = std::min(double(params_ptr->collision_predict_time), remaining_rotation_time);
			RCLCPP_DEBUG(logger_, "predict_time: %.2f", predict_time);
			if (params_ptr->collision_check)
			{
				double cost_value = get_cost_value(logger_,collision_checker, robot_pose_map, footprint_vec, true, 0.0, servo_vel->angular.z,
													predict_time, params_ptr->cmd_vel_hz, params_ptr->odom_twist_scale);
				if (cost_value >= nav2_costmap_2d::LETHAL_OBSTACLE)
				{
					RCLCPP_DEBUG(logger_, "cost value: %.2f >= %.2f", cost_value,  static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));
					servo_vel->angular.z = 0.0;
					RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "stop for collision check, when %s", magic_enum::enum_name(current_state_).data());

					if(params_ptr->enable_clear_local_costmap)
					{
						clear_local_costmap(client_clear_entire_local_costmap);
					}

					return servo_vel;
				}
			}
			else
			{
				RCLCPP_DEBUG(logger_, "collision_check: %s", params_ptr->collision_check ? "true":"false");
			}

			state = std::string("LOOKUP_MARKER");
			infos = std::string("Reason: camera's orientation not towards charger's marker ==> rotate robot");
		}
		else
		{
			RCLCPP_DEBUG(logger_, "waiting for find the best coordinate for robot in marker frame");
			double x_,y_, theta_;
			x_ = current_pose.getOrigin()[0];
			y_ = current_pose.getOrigin()[1];
			theta_ = tf2::getYaw(current_pose.getRotation());
			RCLCPP_DEBUG(logger_, "robot_x: %.2f", x_);
			RCLCPP_DEBUG(logger_, "robot_y: %.2f", y_);
			RCLCPP_DEBUG(logger_, "robot_theta: %.2f", theta_);

			if(!start_time_recorded)
			{
				start_time_recorded = true;
				waiting_for_best_coord_start_time = clock_->now().seconds();
				RCLCPP_DEBUG(logger_, "waiting_for_best_coord_start_time : %.2f", waiting_for_best_coord_start_time);
				state = std::string("LOOKUP_MARKER");
				infos = std::string("Reason: camera's orientation first towards charger's marker ==> record the start time.");
			}
			else
			{
				now_time_ = clock_->now().seconds();
				double time_wating = now_time_ - waiting_for_best_coord_start_time;

				if (time_wating < params_ptr->localization_converged_time)
				{
					RCLCPP_DEBUG(logger_, "coords not converged, wait %.2f seconds.", time_wating);
					state = std::string("LOOKUP_MARKER");
					infos = std::string("Reason: coords not converged ==> just wait.");
				}
				else
				{
					RCLCPP_DEBUG(logger_, "coords converged, change state.");
					start_time_recorded = false;

					float distance_tmp = params_ptr->offset_last_docked_distance
											+ params_ptr->offset_low_speed
											+ params_ptr->offset_seconde_goal;
					double theta = std::atan2(std::abs(robot_y_charger_), std::abs(robot_x_charger_) - distance_tmp);
					RCLCPP_DEBUG(logger_, "robot_x_charger: %.2f", robot_x_charger_);
					RCLCPP_DEBUG(logger_, "robot_y_charger: %.2f", robot_y_charger_);
					RCLCPP_DEBUG(logger_, "robot_yaw_charger: %.2f", robot_yaw_charger_);					

					double base_link_y, base_link_x;
					base_link_y = robot_y_charger_ - params_ptr->base_link_dummy_dis * std::sin(robot_yaw_charger_);
					base_link_x = robot_x_charger_ - params_ptr->base_link_dummy_dis * std::cos(robot_yaw_charger_);

					RCLCPP_DEBUG(logger_, "base_link_x: %.2f", base_link_x);
					RCLCPP_DEBUG(logger_, "base_link_y: %.2f", base_link_y);

					// 三个判断条件
					RCLCPP_DEBUG(logger_, "theta: %.2f", theta); 
					RCLCPP_DEBUG(logger_, "thre_angle_diff: %.2f", thre_angle_diff);

					RCLCPP_DEBUG(logger_, "std::abs(base_link_y): %.2f", std::abs(base_link_y));
					RCLCPP_DEBUG(logger_, " params_ptr->base_link_y_thr: %.2f",  params_ptr->base_link_y_thr);

					RCLCPP_DEBUG(logger_, "std::abs(robot_x_charger_): %.2f", std::abs(robot_x_charger_));
					RCLCPP_DEBUG(logger_, "distance_tmp + params_ptr->deviate_second_goal_x: %.2f", distance_tmp + params_ptr->deviate_second_goal_x);

					if (theta < thre_angle_diff // 角度小于阀值
						&& std::abs(base_link_y) < params_ptr->base_link_y_thr // y坐标(左右)小于阀值
						&& std::abs(robot_x_charger_) > (distance_tmp + params_ptr->deviate_second_goal_x))// x坐标(前后) > （second_goal + 阀值）                                                                                                                                                                                   // 0.7 <= 0.5 + 0.2(x_error)
					{
						RCLCPP_DEBUG(logger_, "robot change state to angle_to_goal");
						change_state(current_state_, NavigateStates::ANGLE_TO_GOAL, clock_->now().seconds(), params_ptr->timeout_angle_to_goal);
						state = std::string("LOOKUP_MARKER");
						infos = std::string("Reason: robot's position converged ==> directly change state to ANGLE_TO_GOAL");
					}
					else
					{
						RCLCPP_DEBUG(logger_, "robot change state to angle_to_buffer_point");
						RCLCPP_DEBUG(logger_, "buffer_goal_point_x: %.2f, buffer_goal_point_y: %.2f", buffer_goal_point_x, buffer_goal_point_y);

						tf_before_angle_to_buffer_point = robot_pose_map;

						double buffer_goal_point_x_base_link = buffer_goal_point_x - params_ptr->base_link_dummy_dis;
						double buffer_goal_point_y_base_link = buffer_goal_point_y;
						RCLCPP_DEBUG(logger_, "buffer_goal_point_x_base_link: %.2f, buffer_goal_point_y_base_link: %.2f",
							buffer_goal_point_x_base_link, buffer_goal_point_y_base_link);

						dist_buffer_point = std::hypot(base_link_x - buffer_goal_point_x_base_link,
														base_link_y - buffer_goal_point_y_base_link);

						dist_move_to_buffer_point = dist_buffer_point;

						robot_angle_to_buffer_point_yaw = std::atan2(buffer_goal_point_y_base_link - base_link_y,
																		buffer_goal_point_x_base_link - base_link_x);

						// decide drive back or not
						robot_current_yaw = robot_yaw_charger_;
						theta_positive = angles::shortest_angular_distance(
							angles::normalize_angle(robot_current_yaw + M_PI),
							robot_angle_to_buffer_point_yaw);
						theta_negative = angles::shortest_angular_distance(robot_current_yaw,
																			robot_angle_to_buffer_point_yaw);
						if (std::abs(theta_positive) < std::abs(theta_negative))
						{
							drive_back = false;
							dist_buffer_point_yaw = theta_positive;
						}
						else
						{
							drive_back = true;
							dist_buffer_point_yaw = theta_negative;
						}

						theta_angle_to_buffer_point = dist_buffer_point_yaw;

						RCLCPP_DEBUG(logger_, "robot_current_yaw: %.2f", robot_current_yaw);
						RCLCPP_DEBUG(logger_, "robot_angle_to_buffer_point_yaw: %.2f", robot_angle_to_buffer_point_yaw);
						RCLCPP_DEBUG(logger_, "theta_negative: %.2f", theta_negative);
						RCLCPP_DEBUG(logger_, "theta_positive: %.2f", theta_positive);
						RCLCPP_DEBUG(logger_, "dist_buffer_point: %.2f", dist_buffer_point);
						RCLCPP_DEBUG(logger_, "dist_buffer_point_yaw: %.2f", dist_buffer_point_yaw);
						update_time_smart();
						change_state(current_state_, NavigateStates::ANGLE_TO_BUFFER_POINT, clock_->now().seconds(), params_ptr->timeout_angle_to_buffer_point);
						state = std::string("LOOKUP_MARKER");
						infos = std::string("Reason: robot's position not converged ==> directly change state to ANGLE_TO_BUFFER_POINT");
					}
				}
			}
		}
		break;
	}
	case NavigateStates::ANGLE_TO_BUFFER_POINT:
	{
		print_current_state_debug(current_state_);
		servo_vel = geometry_msgs::msg::Twist();

		b_timeout_current_state = check_current_state_timeout();
		if (b_timeout_current_state)
		{
			return servo_vel;		
		}

		update_time_smart();

		RCLCPP_DEBUG(logger_, "delta_time: %.2f", delta_time_);
		RCLCPP_DEBUG(logger_, "angular.z: %.2f", odom_msg.twist.twist.angular.z);
		RCLCPP_DEBUG(logger_, "delta_angular: %.2f", odom_msg.twist.twist.angular.z * delta_time_);
		RCLCPP_DEBUG(logger_, "dist_buffer_point_yaw pre: %.2f", dist_buffer_point_yaw);
		dist_buffer_point_yaw -= odom_msg.twist.twist.angular.z * delta_time_;
		robot_current_yaw += odom_msg.twist.twist.angular.z * delta_time_;
		RCLCPP_DEBUG(logger_, "dist_buffer_point_yaw now: %.2f", dist_buffer_point_yaw);
		double angle_dist = dist_buffer_point_yaw;
		RCLCPP_DEBUG(logger_, "angle_dist: %.2f", angle_dist);
		RCLCPP_DEBUG(logger_, "robot_map_yaw: %.2f", tf2::getYaw(robot_pose_map.getRotation()));
		if(std::abs(angle_dist) < params_ptr->tolerance_angle)
		{
			RCLCPP_DEBUG(logger_, "change state to move_to_buffer_point.");
			tf_after_angle_to_buffer_point = robot_pose_map;
			double theta_delta_map = tf2::getYaw((tf_before_angle_to_buffer_point.inverse() * tf_after_angle_to_buffer_point).getRotation());
			RCLCPP_DEBUG(logger_, "dist_angle_to_buffer_point: %.2f, theta_delta: %.2f", theta_angle_to_buffer_point, theta_delta_map);
			change_state(current_state_, NavigateStates::MOVE_TO_BUFFER_POINT, clock_->now().seconds(), params_ptr->timeout_move_to_buffer_point);
			state = std::string("ANGLE_TO_BUFFER_POINT");
			infos = std::string("Reason: ANGLE_TO_BUFFER_POINT converged ==> change state to MOVE_TO_BUFFER_POINT");
		}
		else
		{
			// before rotation, collision_check first
			double x, y, theta;
			x = robot_pose_map.getOrigin().getX();
			y = robot_pose_map.getOrigin().getY();
			theta = tf2::getYaw(robot_pose_map.getRotation());
			double cost = collision_checker.footprintCostAtPose(x, y, theta, footprint_vec);
			// double cost = collision_checker.footprintCost(footprint_vec);
			RCLCPP_DEBUG(logger_, "x: %.2f, y: %.2f, theta: %.2f", x, y, theta);
			for (size_t index = 0; index < footprint_vec.size(); index++)
			{
				RCLCPP_DEBUG(logger_, "footprint Point(%.2f, %.2f)", footprint_vec[index].x, footprint_vec[index].y);
			}
			if ((cost >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) && params_ptr->collision_check)
			{
				RCLCPP_DEBUG(logger_, "cost value: %.2f >= %.2f", cost, static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));
				servo_vel->angular.z = 0.0;
				return servo_vel;
			}
			else
			{
				RCLCPP_DEBUG(logger_, "cost value: %.2f, go on ......", cost);
				RCLCPP_DEBUG(logger_, "collision_check: %s", params_ptr->collision_check ? "true":"false");
			}


			bound_rotation(angle_dist, params_ptr->min_rotation, params_ptr->max_rotation);
			if(std::abs(angle_dist) < params_ptr->min_rotation)                  // 0.1 => 0.08 => raw_vel output 0
			{
				angle_dist = std::copysign(params_ptr->min_rotation, angle_dist);
			}
			servo_vel->angular.z = angle_dist;
			RCLCPP_DEBUG(logger_, "angular.z: %.2f", angle_dist);

			// before actually begin rotation, collision_check first
			// current state: ANGLE_TO_BUFFER_POINT
			double remaining_rotation_time = std::abs(dist_buffer_point_yaw / servo_vel->angular.z);
			double predict_time = std::min(double(params_ptr->collision_predict_time), remaining_rotation_time);
			RCLCPP_DEBUG(logger_, "predict_time: %.2f", predict_time);
			if (params_ptr->collision_check)
			{
				double cost_value = get_cost_value(logger_,collision_checker, robot_pose_map, footprint_vec, true, 0.0, servo_vel->angular.z,
				                                   predict_time, params_ptr->cmd_vel_hz, params_ptr->odom_twist_scale);
				if (cost_value >= nav2_costmap_2d::LETHAL_OBSTACLE)
				{
					RCLCPP_DEBUG(logger_, "cost value: %.2f >= %.2f", cost_value,  static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));
					servo_vel->angular.z = 0.0;
					RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "stop for collision check, when  %s", magic_enum::enum_name(current_state_).data());

					if(params_ptr->enable_clear_local_costmap)
					{
						clear_local_costmap(client_clear_entire_local_costmap);
					}

					return servo_vel;
				}
			}
			else
			{
				RCLCPP_DEBUG(logger_, "collision_check: %s", params_ptr->collision_check ? "true":"false");
			}

			state = std::string("ANGLE_TO_BUFFER_POINT");
			infos = std::string("Reason: ANGLE_TO_BUFFER_POINT not converged ==> keep on rotating robot");
		}
		break;
	}
	case NavigateStates::MOVE_TO_BUFFER_POINT:
	{
		print_current_state_debug(current_state_);
		servo_vel = geometry_msgs::msg::Twist();

		b_timeout_current_state = check_current_state_timeout();
		if (b_timeout_current_state)
		{
			return servo_vel;		
		}

		update_time_smart();

		dist_buffer_point -= delta_time_ * std::abs(odom_msg.twist.twist.linear.x);
		double dist_y = dist_buffer_point;
		RCLCPP_DEBUG(logger_, "odom_msg.linear_x: %.2f", odom_msg.twist.twist.linear.x);
		RCLCPP_DEBUG(logger_, "delta_time: %.2f", delta_time_);
		RCLCPP_DEBUG(logger_, "dist_buffer_point now: %.2f", dist_buffer_point);
		RCLCPP_DEBUG(logger_, "dist_y: %.2f", dist_y);
		RCLCPP_DEBUG(logger_, "robot_map_x: %.2f, robot_map_y: %.2f", robot_pose_map.getOrigin().getX(), robot_pose_map.getOrigin().getY());
		if (std::abs(dist_y) < params_ptr->tolerance_r)
		{
			RCLCPP_DEBUG(logger_, "change state to angle_to_x_positive_orientation");
			tf_after_move_to_buffer_point = robot_pose_map;
			tf2::Transform tf_ = tf_after_angle_to_buffer_point.inverse() * tf_after_move_to_buffer_point;
			double dist_move_to_buffer_point_delta = std::hypot(tf_.getOrigin().getX(), tf_.getOrigin().getY());
			RCLCPP_DEBUG(logger_, "dist_move_to_buffer_point: %.2f, dist_delta: %.2f", dist_move_to_buffer_point, dist_move_to_buffer_point_delta);

			change_state(current_state_, NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION, clock_->now().seconds(), params_ptr->timeout_angle_to_x_positive_orientation );
			state = std::string("MOVE_TO_BUFFER_POINT");
			infos = std::string("Reason: MOVE_TO_BUFFER_POINT converged ==> change state to ANGLE_TO_X_POSITIVE_ORIENTATION");
		}
		else
		{
			double translate_velocity = dist_y;
			if(drive_back)
			{
				translate_velocity *= -1;
			}
			if (std::abs(translate_velocity) > params_ptr->max_translation) {
				translate_velocity = std::copysign(params_ptr->max_translation, translate_velocity);
			}
			if (std::abs(translate_velocity) < params_ptr->min_translation) {
				translate_velocity = std::copysign(params_ptr->min_translation, translate_velocity);
			}
			servo_vel->linear.x = translate_velocity;
			RCLCPP_DEBUG(logger_, "linear.x: : %.2f", translate_velocity);

			// before actually begin moving, collision_check first
			// current state: MOVE_TO_BUFFER_POINT
			bool need_check_collision = true;
			double x_c2r, yaw_c2r_abs;
			if (params_ptr->collision_check)
			{
				// 如果需要碰撞检查，只有当机器人和充电桩的距离< dock_valid_obstale_x,且朝向充电桩运动时不需要检查是否碰撞
				// 因为马上就要对接上充电桩，机器人必然要和充电桩进行接触
				auto tf_charger_to_robot = charger_pose_map.inverse() * robot_pose_map;
				auto translation_charger_to_robot = tf_charger_to_robot.getOrigin();
				x_c2r = std::abs(translation_charger_to_robot.getX());
				auto orintation_charger_to_robot = tf_charger_to_robot.getRotation();
				yaw_c2r_abs = std::abs(tf2::getYaw(orintation_charger_to_robot));
				RCLCPP_DEBUG(logger_, "x_charge_to_robot: %.2f, dock_valid_obstacle_x: %.2f", x_c2r, params_ptr->dock_valid_obstacle_x);
				RCLCPP_DEBUG(logger_, "yaw_charger_to_robot: %.2f, throttle: %.2f", yaw_c2r_abs, M_PI * 0.5);
				if ((yaw_c2r_abs < M_PI * 0.5) && (x_c2r < params_ptr->dock_valid_obstacle_x))
				{
					need_check_collision = false;
				}
				RCLCPP_DEBUG(logger_, "need_check_collision: %s", need_check_collision?"true":"false");
			}

			double remaining_rotation_time = std::abs(dist_buffer_point / servo_vel->linear.x);
			double predict_time = std::min(double(params_ptr->collision_predict_time), remaining_rotation_time);
			RCLCPP_DEBUG(logger_, "predict_time: %.2f", predict_time);
			if (params_ptr->collision_check && need_check_collision)
			{
				double cost_value = get_cost_value(logger_,collision_checker, robot_pose_map, footprint_vec, false, servo_vel->linear.x, 0.0,
				                                   predict_time, params_ptr->cmd_vel_hz, params_ptr->odom_twist_scale);
				if (cost_value >= nav2_costmap_2d::LETHAL_OBSTACLE)
				{

					RCLCPP_DEBUG(logger_, "cost value: %.2f >= %.2f", cost_value,  static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));
					servo_vel->linear.x = 0.0;
					RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "stop for collision check, when %s", magic_enum::enum_name(current_state_).data());

					if(params_ptr->enable_clear_local_costmap)
					{
						clear_local_costmap(client_clear_entire_local_costmap);
					}

					return servo_vel;
				}
			}
			else
			{
				RCLCPP_DEBUG(logger_, "collision_check: %s", params_ptr->collision_check ? "true":"false");
				RCLCPP_DEBUG(logger_, "need_check_collision: %s", need_check_collision ? "true":"false");
				RCLCPP_DEBUG(logger_, "yaw_c2r_abs: %.2f", yaw_c2r_abs);
				RCLCPP_DEBUG(logger_, "x_c2r: %.2f, dock_valid_obstacle_x: %.2f", x_c2r, params_ptr->dock_valid_obstacle_x);
			}

			state = std::string("MOVE_TO_BUFFER_POINT");
			infos = std::string("Reason: MOVE_TO_BUFFER_POINT not converged ==> keep on moving");
		}
		break;
	}
	case NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION:
	{
		print_current_state_debug(current_state_);
		servo_vel = geometry_msgs::msg::Twist();

		b_timeout_current_state = check_current_state_timeout();
		if (b_timeout_current_state)
		{
			return servo_vel;		
		}

		update_time_smart();

		double theta_charger_to_robot = std::atan2(robot_y_map_ - charger_y_map_, robot_x_map_ - charger_x_map_);
		double dist_yaw_map = angles::shortest_angular_distance(robot_yaw_map_, theta_charger_to_robot);

		RCLCPP_DEBUG(logger_, "marker_visible: %s", sees_dock?"true":"false");
		RCLCPP_DEBUG(logger_, "dist_yaw_map: %.2f", dist_yaw_map);
		RCLCPP_DEBUG(logger_, "delta_time: %.2f", delta_time_);

		if(std::abs(dist_yaw_map) < params_ptr->tolerance_angle) 
		{
			if (marker_visible_) // marker_visible: true
			{
				float distance_tmp = params_ptr->offset_last_docked_distance
									+ params_ptr->offset_low_speed
									+ params_ptr->offset_seconde_goal;
				double theta = std::atan2(std::abs(robot_y_charger_), std::abs(robot_x_charger_) - distance_tmp);
				RCLCPP_DEBUG(logger_, "robot_x_charger: %.2f", robot_x_charger_);
				RCLCPP_DEBUG(logger_, "robot_y_charger: %.2f", robot_y_charger_);
				RCLCPP_DEBUG(logger_, "robot_yaw_charger_: %.2f", robot_yaw_charger_);
				
				RCLCPP_DEBUG(logger_, "robot_theta: %.2f", robot_yaw_charger_);

				double base_link_y;
				base_link_y = robot_y_charger_ - params_ptr->base_link_dummy_dis * std::sin(robot_yaw_charger_);
				RCLCPP_DEBUG(logger_, "base_link_y: %.2f", base_link_y);

				// 三个判断条件
				RCLCPP_DEBUG(logger_, "theta: %.2f", theta);
				RCLCPP_DEBUG(logger_, "thre_angle_diff: %.2f", thre_angle_diff);

				RCLCPP_DEBUG(logger_, "std::abs(base_link_y): %.2f", std::abs(base_link_y));
				RCLCPP_DEBUG(logger_, " params_ptr->base_link_y_thr: %.2f",  params_ptr->base_link_y_thr);

				RCLCPP_DEBUG(logger_, "std::abs(robot_x_charger_): %.2f", std::abs(robot_x_charger_));
				RCLCPP_DEBUG(logger_, "distance_tmp + params_ptr->deviate_second_goal_x: %.2f", distance_tmp + params_ptr->deviate_second_goal_x);

				if (theta < thre_angle_diff  // 角度小于阀值
					&& std::abs(base_link_y) < params_ptr->base_link_y_thr // y坐标(左右)小于阀值
					&& std::abs(robot_x_charger_) > (distance_tmp + params_ptr->deviate_second_goal_x)) // x坐标(前后) > （second_goal + 阀值）
				{
					RCLCPP_INFO(logger_, "converged ==>Change state to ANGLE_TO_GOAL");
					change_state(current_state_, NavigateStates::ANGLE_TO_GOAL, clock_->now().seconds(), params_ptr->timeout_angle_to_goal );
					state = std::string("ANGLE_TO_X_POSITIVE_ORIENTATION => ANGLE_TO_GOAL");
					infos = std::string("Reason: ANGLE_TO_X_POSITIVE_ORIENTATION converged ==> change state to ANGLE_TO_GOAL");
				}
				else
				{
					RCLCPP_INFO(logger_, "To re-execute ANGLE_TO_BUFFER_POINT, change state to LOOKUP_MARKER");
					change_state(current_state_, NavigateStates::LOOKUP_MARKER, clock_->now().seconds(), params_ptr->timeout_lookup_marker );
					state = std::string("ANGLE_TO_X_POSITIVE_ORIENTATION => LOOKUP_MARKER");
					infos = std::string("Reason: ANGLE_TO_X_POSITIVE_ORIENTATION not converged ==> change state to LOOKUP_MARKER");
				}
			}
			else // marker_visible: false
			{
				RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "current_ state: %s, can not see the marker, just waiting ...", magic_enum::enum_name(current_state_).data());
				return servo_vel;
			}
		}
		else // std::abs(dist_yaw_map) >= params_ptr->tolerance_angle
		{
			bound_rotation(dist_yaw_map, params_ptr->min_rotation, params_ptr->max_rotation);
			servo_vel->angular.z = dist_yaw_map;

			// before actually begin rotation, collision_check first
			// current state: ANGLE_TO_X_POSITIVE_ORIENTATION
			double remaining_rotation_time = std::abs(dist_yaw_map / servo_vel->angular.z);
			double predict_time = std::min(double(params_ptr->collision_predict_time), remaining_rotation_time);
			RCLCPP_DEBUG(logger_, "predict_time: %.2f", predict_time);
			if (params_ptr->collision_check)
			{
				double cost_value = get_cost_value(logger_,collision_checker, robot_pose_map, footprint_vec, true, 0.0, servo_vel->angular.z,
				                                   predict_time, params_ptr->cmd_vel_hz, params_ptr->odom_twist_scale);
				if (cost_value >= nav2_costmap_2d::LETHAL_OBSTACLE)
				{
					RCLCPP_DEBUG(logger_, "cost value: %.2f >= %.2f", cost_value,  static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));
					servo_vel->angular.z = 0.0;
					RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "stop for collision check, when  %s", magic_enum::enum_name(current_state_).data());

					if(params_ptr->enable_clear_local_costmap)
					{
						clear_local_costmap(client_clear_entire_local_costmap);
					}

					return servo_vel;
				}
			}
			else // 不需要碰撞检查
			{
				RCLCPP_DEBUG(logger_, "collision_check: %s", params_ptr->collision_check ? "true":"false");
			}
			RCLCPP_DEBUG(logger_, "angular.z: %.2f", servo_vel->angular.z);
		}
		break;
	}
	case NavigateStates::ANGLE_TO_GOAL:
	{
		print_current_state_debug(current_state_);
		servo_vel = geometry_msgs::msg::Twist();

		b_timeout_current_state = check_current_state_timeout();
		if (b_timeout_current_state)
		{
			return servo_vel;		
		}

		const GoalPoint & gp = goal_points_.front();

		RCLCPP_DEBUG(logger_, "goal =>  x: %.2f, y: %.2f, yaw: %.2f",
		             gp.x, gp.y, gp.theta);
		RCLCPP_DEBUG(logger_, "robot =>  x: %.2f, y: %.2f, yaw: %.2f",
		             current_position.getX(), current_position.getY(),
		             current_angle);


		double delta_y, delta_x;
		delta_y = std::abs(gp.y - current_position.getY());
		delta_x = std::abs(gp.x - current_position.getX());
		double dist_to_goal = std::hypot(delta_x, delta_y);
		if (dist_to_goal <= gp.radius || delta_y < params_ptr->dist_error_y_1) {
			servo_vel = geometry_msgs::msg::Twist();
			change_state(current_state_, NavigateStates::GO_TO_GOAL_POSITION, clock_->now().seconds(), params_ptr->timeout_go_to_goal_position );
			pose_x_init_recoreded_ = false;
			state = std::string("ANGLE_TO_GOAL");
			infos = std::string("Reason: ANGLE_TO_GOAL converged ==> change state to GO_TO_GOAL_POSITION");
		}
		else
		{
			double ang = diff_angle(gp, current_position, current_angle, logger_);
			double ang_save = ang;
			RCLCPP_DEBUG(logger_, "diff angle: %.2f", ang);
			bound_rotation(ang, 0.05, 0.10);
			RCLCPP_DEBUG(logger_, "bound angle: %.2f", ang);
			servo_vel = geometry_msgs::msg::Twist();
			// fix bug when robot had angle to marker but y coord error or odom data error,  10 degree(0.174533)
			if (std::abs(ang_save) < params_ptr->angle_to_goal_angle_converged || (sees_dock && std::abs(std::abs(current_angle) - M_PI) <  0.174533)) {
				change_state(current_state_, NavigateStates::GO_TO_GOAL_POSITION, clock_->now().seconds(), params_ptr->timeout_go_to_goal_position );
				pose_x_init_recoreded_ = false;
				RCLCPP_DEBUG(logger_, " ******** change to state GO_TO_GOAL_POSITION ******** ");
				state = std::string("ANGLE_TO_GOAL");
				infos = std::string("Reason: ANGLE_TO_GOAL converged ==> change state to GO_TO_GOAL_POSITION");
			} else {
				servo_vel->angular.z = ang;
				state = std::string("ANGLE_TO_GOAL");
				infos = std::string("Reason: ANGLE_TO_GOAL not converged ==> keep on rotating");
			}
		}
		break;
	}
	case NavigateStates::GO_TO_GOAL_POSITION:
	{
		print_current_state_debug(current_state_);		
		servo_vel = geometry_msgs::msg::Twist();

		b_timeout_current_state = check_current_state_timeout();
		if (b_timeout_current_state)
		{
			return servo_vel;		
		}

		GoalPoint gp = goal_points_.front();;
		if (goal_points_.size() > 1)
		{
			RCLCPP_DEBUG(logger_, "not the first goal.");
		}
		else
		{
			RCLCPP_DEBUG(logger_, "the first goal.");
			gp.x = -(params_ptr->offset_last_docked_distance - 0.02);
		}

		if (!pose_x_init_recoreded_)
		{
			pose_x_init_ = current_pose.getOrigin().getX();
			pose_x_init_recoreded_ = true;
			RCLCPP_DEBUG(logger_, "recored the pose_x: %.2f", pose_x_init_);
		}

		RCLCPP_DEBUG(logger_, "goal =>  x: %.2f, y: %.2f, yaw: %.2f",
		             gp.x, gp.y, gp.theta);
		RCLCPP_DEBUG(logger_, "robot =>  x: %.2f, y: %.2f, yaw: %.2f degree.",
		             current_pose.getOrigin().getX(), current_pose.getOrigin().getY(),
		             current_angle / 3.1415926 * 180.0);
		double delta_y, delta_x;
		delta_y = std::abs(gp.y - current_position.getY());
		delta_x = std::abs(gp.x - current_position.getX());
		double dist_to_goal = std::hypot(delta_x, delta_y);
		double ang = diff_angle(gp, current_position, current_angle, logger_);

		double abs_ang = std::abs(ang);

		double translate_velocity = params_ptr->go_to_goal_translation_max;

		auto robot_abs_x = std::abs(current_position.getX());
		auto dist_low_speed = params_ptr->offset_last_docked_distance + params_ptr->offset_low_speed;
		auto dist_speed_down_length = (params_ptr->go_to_goal_translation_max + params_ptr->go_to_goal_translation_min) / 2.0 *
		                              ((params_ptr->go_to_goal_translation_max - params_ptr->go_to_goal_translation_min) / params_ptr->go_to_goal_linear_acc);
		auto dist_speed_down_range = params_ptr->go_to_goal_translation_max - params_ptr->go_to_goal_translation_min;
		auto dist_speed_down = dist_low_speed + dist_speed_down_length;

		if(robot_abs_x >= std::abs(pose_x_init_))                 // linear_low_speed(0.05)
		{
			translate_velocity = params_ptr->go_to_goal_translation_min;
		}

		if((robot_abs_x > dist_speed_down) && (robot_abs_x < std::abs(pose_x_init_)))                 // linear_low_speed(0.05) => linear_max_speed(0.2)
		{
			double max_linear_speed = params_ptr->go_to_goal_translation_max;
			translate_velocity = std::min(params_ptr->go_to_goal_translation_min +
			                              (std::abs(pose_x_init_) - robot_abs_x) / dist_speed_down_length * dist_speed_down_range, max_linear_speed);
		}

		if ((robot_abs_x <= dist_speed_down) && robot_abs_x >= dist_low_speed)                 // linear_max_speed(0.2) => linear_low_speed(0.05)
		{
			translate_velocity = params_ptr->go_to_goal_translation_max -
			                     (dist_speed_down - robot_abs_x) / dist_speed_down_length * dist_speed_down_range;
		}

		if (robot_abs_x < dist_low_speed)                 // linear_low_speed(0.05)
		{
			translate_velocity = params_ptr->go_to_goal_translation_min;
		}

		// If robot is close enough to goal, move to final stage
		if (dist_to_goal < goal_points_.front().radius || std::abs(current_position.getX()) < std::abs(gp.x)) {
			change_state(current_state_, NavigateStates::GOAL_ANGLE, clock_->now().seconds(), params_ptr->timeout_goal_angle );
			RCLCPP_DEBUG(logger_, " ******** change to state GOAL_ANGLE ******** ");
			servo_vel->linear.x = gp.drive_backwards ? -translate_velocity : translate_velocity;
			RCLCPP_DEBUG(logger_, "linear_x: %.2f", servo_vel->linear.x);
			state = std::string("GO_TO_GOAL_POSITION");
			infos = std::string("GO_TO_GOAL_POSITION converged ==> change state to GOAL_ANGLE");
			// If robot angle has deviated too much from path, reset
		}
		else 
		{
			// only use low speed for test
			// translate_velocity = params_ptr->go_to_goal_translation_min;

			if (gp.drive_backwards) {
				translate_velocity *= -1;
			}

			// double angle_dist = angles::shortest_angular_distance(current_angle, 0);
			if(std::abs(current_position.getX()) < (params_ptr->offset_last_docked_distance + params_ptr->offset_low_speed))
			{
				RCLCPP_DEBUG(logger_, "low speed mode ");
				if (!bluetooth_connected)
				{
					RCLCPP_INFO_THROTTLE(logger_, *clock_, 2000, "bluetooth disconnected, waiting ......");
					RCLCPP_DEBUG(logger_, "bluetooth disconnected, waiting ......");
					state = std::string("GO_TO_GOAL_POSITION");
					infos = std::string("Reason: bluetooth disconnected ==> stop");
					break;
				}

				servo_vel->linear.x = translate_velocity;

				if (std::abs(current_position.getX()) < (params_ptr->offset_last_docked_distance + params_ptr->last_goal_angle_to_x_positive_dis) )
				{
					double ang2 = angles::shortest_angular_distance(current_angle, 0);
					RCLCPP_DEBUG(logger_, "ang2: %.2f", ang2);
					if (ang2 < 0 && std::abs(ang2) > params_ptr->go_to_goal_apply_rotation_angle && current_position.getY() > -params_ptr->last_goal_angle_to_x_positive_y)
					{
						RCLCPP_DEBUG(logger_, "ang2: %.2f, y: %.2f, angle_to_x_positive direction", ang2, current_position.getY());
						servo_vel->linear.x = -0.05;
						ang = ang2;
					}
					else if (ang2 > 0 && std::abs(ang2) > params_ptr->go_to_goal_apply_rotation_angle && current_position.getY() < params_ptr->last_goal_angle_to_x_positive_y)
					{
						RCLCPP_DEBUG(logger_, "ang2: %.2f, y: %.2f, angle_to_x_positive direction", ang2, current_position.getY());
						servo_vel->linear.x = -0.05;
						ang = ang2;
					}
				}
				bound_rotation(ang, params_ptr->go_to_goal_rotation_min, params_ptr->go_to_goal_rotation_max);
				ang = generate_smooth_rotation_speed(last_rotation_speed_, last_rotation_speed_time_, ang, params_ptr, clock_, logger_);
				servo_vel->angular.z = ang;
				// if (last_rotation_speed_ != ang)
				// {
				// 	RCLCPP_DEBUG(logger_, "stop, and only rotation for fix bug of motor_resoponse_delay");
				// 	// servo_vel->linear.x = 0;
				// 	servo_vel->linear.x = translate_velocity;
				// }
				// else
				// {
				// 	servo_vel->linear.x = translate_velocity;
				// }
				last_rotation_speed_ = ang;

				state = std::string("GO_TO_GOAL_POSITION");
				infos = std::string("GO_TO_GOAL_POSITION (low speed mode) ==> keep on moving");

			}
			else
			{
				RCLCPP_DEBUG(logger_, "normal speed mode ");
				RCLCPP_DEBUG(logger_, "diff angle_to_goal: %.2f", ang);
				RCLCPP_DEBUG(logger_, "abs_angle: %.2f", abs_ang);
				RCLCPP_DEBUG(logger_, "thre: %.2f", params_ptr->go_to_goal_apply_rotation_angle);
				if (abs_ang > params_ptr->go_to_goal_apply_rotation_angle) {
					RCLCPP_DEBUG(logger_, "Need adjust direction.");
					bound_rotation(ang, params_ptr->go_to_goal_rotation_min, params_ptr->go_to_goal_rotation_max);
					ang = generate_smooth_rotation_speed(last_rotation_speed_, last_rotation_speed_time_, ang, params_ptr, clock_, logger_);
					servo_vel->angular.z = ang;
					last_rotation_speed_ = ang;

					state = std::string("GO_TO_GOAL_POSITION");
					infos = std::string("GO_TO_GOAL_POSITION (normal speed mode) ==> keep on moving");
				}
				else
				{
					RCLCPP_DEBUG(logger_, "Don't need adjust direction.");
				}
				servo_vel->linear.x = translate_velocity;
			}
			RCLCPP_DEBUG(logger_, "linear_x: %.2f", servo_vel->linear.x);
			RCLCPP_DEBUG(logger_, "angular.z: %.2f", servo_vel->angular.z);

			// current state: GO_TO_GOAL_POSITION
			bool need_check_collision = true;
			double remaining_rotation_time, predict_time = params_ptr->collision_predict_time;
			double x_c2r, yaw_c2r_abs;
			if (params_ptr->collision_check)
			{
				// 如果需要碰撞检查，只有当机器人和充电桩的距离< dock_valid_obstale_x,且朝向充电桩运动时不需要检查是否碰撞
				// 因为马上就要对接上充电桩，机器人必然要和充电桩进行接触
				auto tf_charger_to_robot = charger_pose_map.inverse() * robot_pose_map;
				auto translation_charger_to_robot = tf_charger_to_robot.getOrigin();
				x_c2r = std::abs(translation_charger_to_robot.getX());
				auto orintation_charger_to_robot = tf_charger_to_robot.getRotation();
				yaw_c2r_abs = std::abs(tf2::getYaw(orintation_charger_to_robot));
				RCLCPP_DEBUG(logger_, "x_charge_to_robot: %.2f, dock_valid_obstacle_x: %.2f", x_c2r, params_ptr->dock_valid_obstacle_x);
				RCLCPP_DEBUG(logger_, "yaw_charger_to_robot: %.2f, throttle: %.2f", yaw_c2r_abs, M_PI * 0.5);
				if ((yaw_c2r_abs < M_PI * 0.5) && (x_c2r < params_ptr->dock_valid_obstacle_x))
				{
					need_check_collision = false;
				}
				else
				{
					remaining_rotation_time = std::abs((x_c2r - params_ptr->dock_valid_obstacle_x) / servo_vel->linear.x);
					predict_time = std::min(double(params_ptr->collision_predict_time), remaining_rotation_time);
					RCLCPP_DEBUG(logger_, "predict_time: %.2f", predict_time);
				}
				RCLCPP_DEBUG(logger_, "need_check_collision: %s", need_check_collision?"true":"false");
			}

			if (params_ptr->collision_check && need_check_collision)
			{
				double cost_value = get_cost_value(logger_,collision_checker, robot_pose_map, footprint_vec, false, servo_vel->linear.x, 0.0,
				                                   predict_time, params_ptr->cmd_vel_hz, params_ptr->odom_twist_scale);
				if (cost_value >= nav2_costmap_2d::LETHAL_OBSTACLE)
				{

					RCLCPP_DEBUG(logger_, "cost value: %.2f >= %.2f", cost_value,  static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));
					servo_vel->linear.x = 0.0;
					RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "stop for collision check, when  %s", magic_enum::enum_name(current_state_).data());

					if(params_ptr->enable_clear_local_costmap)
					{
						clear_local_costmap(client_clear_entire_local_costmap);
					}

					return servo_vel;
				}
			}
			else
			{
				RCLCPP_DEBUG(logger_, "collision_check: %s", params_ptr->collision_check ? "true":"false");
				RCLCPP_DEBUG(logger_, "need_check_collision: %s", need_check_collision ? "true":"false");
				RCLCPP_DEBUG(logger_, "yaw_c2r_abs: %.2f", yaw_c2r_abs);
				RCLCPP_DEBUG(logger_, "x_c2r: %.2f, dock_valid_obstacle_x: %.2f", x_c2r, params_ptr->dock_valid_obstacle_x);

			}

		}
		break;
	}
	case NavigateStates::GOAL_ANGLE:
	{
		RCLCPP_DEBUG(logger_, "***********************************");
		RCLCPP_DEBUG(logger_, "***********************************");
		print_current_state_debug(current_state_);
		
		servo_vel = geometry_msgs::msg::Twist();

		b_timeout_current_state = check_current_state_timeout();
		if (b_timeout_current_state)
		{
			return servo_vel;		
		}

		const GoalPoint & gp = goal_points_.front();
		RCLCPP_DEBUG(logger_, "goal =>  x: %.2f, y: %.2f, yaw: %.2f",
		             gp.x, gp.y, gp.theta);
		RCLCPP_DEBUG(logger_, "robot =>  x: %.2f, y: %.2f, yaw: %.2f",
		             current_pose.getOrigin().getX(), current_pose.getOrigin().getY(),
		             current_angle);
		double ang = angles::shortest_angular_distance(current_angle, gp.theta);
		bound_rotation(ang, params_ptr->go_to_goal_rotation_min, params_ptr->go_to_goal_rotation_max);
		RCLCPP_DEBUG(logger_, "diff angle: %.2f", ang);
		

		double translate_velocity = params_ptr->go_to_goal_translation_max;

		if (gp.drive_backwards)
		{
			translate_velocity *= -1.0;
		}
		// servo_vel->linear.x = translate_velocity;

		if (std::abs(ang) > params_ptr->goal_angle_converged) {
			servo_vel->angular.z = ang;
		} else {
		}
		goal_points_.pop_front();
		RCLCPP_DEBUG(logger_, "============ pop goal============");
		if (goal_points_.size() > 0) {
			change_state(current_state_, NavigateStates::GO_TO_GOAL_POSITION, clock_->now().seconds(), params_ptr->timeout_go_to_goal_position );
			RCLCPP_DEBUG(logger_, "******** change to state GO_TO_GOAL_POSITION ******** ");
		}
		RCLCPP_DEBUG(logger_, " linear_x: %.2f", servo_vel->linear.x);
		RCLCPP_DEBUG(logger_, "angular.z: %.2f", servo_vel->angular.z);
		state = std::string("GOAL_ANGLE");
		infos = std::string("GOAL_ANGLE  ==> keep on rotating");
		break;
	}
	case NavigateStates::UNDOCK:
	{
		print_current_state_debug(current_state_);
		servo_vel = geometry_msgs::msg::Twist();

		b_timeout_current_state = check_current_state_timeout();
		if (b_timeout_current_state)
		{
			undocking = false;
			return servo_vel;		
		}

		update_time_smart();
		undock_dis_moved_ += odom_msg.twist.twist.linear.x * delta_time_;
		RCLCPP_INFO_THROTTLE(logger_, *clock_, 500, "undock cost time: %.2f, dis_moved: %.2f", delta_time_, undock_dis_moved_);
		if (undock_dis_moved_ < params_ptr->undock_dis)
		{
			servo_vel->linear.x = params_ptr->undock_speed;

			double predict_time = std::min((current_state_timeout_ - delta_time_), double(params_ptr->collision_predict_time));
			if (params_ptr->collision_check)
			{
				double cost_value = get_cost_value(logger_,collision_checker, robot_pose_map, footprint_vec, false, servo_vel->linear.x, 0.0,
				                                   predict_time, params_ptr->cmd_vel_hz, params_ptr->odom_twist_scale);
				if (cost_value >= nav2_costmap_2d::LETHAL_OBSTACLE)
				{
					RCLCPP_DEBUG(logger_, "cost value: %.2f >= %.2f", cost_value,  static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));
					servo_vel->linear.x = 0.0;
					RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "stop for collision check, when %s", magic_enum::enum_name(current_state_).data());

					if(params_ptr->enable_clear_local_costmap)
					{
						clear_local_costmap(client_clear_entire_local_costmap);
					}

					return servo_vel;
				}
			}
		}
		else
		{
			goal_points_.clear();
			RCLCPP_INFO(logger_, "undock succeed.");
			undocking = false;
		}

		break;
	}
	} // end of switch
	time_end = std::chrono::high_resolution_clock::now();
	time_cost = std::chrono::duration_cast<std::chrono::milliseconds>(time_end - time_start).count();
	RCLCPP_DEBUG(logger_, "cost %d ms.", (int)time_cost);
	return servo_vel;
}


private:
enum class NavigateStates
{
	INIT,
	LOOKUP_MARKER,
	ANGLE_TO_BUFFER_POINT,
	MOVE_TO_BUFFER_POINT,
	ANGLE_TO_X_POSITIVE_ORIENTATION,
	ANGLE_TO_GOAL,
	GO_TO_GOAL_POSITION,
	GOAL_ANGLE,
	UNDOCK
};

/// @brief 更新当前state
/// @param current_state 当前state
/// @param target_state  目标state
/// @return 返回为空
void change_state(NavigateStates& current_state, NavigateStates target_state, const double& current_state_start_time, const double& current_state_timeout)
{
	RCLCPP_INFO(logger_, "change current state from %s to %s", magic_enum::enum_name(current_state).data(), magic_enum::enum_name(target_state).data());
	current_state = target_state;
	current_state_start_time_ = current_state_start_time;
	current_state_timeout_ = current_state_timeout;
}

bool check_current_state_timeout()
{
	bool ret = false;
	now_time_ = clock_->now().seconds();
	if (now_time_ - current_state_start_time_ > current_state_timeout_)
	{
		ret = true;
	}
	return ret;
}

struct GoalPoint
{
	double x;
	double y;
	double theta;
	float radius;
	bool drive_backwards;
};

// 机器人的位姿信息
struct RobotPose
{
	double x;
	double y;
	double theta;
};

// 机器人的速度信息
struct CmdVel
{
	double linear_x;
	double angular_z;
};

// 机器人的信息，包括map和充电桩下的位姿，速度，当前对接阶段及该阶段开始的时间、最大可持续时间
struct RobotInfo
{
	RobotPose robot_map;
	bool marker_visible;
	RobotPose robot_marker;
	CmdVel cmd_vel;
	double start_time;
	double time_out;
};

void bound_rotation(double & rotation_velocity, float min, float max)
{
	double abs_rot = std::abs(rotation_velocity);
	if (abs_rot > max) {
		rotation_velocity = std::copysign(max, rotation_velocity);
	} else if (abs_rot < min) {
		// min speed if desire small non zero velocity
		rotation_velocity = std::copysign(min, rotation_velocity);
	}
}

double get_cost_value(rclcpp::Logger logger_, nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>  collision_checker,
                      tf2::Transform tf_robot,std::vector<geometry_msgs::msg::Point> footprint, bool rotation,
                      double linear, double angular, double predict_time, int hz, double scale)
{
	(void) logger_;
	double cost_value = 0.0;
	double x,y,theta;
	tf2::Transform tf_offset;
	tf2::Transform tf_new;
	tf_offset.setIdentity();
	int counts_number = std::floor(predict_time * hz);

	if (rotation)
	{
		double vel_angular = angular * scale;
		tf2::Quaternion q;
		for (int i = 0; i < counts_number; i++)
		{
			tf_offset.setOrigin(tf2::Vector3(0,0,0));
			double yaw = 1.0 / hz * vel_angular * i;
			q.setRPY(0, 0, yaw);
			tf_offset.setRotation(q);
			tf_new = tf_robot * tf_offset;
			x = tf_new.getOrigin().getX();
			y = tf_new.getOrigin().getY();
			theta = tf2::getYaw(tf_new.getRotation());
			// RCLCPP_DEBUG(logger_, "x: %.2f, y: %.2f, theta: %.2f", x, y, theta);
			// RCLCPP_DEBUG(logger_,"base footprint");
			// RCLCPP_DEBUG(logger_, "Point(%.2f, %.2f)", footprint[0].x, footprint[0].y);
			// RCLCPP_DEBUG(logger_, "Point(%.2f, %.2f)", footprint[1].x, footprint[1].y);
			// RCLCPP_DEBUG(logger_, "Point(%.2f, %.2f)", footprint[2].x, footprint[2].y);
			// RCLCPP_DEBUG(logger_, "Point(%.2f, %.2f)", footprint[3].x, footprint[3].y);
			double cost_value_tmp = collision_checker.footprintCostAtPose(x, y, theta, footprint);
			// RCLCPP_DEBUG(logger_, "predict number %d cost_value: %.2f", i, cost_value_tmp);
			cost_value = std::max(cost_value, cost_value_tmp);
			if (cost_value >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE))
			{
				return cost_value;
			}
		}
	}
	else
	{
		double vel_linear = linear * scale;

		for (int i = 0; i < counts_number; i++)
		{
			tf_offset.setOrigin(tf2::Vector3(1.0 / hz * vel_linear * i, 0.0, 0.0));
			tf_new = tf_robot * tf_offset;
			x = tf_new.getOrigin().getX();
			y = tf_new.getOrigin().getY();
			theta = tf2::getYaw(tf_new.getRotation());
			// RCLCPP_DEBUG(logger_, "x: %.2f, y: %.2f, theta: %.2f", x, y, theta);
			// RCLCPP_DEBUG(logger_,"base footprint");
			// RCLCPP_DEBUG(logger_, "Point(%.2f, %.2f)", footprint[0].x, footprint[0].y);
			// RCLCPP_DEBUG(logger_, "Point(%.2f, %.2f)", footprint[1].x, footprint[1].y);
			// RCLCPP_DEBUG(logger_, "Point(%.2f, %.2f)", footprint[2].x, footprint[2].y);
			// RCLCPP_DEBUG(logger_, "Point(%.2f, %.2f)", footprint[3].x, footprint[3].y);
			double cost_value_tmp = collision_checker.footprintCostAtPose(x, y, theta, footprint);
			// RCLCPP_DEBUG(logger_, "predict number %d cost_value: %.2f", i, cost_value_tmp);
			cost_value = std::max(cost_value, cost_value_tmp);
			if (cost_value >= static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE))
			{
				return cost_value;
			}
		}
		cost_value = vel_linear;
	}

	return cost_value;
}

float degree_to_radian(float degree)
{
	return degree / 180.0 * M_PI;
}

float radian_to_degree(float theta)
{
	return theta / M_PI * 180.0;
}

// angular unit: radian
float camera_horizontal_view_y_coord(float theta_robot_to_goal, float camera_horizontal_view,  float camera_baselink_dis, float goal_dis_x)
{
	// all parameters are positive values;
	// first, theta_robot_to_goal shall < camera_horizontal_view * 0.5
	// consider one of the situations, robot is in back left(the other situation back right) of the marker
	// when reach the goal, robot pose (-goal_dis_x, 0),
	// camera pose (-(goal_dis_x - camera_baselink_dis * cos(theta_robot_to_goal)), -camera_baselink_dis * sin(theta_robot_to_goal))
	// using cr representing the radius from origin to camera, cx,cy representing camera's x,y coord
	// if camera can see all of  the marker, should satisfied the following condition
	// result : camera_horizontal_view on y's axis: cr * sin(camera_horizontal_view * 0.5 - theta_robot_to_goal) - std::abs(cy)
	// if result is negative, camera only can see y's negative axis, don't satisfied
	// if result is positive, camera can see y's positive axis
	// in this case, if result > marker_size * 0.5, satisfied

	float cx, cy, cr;
	cy = -camera_baselink_dis * std::sin(theta_robot_to_goal);
	cx = -(goal_dis_x - camera_baselink_dis * std::cos(theta_robot_to_goal));
	cr = std::hypot(cx, cy);
	float y_ = cr * std::sin(camera_horizontal_view * 0.5 - theta_robot_to_goal);
	if (y_ > std::abs(cy))
	{
		return y_ - std::abs(cy);
	}
	else
	{
		return -(std::abs(cy) - y_);
	}
}

// smooth rotation speed
float generate_smooth_rotation_speed(const float & last_rotation, double & last_rotation_time, float cur_rotation, motion_control_params* params_ptr, rclcpp::Clock::SharedPtr clock_, rclcpp::Logger logger_)
{
	float new_rotation_speed, rotation_max_change_abs, rotation_cur_change_abs;
	float acc = params_ptr->speed_rotation_acceleration;
	double cur_time = clock_->now().seconds();
	double delta_time = cur_time - last_rotation_time;
	if (delta_time > 0.13) // 10hz
	{
		first_pub_rotation_speed = true;
	}
	else
	{
		first_pub_rotation_speed = false;
	}

	if (first_pub_rotation_speed)
	{
		RCLCPP_DEBUG(logger_, "first pub rotation speed.");
		new_rotation_speed = std::copysign(params_ptr->speed_rotation_init_abs, cur_rotation);
	}
	else
	{
		rotation_max_change_abs = std::abs(delta_time * acc);
		rotation_cur_change_abs = std::abs(cur_rotation - last_rotation);
		if (rotation_cur_change_abs > rotation_max_change_abs)
		{
			new_rotation_speed = last_rotation + std::copysign(rotation_max_change_abs, cur_rotation - last_rotation);
		}
		else
		{
			new_rotation_speed = cur_rotation;
		}
		RCLCPP_DEBUG(logger_, "last_rotation          : %.2f", last_rotation);
		RCLCPP_DEBUG(logger_, "cur_rotation           : %.2f", cur_rotation);
		RCLCPP_DEBUG(logger_, "delta_time             : %.2f", delta_time);
		RCLCPP_DEBUG(logger_, "rotation_max_change_abs: %.2f", rotation_max_change_abs);
		RCLCPP_DEBUG(logger_, "rotation_cur_change_abs: %.2f", rotation_cur_change_abs);
		RCLCPP_DEBUG(logger_, "new_rotation_speed     : %.2f", new_rotation_speed);
	}

	// last_rotation = new_rotation_speed;
	last_rotation_time = cur_time;

	return new_rotation_speed;
}

double diff_angle(const GoalPoint & goal_pt, const tf2::Vector3 & cur_position, double cur_angle, rclcpp::Logger logger_)
{

	double y = goal_pt.y - cur_position.getY();
	double x = goal_pt.x - cur_position.getX();
	double atan2_value = std::atan2(y, x);

	double result = angles::shortest_angular_distance(cur_angle, atan2_value);

	// RCLCPP_DEBUG(logger_, "------caculate diff-------");
	// RCLCPP_DEBUG(logger_, "gp.x: %.2f, gp.y: %.2f, cur.x: %.2f, cur.y: %.2f",
	//              goal_pt.x, goal_pt.y, cur_position.getX(), cur_position.getY());
	// RCLCPP_DEBUG(logger_, "y       => %.2f", y);
	// RCLCPP_DEBUG(logger_, "x       => %.2f", x);
	// RCLCPP_DEBUG(logger_, "atan2   => %.2f", atan2_value);
	// RCLCPP_DEBUG(logger_, "cur_ang => %.2f", cur_angle);
	RCLCPP_DEBUG(logger_, "dist    => %.2f", result);

	return result;
}

bool clear_local_costmap(rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_clear_entire_local_costmap)
{
	bool ret = false;
	clear_time_now = clock_->now().seconds();
	clear_time_delta = clear_time_now - clear_time_last;

	RCLCPP_DEBUG(logger_, "/local_costmap/clear_entirely_local_costmap time_now  : %.2f", clear_time_now);
	RCLCPP_DEBUG(logger_, "/local_costmap/clear_entirely_local_costmap time_last : %.2f", clear_time_last);
	RCLCPP_DEBUG(logger_, "/local_costmap/clear_entirely_local_costmap time_delta: %.2f", clear_time_delta);

	if (clear_time_delta > params_ptr->timout_clear_local_costmap)
	{
		auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();

		auto ret = client_clear_entire_local_costmap->wait_for_service(0.05s);
		if (!ret)
		{
			RCLCPP_INFO(logger_, "/local_costmap/clear_entirely_local_costmap service not online.");
		}
		else
		{
			clear_time_last = clear_time_now;
			RCLCPP_INFO(logger_, "call service: /local_costmap/clear_entirely_local_costmap. (delta_time: %.2f, timout: %.2f)", clear_time_delta, params_ptr->timout_clear_local_costmap);
			client_clear_entire_local_costmap->async_send_request(request);
			ret = true; // 这里为简化处理，只要发送了/local_costmap/clear_entirely_local_costmap的服务请求,就返回true
		}
	}
	else
	{
	}
	return ret;
}

// 用于处理未知意外导致的两帧时间间隔与帧率严重不符的情况
// 更新now_time_, pre_time_, delta_time_
void update_time_smart()
{
	now_time_ = clock_->now().seconds();
	delta_time_ = now_time_ - pre_time_;
	pre_time_ = now_time_;
	if (abs(delta_time_) > 1.5 / params_ptr->cmd_vel_hz)
	{
		RCLCPP_WARN(logger_, "error occurs, real delta_time: %.2f, hz is %d", delta_time_, params_ptr->cmd_vel_hz);
		delta_time_ = 1.0 / params_ptr->cmd_vel_hz;
	}
}

void print_current_state_debug(const NavigateStates& state)
{
	RCLCPP_DEBUG(logger_, "--------------- %s ---------------", magic_enum::enum_name(state).data());
}

// 保存充电桩在map下的位姿信息和机器人在map和充电桩下的位姿信息
void save_all_poses_infos(tf2::Transform tf_robot_map, tf2::Transform tf_robot_charger, tf2::Transform tf_charger_map, bool sees_dock)
{
	marker_visible_ = sees_dock;
	tf_robot_map_ = tf_robot_map;
	tf_robot_charger_ = tf_robot_charger;
	tf_charger_map_ = tf_charger_map;

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

bool hazards_valid(const tf2::Transform & current_pose, capella_ros_dock_msgs::msg::HazardDetectionVector hazards)
{
	bool ret = false;
	auto distance = std::abs(current_pose.getOrigin().getX());
	auto detections = hazards.detections;
	for (int i = 0; i < (int)(detections.size()); i++)
	{
		auto hazard = detections[i];
		using HazardDetection = capella_ros_dock_msgs::msg::HazardDetection;
		switch(hazard.type)
		{
		case HazardDetection::BACKUP_LIMIT:
		case HazardDetection::BUMP:
		case HazardDetection::CLIFF:
		case HazardDetection::STALL:
		case HazardDetection::WHEEL_DROP:
		case HazardDetection::OBJECT_PROXIMITY:
		{
			ret = true;
			break;
		}
		}
	}
	ret = ret && distance > params_ptr->dock_valid_obstacle_x;
	return ret;
}

// SimpleGoalController成员变量
rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node_;
rclcpp::Logger logger_;
rclcpp::Clock::SharedPtr clock_;

motion_control_params *params_ptr;

std::mutex mutex_;
std::deque<GoalPoint> goal_points_;
NavigateStates current_state_;

std::chrono::high_resolution_clock::time_point time_start;
std::chrono::high_resolution_clock::time_point time_end;
int64_t time_cost;

double dist_buffer_point;
double dist_buffer_point_yaw;

double waiting_for_best_coord_start_time;
bool start_time_recorded = false;
double thre_angle_diff = 0.30; // 0.4461565280195475968735605160853 <= tan(32-arctan2(0.12/(0.32+0.1+0.5))

// buffer_goal_point
double buffer_goal_point_x; // docked,low_vel_dist,first_goal_dist, buffer_goal_dist
double buffer_goal_point_y = 0.0;
double robot_angle_to_buffer_point_yaw;
double robot_current_yaw;
double robot_current_yaw_positive;
bool drive_back = false;
double theta_positive, theta_negative;

rclcpp::Time last_time_cannot_see_dock;
rclcpp::Time now_time_cannot_see_dock;
float time_sleep;
float last_rotation_speed_ = 0.0f;
double last_rotation_speed_time_ = 0.0f;
bool first_pub_rotation_speed = true;
bool need_get_outof_charger_range = false;
bool get_out_of_charger_range_completed = true;

// when has contacted, keep moving a little time
bool first_contacted = true;
double first_contacted_time;

bool first_cannot_see_dock = true;

// impl for undock
bool undocking = false;
double undock_dis_moved_ = 0.0;

bool pose_x_init_recoreded_{false};
double pose_x_init_;

// angle_to_buffer_point and move_to_buffer_point 参数
double theta_angle_to_buffer_point;
double dist_move_to_buffer_point;
tf2::Transform tf_before_angle_to_buffer_point;
tf2::Transform tf_after_angle_to_buffer_point;
tf2::Transform tf_after_move_to_buffer_point;

double clear_time_last = 0.0;
double clear_time_now, clear_time_delta;

double goal_dist_move_;
double goal_dist_rotate_;

// 时间参数
double now_time_;
double pre_time_;
double delta_time_;
double current_state_start_time_;
double current_state_timeout_;

// 保存机器人的信息
RobotInfo robot_info_;
// map下的机器人位姿
tf2::Transform tf_robot_map_;
double robot_x_map_;
double robot_y_map_;
double robot_yaw_map_;
// 充电桩码坐标系下的机器人位姿
tf2::Transform tf_robot_charger_;
double robot_x_charger_;
double robot_y_charger_;
double robot_yaw_charger_;
// map下充电桩的位姿
tf2::Transform tf_charger_map_;
double charger_x_map_;
double charger_y_map_;
double charger_yaw_map_;
// 是否可以识别出充电桩码坐标
bool marker_visible_{false};
};

}  // namespace capella_ros_dock
#endif   // CAPELLA_ROS_DOCK__SIMPLE_GOAL_CONTROLLER_HPP_
