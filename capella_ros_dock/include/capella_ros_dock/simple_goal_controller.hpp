

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



using namespace std;

namespace capella_ros_dock
{


/**
 * @brief This class provides an API to give velocity commands given a goal and robot position.
 */
class SimpleGoalController
{
public:
SimpleGoalController(motion_control_params *params_ptr)
{
	this->params_ptr = params_ptr;
	std::cout << "dock_valid_obstacle_x: " << params_ptr->dock_valid_obstacle_x << std::endl;
	init_params(params_ptr);
}

void init_params(motion_control_params* params_ptr)
{
	buffer_goal_point_x = -(params_ptr->last_docked_distance_offset_
	                        + params_ptr->distance_low_speed
	                        + params_ptr->second_goal_distance
	                        + params_ptr->buffer_goal_distance);
	buffer_goal_point_y = 0.0 + params_ptr->goal_y_correction;

	// for mk5,not accurate
	// 0.4461565280195475968735605160853 <= tan(32-arctan2(0.12/(0.32+0.1+0.5))
	// thre_angle_diff = std::tan(params_ptr->camera_horizontal_view  * 0.5 / 180.0 * M_PI
	//                            - std::atan(0.5 * params_ptr->marker_size / (params_ptr->last_docked_distance_offset_ + params_ptr->distance_low_speed + params_ptr->second_goal_distance))
	//                            - params_ptr->angle_delta
	//                            );
	// RCLCPP_INFO(rclcpp::get_logger("simple_goal_controller"), "thre_angle_diff: %f", thre_angle_diff);

	float camera_horizontal_view, marker_size, camera_baselink_dis, goal_dis_x;
		camera_horizontal_view = degree_to_radian(params_ptr->camera_horizontal_view);
		marker_size = params_ptr->marker_size;
		camera_baselink_dis = params_ptr->camera_baselink_dis; //0.4 -0.1(gp.r)
		goal_dis_x = params_ptr->last_docked_distance_offset_ + params_ptr->distance_low_speed + params_ptr->second_goal_distance;
		RCLCPP_INFO(rclcpp::get_logger("simple_goal_controller"), "camera_horizontal_view: %f, marker_size: %f, camera_baselink_dis: %f, goal_dis_x: %f",
			camera_horizontal_view, marker_size, camera_baselink_dis, goal_dis_x);

		float d1, d2, d3, alpha;
		d1 = goal_dis_x;
		d2 = camera_baselink_dis;
		d3 = marker_size * 0.5;
		alpha = camera_horizontal_view * 0.5;
		float tan_alpha = std::tan(alpha);
		RCLCPP_INFO(rclcpp::get_logger("simple_goal_controller"), "d1: %f, d2: %f, d3: %f, alpha: %f", d1, d2, d3, alpha);
		float r = std::hypot(d1 * tan_alpha - d3, d1 + d3 * tan_alpha);
		float x1 = d2 * tan_alpha;
		float x2 = d1 * tan_alpha - d3;
		float beta_plus_theta = std::acos(x1 / r);
		float beta = std::acos(x2 / r);
		thre_angle_diff = beta_plus_theta - beta;
		RCLCPP_INFO(rclcpp::get_logger("simple_goal_controller"), "r: %f, x1: %f, x2: %f, beta_plus_theta: %f, beta: %f", r, x1, x2, beta_plus_theta, beta);
		RCLCPP_INFO(rclcpp::get_logger("simple_goal_controller"), "thre_angle_diff: %F", thre_angle_diff);

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
	// 			RCLCPP_INFO(logger_, "x: %f, y: %f, theta: %f, theta_to_goal: %f", x, y, theta, theta_to_goal);
	// 			RCLCPP_INFO(logger_, "camera_horizontal_view/2: %f< theta_to_goal: %f",
	// 				camera_horizontal_view * 0.5, std::abs(theta_to_goal));
	// 		}
	// 		else
	// 		{
	// 			float y_coord = camera_horizontal_view_y_coord(std::abs(theta_to_goal), camera_horizontal_view, camera_baselink_dis, goal_dis_x);
	// 			if (y_coord > marker_size * 0.5)
	// 			{
	// 				RCLCPP_INFO(logger_, "============success============");
	// 				RCLCPP_INFO(logger_, "x: %f, y: %f, theta: %f, theta_to_goal: %f", x, y, theta, theta_to_goal);
	// 				RCLCPP_INFO(logger_, "y_coord: %f", y_coord);
	// 			}
	// 			else
	// 			{
	// 				RCLCPP_INFO(logger_, "************failed************");
	// 				RCLCPP_INFO(logger_, "x: %f, y: %f, theta: %f, theta_to_goal: %f", x, y, theta, theta_to_goal);
	// 				RCLCPP_INFO(logger_, "y_coord: %f", y_coord);
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
	navigate_state_ = NavigateStates::LOOKUP_ARUCO_MARKER;
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
	nav_msgs::msg::Odometry odom_msg, rclcpp::Clock::SharedPtr clock_, rclcpp::Logger logger_, motion_control_params* params_ptr, capella_ros_dock_msgs::msg::HazardDetectionVector hazards, std::string & state, std::string & infos,
	nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*>  collision_checker, std::vector<geometry_msgs::msg::Point> footprint_vec, rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_clear_entire_local_costmap)
{	
	// impl undock (go to undock state)
	if (goal_points_.size() >0 && !(goal_points_.front().drive_backwards))
	{
		RCLCPP_INFO(logger_, "***************** start undock *****************");
		navigate_state_ = NavigateStates::UNDOCK;
		sleep(0.5); // wait for /charger/stop to execute.
		undocking = true;
	}
	else
	{
		undocking = false;
		start_undock = true;
	}
	
	// RCLCPP_INFO_STREAM(logger_, "simple_goal_controller => max_dock_action_run_time: " << params_ptr->max_dock_action_run_time << " seconds.");
	time_start = std::chrono::high_resolution_clock::now();
	BehaviorsScheduler::optional_output_t servo_vel;
	const std::lock_guard<std::mutex> lock(mutex_);
	if (is_docked && !undocking)
	{
		if(first_contacted)
		{
			first_contacted = false;
			first_contacted_time = clock_->now().seconds();
			RCLCPP_DEBUG(logger_, "keep moving until %f expired.", params_ptr->contacted_keep_move_time);
		}
		else
		{
			auto now_time = clock_->now().seconds();
			if ((now_time - first_contacted_time) > params_ptr->contacted_keep_move_time)
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
	// if (navigate_state_ >= NavigateStates::ANGLE_TO_GOAL)
	// {
		current_angle = tf2::getYaw(current_pose.getRotation());
		current_position = current_pose.getOrigin();
	// }

	// stop when has valid hazards
	if(hazards_valid(current_pose, hazards) && navigate_state_ > NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION)
	{
		last_time_cannot_see_dock = clock_->now();
		RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "stop for hazards.");
		auto distance = std::abs(current_pose.getOrigin().getX());
		RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "distance: %f", distance);
		RCLCPP_INFO_THROTTLE(logger_, *clock_, 1000, "throttle: %f", params_ptr->dock_valid_obstacle_x);
		servo_vel = geometry_msgs::msg::Twist();
		state = std::string(" > ANGLE_TO_X_POSITIVE_ORIENTATION");
		infos = std::string("Reason: have valid hazards and navigate_state > ANGLE_TO_X_POSITIVE_ORIENTATION  ==> stop ...");
		return servo_vel;
	}
	if (sees_dock)
	{
		first_cannot_see_dock = true;
	}

	
	if ((navigate_state_ > NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION) && need_get_outof_charger_range && (clock_->now().seconds() - last_time_cannot_see_dock.seconds()) < (params_ptr->time_sleep + 2))
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
		navigate_state_ = NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION;
		servo_vel = geometry_msgs::msg::Twist();
		RCLCPP_INFO(logger_, "get_outof_charge_range completed");
		state = std::string(" get_outof_charge_range completed");
		infos = std::string("Reason: get_outof_charger_range completed, go to state ANGLE_TO_X_POSITIVE_ORIENTATION");
		return servo_vel;
	}

	if(!sees_dock && navigate_state_ > NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION)
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
				navigate_state_ = NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION;
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
	switch (navigate_state_) {
	case NavigateStates::LOOKUP_ARUCO_MARKER:
	{
		RCLCPP_DEBUG(logger_, "------------- LOOKUP_ARUCO_MARKER -------------");
		
		servo_vel = geometry_msgs::msg::Twist();

		if (using_localization)
		{
			auto angle_robot = tf2::getYaw(robot_pose_map.getRotation());
			double x_charger, y_charger, x_robot, y_robot;
			x_robot = robot_pose_map.getOrigin()[0];
			y_robot = robot_pose_map.getOrigin()[1];
			x_charger = charger_pose_map.getOrigin()[0];
			y_charger = charger_pose_map.getOrigin()[1];
			auto angle_charger_to_robot = std::atan2(y_robot - y_charger, x_robot - x_charger);
			auto dist_angle = angles::shortest_angular_distance(angle_robot, angle_charger_to_robot);
			RCLCPP_DEBUG(logger_, "using localization: %s", using_localization?"true":"false");
			double angular_z_ = std::copysign(params_ptr->max_rotation, dist_angle);

			if (!sees_dock)
			{
				RCLCPP_DEBUG(logger_, "x_robot_map: %f, y_robot_map: %f", x_robot, y_robot);
				RCLCPP_DEBUG(logger_, "x_charger_map: %f, y_charger_map: %f", x_charger, y_charger);

				RCLCPP_DEBUG(logger_, "angle_robot: %f", angle_robot);
				RCLCPP_DEBUG(logger_, "angle_charger_to_robot: %f", angle_charger_to_robot);
				RCLCPP_DEBUG(logger_, "dist_angle: %f", dist_angle);

				start_time_recorded = false;
				RCLCPP_DEBUG(logger_, "Need rotate robot for it can see the marker.");				

				servo_vel->angular.z = angular_z_;
				RCLCPP_DEBUG(logger_, "angular_z: %f", servo_vel->angular.z);

				// before actually begin rotation, collision_check first
				double cost_value = get_cost_value(logger_,collision_checker, robot_pose_map, footprint_vec, true, 0.0, servo_vel->angular.z,
					params_ptr->collision_predict_time, params_ptr->cmd_vel_hz, 0.8);
				if ((cost_value == static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE)) && params_ptr->rotation_collision_check)
				{
					RCLCPP_DEBUG(logger_, "cost value: %f == %f", cost_value,  static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE));	
					servo_vel->angular.z = 0.0;

					double clear_time_now, clear_time_delta;
					clear_time_now = clock_->now().seconds();
					clear_time_delta = clear_time_now - time_last_local_costmap_clear;
					if (clear_time_delta > params_ptr->time_local_costmap_clear_min)
					{
						auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
						RCLCPP_DEBUG(logger_, "clear_time_now: %f", clear_time_now);
						RCLCPP_DEBUG(logger_, "clear_time_last: %f", time_last_local_costmap_clear);
						RCLCPP_DEBUG(logger_, "clear_time_delta: %f", clear_time_delta);
						RCLCPP_INFO(logger_, "call service for clear local_costmap.");
						time_last_local_costmap_clear = clear_time_now;
						client_clear_entire_local_costmap->async_send_request(request);
					}
					else
					{
						time_last_local_costmap_clear = clear_time_now;
					}
					return servo_vel;			
				}
				else
				{
					RCLCPP_DEBUG(logger_, "cost value: %f, go on ......", cost_value);
					RCLCPP_DEBUG(logger_, "rotation_collision_check: %s", params_ptr->rotation_collision_check ? "true":"false");
				}



				// double x, y, theta;
				// x = robot_pose_map.getOrigin().getX();
				// y = robot_pose_map.getOrigin().getY();
				// theta = tf2::getYaw(robot_pose_map.getRotation());
				// // double cost = collision_checker.footprintCostAtPose(x, y, theta, footprint_vec);
				// double cost = collision_checker.footprintCost(footprint_vec);
				// RCLCPP_DEBUG(logger_, "x: %f, y: %f, theta: %f", x, y, theta);
				// for (size_t index = 0; index < footprint_vec.size(); index++)
				// {
				// 	RCLCPP_DEBUG(logger_, "footprint Point(%f, %f)", footprint_vec[index].x, footprint_vec[index].y);
				// }
				// if ((cost >= static_cast<double>(251)) && params_ptr->rotation_collision_check)
				// {
				// 	RCLCPP_DEBUG(logger_, "cost value: %f >= %f", cost, static_cast<double>(251));	
				// 	servo_vel->angular.z = 0.0;
				// 	return servo_vel;			
				// }
				// else
				// {
				// 	RCLCPP_DEBUG(logger_, "cost value: %f, go on ......", cost);
				// 	RCLCPP_DEBUG(logger_, "rotation_collision_check: %s", params_ptr->rotation_collision_check ? "true":"false");
				// }
				
				state = std::string("LOOKUP_ARUCO_MARKER");
				infos = std::string("Reason: camera's orientation not towards charger's marker ==> rotate robot");
			}
			else
			{
				RCLCPP_DEBUG(logger_, "waiting for find the best coordinate for robot in marker frame");
				double x_,y_, theta_;
				x_ = current_pose.getOrigin()[0];
				y_ = current_pose.getOrigin()[1];
				theta_ = tf2::getYaw(current_pose.getRotation());
				RCLCPP_DEBUG(logger_, "robot_x: %f", x_);
				RCLCPP_DEBUG(logger_, "robot_y: %f", y_);
				RCLCPP_DEBUG(logger_, "robot_theta: %f", theta_);

				if(!start_time_recorded)
				{
					start_time_recorded = true;
					waiting_for_best_coord_start_time = clock_->now().seconds();
					RCLCPP_DEBUG(logger_, "waiting_for_best_coord_start_time : %f", waiting_for_best_coord_start_time);
					state = std::string("LOOKUP_ARUCO_MARKER");
					infos = std::string("Reason: camera's orientation first towards charger's marker ==> record the start time.");
				}
				else
				{
					auto now_time = clock_->now().seconds();

					if (now_time - waiting_for_best_coord_start_time < params_ptr->localization_converged_time)
					{
						RCLCPP_DEBUG(logger_, "coords not converged, just wait.");
						state = std::string("LOOKUP_ARUCO_MARKER");
						infos = std::string("Reason: coords not converged ==> just wait.");
					}
					else
					{
						RCLCPP_DEBUG(logger_, "coords converged, change state.");
						start_time_recorded = false;

						double robot_x = current_pose.getOrigin().getX();
						double robot_y = current_pose.getOrigin().getY();
						float distance_tmp = params_ptr->last_docked_distance_offset_
								+ params_ptr->distance_low_speed
								+ params_ptr->second_goal_distance;
						double theta = std::atan2(std::abs(robot_y), std::abs(robot_x) - distance_tmp);
						double robot_theta = tf2::getYaw(current_pose.getRotation());
						RCLCPP_DEBUG(logger_, "robot_x: %f", robot_x);
						RCLCPP_DEBUG(logger_, "robot_y: %f", robot_y);
						RCLCPP_DEBUG(logger_, "theta_to_last_goal: %f", theta);
						RCLCPP_DEBUG(logger_, "thre_angle_diff: %f", thre_angle_diff);
						RCLCPP_DEBUG(logger_, "robot_theta: %f", robot_theta);

						double base_link_y, base_link_x; 
						base_link_y = robot_y - params_ptr->base_link_charge_dis * std::sin(robot_theta);
						base_link_x = robot_x - params_ptr->base_link_charge_dis * std::cos(robot_theta);

						RCLCPP_DEBUG(logger_, "base_link_x: %f", base_link_x);
						RCLCPP_DEBUG(logger_, "base_link_y: %f", base_link_y);

						if (theta < thre_angle_diff && std::abs(robot_x) > (distance_tmp + params_ptr->deviate_second_goal_x) && std::abs(base_link_y) < params_ptr->base_link_y_thr)                                                                                                                                                                      // 0.7 <= 0.5 + 0.2(x_error)
						{
							RCLCPP_DEBUG(logger_, "robot change state to angle_to_goal");
							navigate_state_ = NavigateStates::ANGLE_TO_GOAL;
							state = std::string("LOOKUP_ARUCO_MARKER");
							infos = std::string("Reason: robot's position converged ==> directly change state to ANGLE_TO_GOAL");
						}
						else
						{
							RCLCPP_DEBUG(logger_, "robot change state to angle_to_buffer_point");
							RCLCPP_DEBUG(logger_, "buffer_goal_point_x: %f, buffer_goal_point_y: %f", buffer_goal_point_x, buffer_goal_point_y);

							tf_before_angle_to_buffer_point = robot_pose_map;
							
							double buffer_goal_point_x_base_link = buffer_goal_point_x - params_ptr->base_link_charge_dis;
							double buffer_goal_point_y_base_link = buffer_goal_point_y;
							
							dist_buffer_point = std::hypot(base_link_x - buffer_goal_point_x_base_link,
										base_link_y - buffer_goal_point_y_base_link);
							
							dist_move_to_buffer_point = dist_buffer_point;
							
							robot_angle_to_buffer_point_yaw = std::atan2(buffer_goal_point_y_base_link - base_link_y,
												buffer_goal_point_x_base_link - base_link_x);

							// decide drive back or not
							robot_current_yaw = robot_theta;
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

							RCLCPP_DEBUG(logger_, "robot_current_yaw: %f", robot_current_yaw);
							RCLCPP_DEBUG(logger_, "robot_angle_to_buffer_point_yaw: %f", robot_angle_to_buffer_point_yaw);
							RCLCPP_DEBUG(logger_, "theta_negative: %f", theta_negative);
							RCLCPP_DEBUG(logger_, "theta_positive: %f", theta_positive);
							RCLCPP_DEBUG(logger_, "dist_buffer_point: %f", dist_buffer_point);
							RCLCPP_DEBUG(logger_, "dist_buffer_point_yaw: %f", dist_buffer_point_yaw);
							pre_time = clock_->now().seconds();
							navigate_state_ = NavigateStates::ANGLE_TO_BUFFER_POINT;
							state = std::string("LOOKUP_ARUCO_MARKER");
							infos = std::string("Reason: robot's position not converged ==> directly change state to ANGLE_TO_BUFFER_POINT");
						}
					}
					
				}
			}
		}
		else // process for the case that don't use robot localization
		{

		}
	
		break;
	}
	case NavigateStates::ANGLE_TO_BUFFER_POINT:
	{
		RCLCPP_DEBUG(logger_, "------------- ANGLE_TO_BUFFER_POINT -------------");
		servo_vel = geometry_msgs::msg::Twist();
		now_time = clock_->now().seconds();
		double dt = now_time - pre_time;

		RCLCPP_DEBUG(logger_, "dt: %f", dt);
		RCLCPP_DEBUG(logger_, "angular.z: %f", odom_msg.twist.twist.angular.z);
		RCLCPP_DEBUG(logger_, "delta_angular: %f", odom_msg.twist.twist.angular.z * dt);
		RCLCPP_DEBUG(logger_, "dist_buffer_point_yaw pre: %f", dist_buffer_point_yaw);
		dist_buffer_point_yaw -= odom_msg.twist.twist.angular.z * dt;
		robot_current_yaw += odom_msg.twist.twist.angular.z * dt;
		RCLCPP_DEBUG(logger_, "dist_buffer_point_yaw now: %f", dist_buffer_point_yaw);
		double angle_dist = dist_buffer_point_yaw;
		RCLCPP_DEBUG(logger_, "angle_dist: %f", angle_dist);
		RCLCPP_DEBUG(logger_, "robot_map_yaw: %f", tf2::getYaw(robot_pose_map.getRotation()));
		pre_time = now_time;
		if(std::abs(angle_dist) < params_ptr->tolerance_angle)
		{
			RCLCPP_DEBUG(logger_, "change state to move_to_buffer_point.");
			tf_after_angle_to_buffer_point = robot_pose_map;
			double theta_delta_map = tf2::getYaw((tf_before_angle_to_buffer_point.inverse() * tf_after_angle_to_buffer_point).getRotation());
			RCLCPP_DEBUG(logger_, "dist_angle_to_buffer_point: %f, theta_delta: %f", theta_angle_to_buffer_point, theta_delta_map);
			navigate_state_ = NavigateStates::MOVE_TO_BUFFER_POINT;
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
			// double cost = collision_checker.footprintCostAtPose(x, y, theta, footprint_vec);
			double cost = collision_checker.footprintCost(footprint_vec);
			RCLCPP_DEBUG(logger_, "x: %f, y: %f, theta: %f", x, y, theta);
			for (size_t index = 0; index < footprint_vec.size(); index++)
			{
				RCLCPP_DEBUG(logger_, "footprint Point(%f, %f)", footprint_vec[index].x, footprint_vec[index].y);
			}
			if ((cost >= static_cast<double>(251)) && params_ptr->rotation_collision_check)
			{
				RCLCPP_DEBUG(logger_, "cost value: %f >= %f", cost, static_cast<double>(251));	
				servo_vel->angular.z = 0.0;
				return servo_vel;			
			}
			else
			{
				RCLCPP_DEBUG(logger_, "cost value: %f, go on ......", cost);
				RCLCPP_DEBUG(logger_, "rotation_collision_check: %s", params_ptr->rotation_collision_check ? "true":"false");
			}
			


			bound_rotation(angle_dist, params_ptr->min_rotation, params_ptr->max_rotation);
			if(std::abs(angle_dist) < params_ptr->min_rotation)                                                                                                                                                                                 // 0.1 => 0.8 => raw_vel output 0
			{
				angle_dist = std::copysign(params_ptr->min_rotation, angle_dist);
			}
			servo_vel->angular.z = angle_dist;
			RCLCPP_DEBUG(logger_, "angular.z: %f", angle_dist);
			state = std::string("ANGLE_TO_BUFFER_POINT");
			infos = std::string("Reason: ANGLE_TO_BUFFER_POINT not converged ==> keep on rotating robot");
		}
		break;
	}
	case NavigateStates::MOVE_TO_BUFFER_POINT:
	{
		RCLCPP_DEBUG(logger_, "------------- MOVE_TO_BUFFER_POINT -------------");
		servo_vel = geometry_msgs::msg::Twist();
		now_time = clock_->now().seconds();
		double dt = now_time - pre_time;
		dist_buffer_point -= dt * std::abs(odom_msg.twist.twist.linear.x);
		pre_time = now_time;
		double dist_y = dist_buffer_point;
		RCLCPP_DEBUG(logger_, "odom_msg.linear_x: %f", odom_msg.twist.twist.linear.x);
		RCLCPP_DEBUG(logger_, "dt: %f", dt);
		RCLCPP_DEBUG(logger_, "dist_buffer_point now: %f", dist_buffer_point);
		RCLCPP_DEBUG(logger_, "dist_y: %f", dist_y);
		RCLCPP_DEBUG(logger_, "robot_map_x: %f, robot_map_y: %f", robot_pose_map.getOrigin().getX(), robot_pose_map.getOrigin().getY());
		if (std::abs(dist_y) < params_ptr->tolerance_r)
		{
			RCLCPP_DEBUG(logger_, "change state to angle_to_x_positive_orientation");
			tf_after_move_to_buffer_point = robot_pose_map;
			tf2::Transform tf_ = tf_after_angle_to_buffer_point.inverse() * tf_after_move_to_buffer_point;
			double dist_move_to_buffer_point_delta = std::hypot(tf_.getOrigin().getX(), tf_.getOrigin().getY());
			RCLCPP_DEBUG(logger_, "dist_move_to_buffer_point: %f, dist_delta: %f", dist_move_to_buffer_point, dist_move_to_buffer_point_delta);
			

			navigate_state_ = NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION;
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
			servo_vel->linear.x = translate_velocity;
			RCLCPP_DEBUG(logger_, "linear.x: : %f", translate_velocity);
			state = std::string("MOVE_TO_BUFFER_POINT");
			infos = std::string("Reason: MOVE_TO_BUFFER_POINT not converged ==> keep on moving");
		}
		break;
	}
	case NavigateStates::ANGLE_TO_X_POSITIVE_ORIENTATION:
	{
		RCLCPP_DEBUG(logger_, "------------- ANGLE_TO_X_POSITIVE_ORIENTATION -------------");
		servo_vel = geometry_msgs::msg::Twist();
		now_time = clock_->now().seconds();
		// double dt = now_time - pre_time;

		double robot_yaw_marker = tf2::getYaw(current_pose.getRotation());
		double dist_yaw_marker = angles::shortest_angular_distance(robot_yaw_marker, 0);

		RCLCPP_DEBUG(logger_, "dist_yaw_marker: %f", dist_yaw_marker);
		if(std::abs(dist_yaw_marker) < params_ptr->tolerance_angle )
		{
			double robot_x = current_pose.getOrigin().getX();
			double robot_y = current_pose.getOrigin().getY();
			float distance_tmp = params_ptr->last_docked_distance_offset_
					+ params_ptr->distance_low_speed
					+ params_ptr->second_goal_distance;
			double theta = std::atan2(std::abs(robot_y), std::abs(robot_x) - distance_tmp);
			double robot_theta = tf2::getYaw(current_pose.getRotation());
			RCLCPP_DEBUG(logger_, "robot_x: %f", robot_x);
			RCLCPP_DEBUG(logger_, "robot_y: %f", robot_y);
			RCLCPP_DEBUG(logger_, "theta_to_second_goal: %f", theta);
			RCLCPP_DEBUG(logger_, "thre_angle_diff: %f", thre_angle_diff);
			RCLCPP_DEBUG(logger_, "robot_theta: %f", robot_theta);

			double base_link_y; 
			base_link_y = robot_y - params_ptr->base_link_charge_dis * std::sin(robot_theta);

			RCLCPP_DEBUG(logger_, "base_link_y: %f", base_link_y);

			if (theta < thre_angle_diff && std::abs(robot_x) > (distance_tmp + params_ptr->deviate_second_goal_x) && std::abs(base_link_y) < params_ptr->base_link_y_thr)                                                                                                                                                                      // 0.7 <= 0.5 + 0.2(x_error)
						 
			{
				RCLCPP_INFO(logger_, "converged ==>Change state to ANGLE_TO_GOAL");
				navigate_state_ = NavigateStates::ANGLE_TO_GOAL;
				state = std::string("ANGLE_TO_X_POSITIVE_ORIENTATION");
				infos = std::string("Reason: ANGLE_TO_X_POSITIVE_ORIENTATION converged ==> change state to ANGLE_TO_GOAL");
			}
			else
			{
				RCLCPP_INFO(logger_, "To re-execute ANGLE_TO_BUFFER_POINT, change state to LOOKUP_ARUCO_MARKER");
				navigate_state_ = NavigateStates::LOOKUP_ARUCO_MARKER;
				state = std::string("ANGLE_TO_X_POSITIVE_ORIENTATION");
				infos = std::string("Reason: ANGLE_TO_X_POSITIVE_ORIENTATION not converged ==> change state to LOOKUP_ARUCO_MARKER");
			}
		}
		else
		{
			RCLCPP_DEBUG(logger_, "executing ANGLE_TO X_POSITIVE_ORIENTATION. dist_yaw_marker: %f", dist_yaw_marker);
			bound_rotation(dist_yaw_marker, params_ptr->min_rotation, params_ptr->max_rotation);
			servo_vel->angular.z = dist_yaw_marker;
			RCLCPP_DEBUG(logger_, "angular.z: %f", servo_vel->angular.z);
			RCLCPP_DEBUG(logger_, "executing ANGLE_TO X_POSITIVE_ORIENTATION. angular_z: %f", servo_vel->angular.z);
		}

		break;
	}
	case NavigateStates::ANGLE_TO_GOAL:
	{
		RCLCPP_DEBUG(logger_, "------------- ANGLE_TO_GOAL -------------");
		const GoalPoint & gp = goal_points_.front();

		RCLCPP_DEBUG(logger_, "goal =>  x: %f, y: %f, yaw: %f",
		             gp.x, gp.y, gp.theta);
		RCLCPP_DEBUG(logger_, "robot =>  x: %f, y: %f, yaw: %f",
		             current_position.getX(), current_position.getY(),
		             current_angle);


		double delta_y, delta_x;
		delta_y = std::abs(gp.y - current_position.getY());
		delta_x = std::abs(gp.x - current_position.getX());
		double dist_to_goal = std::hypot(delta_x, delta_y);
		if (dist_to_goal <= gp.radius || delta_y < params_ptr->dist_error_y_1) {
			servo_vel = geometry_msgs::msg::Twist();
			navigate_state_ = NavigateStates::GO_TO_GOAL_POSITION;
			pose_x_init_recoreded_ = false;
			state = std::string("ANGLE_TO_GOAL");
			infos = std::string("Reason: ANGLE_TO_GOAL converged ==> change state to GO_TO_GOAL_POSITION");
		}
		else
		{
			double ang = diff_angle(gp, current_position, current_angle, logger_);
			double ang_save = ang;
			RCLCPP_DEBUG(logger_, "diff angle: %f", ang);
			bound_rotation(ang, 0.05, 0.10);
			RCLCPP_DEBUG(logger_, "bound angle: %f", ang);
			RCLCPP_DEBUG(logger_, "--------------------------------");
			servo_vel = geometry_msgs::msg::Twist();
			// fix bug when robot had angle to marker but y coord error or odom data error,  10 degree(0.174533)
			if (std::abs(ang_save) < params_ptr->angle_to_goal_angle_converged || (sees_dock && std::abs(std::abs(current_angle) - M_PI) <  0.174533)) {
				navigate_state_ = NavigateStates::GO_TO_GOAL_POSITION;
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
		servo_vel = geometry_msgs::msg::Twist();
		RCLCPP_DEBUG(logger_, "------------- GO_TO_GOAL_POSITION -------------");
		
		GoalPoint gp = goal_points_.front();;
		if (goal_points_.size() > 1)
		{
			RCLCPP_DEBUG(logger_, "not the first goal.");
		}
		else
		{
			RCLCPP_DEBUG(logger_, "the first goal.");
			gp.x = -(params_ptr->last_docked_distance_offset_ - 0.02);
		}
		
		if (!pose_x_init_recoreded_)
		{
			pose_x_init_ = current_pose.getOrigin().getX();
			pose_x_init_recoreded_ = true;
			RCLCPP_DEBUG(logger_, "recored the pose_x: %f", pose_x_init_);
		}

		RCLCPP_DEBUG(logger_, "goal =>  x: %f, y: %f, yaw: %f",
		             gp.x, gp.y, gp.theta);
		RCLCPP_DEBUG(logger_, "robot =>  x: %f, y: %f, yaw: %f degree.",
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
		auto dist_low_speed = params_ptr->last_docked_distance_offset_ + params_ptr->distance_low_speed;
		auto dist_speed_down_length = (params_ptr->go_to_goal_translation_max + params_ptr->go_to_goal_translation_min) / 2.0 * 
			((params_ptr->go_to_goal_translation_max - params_ptr->go_to_goal_translation_min) / params_ptr->go_to_goal_linear_acc);
		auto dist_speed_down_range = params_ptr->go_to_goal_translation_max - params_ptr->go_to_goal_translation_min;
		auto dist_speed_down = dist_low_speed + dist_speed_down_length;

		if(robot_abs_x >= std::abs(pose_x_init_)) // linear_low_speed(0.05)
		{
			translate_velocity = params_ptr->go_to_goal_translation_min;
		}

		if((robot_abs_x > dist_speed_down) && (robot_abs_x < std::abs(pose_x_init_))) // linear_low_speed(0.05) => linear_max_speed(0.2)
		{
			double max_linear_speed = params_ptr->go_to_goal_translation_max;
			translate_velocity = std::min(params_ptr->go_to_goal_translation_min + 
				(std::abs(pose_x_init_) - robot_abs_x) / dist_speed_down_length * dist_speed_down_range, max_linear_speed);
		}
		
		if ((robot_abs_x <= dist_speed_down) && robot_abs_x >= dist_low_speed) // linear_max_speed(0.2) => linear_low_speed(0.05)
		{
			translate_velocity = params_ptr->go_to_goal_translation_max -
						(dist_speed_down - robot_abs_x) / dist_speed_down_length * dist_speed_down_range;
		}

		if (robot_abs_x < dist_low_speed) // linear_low_speed(0.05)
		{
			translate_velocity = params_ptr->go_to_goal_translation_min;
		}

		// If robot is close enough to goal, move to final stage
		if (dist_to_goal < goal_points_.front().radius || std::abs(current_position.getX()) < std::abs(gp.x)) {
			navigate_state_ = NavigateStates::GOAL_ANGLE;
			RCLCPP_DEBUG(logger_, " ******** change to state GOAL_ANGLE ******** ");
			servo_vel->linear.x = gp.drive_backwards ? -translate_velocity : translate_velocity;
			RCLCPP_DEBUG(logger_, "linear_x: %f", servo_vel->linear.x);
			state = std::string("GO_TO_GOAL_POSITION");
			infos = std::string("GO_TO_GOAL_POSITION converged ==> change state to GOAL_ANGLE");
			// If robot angle has deviated too much from path, reset
		}
		// else if (abs_ang > params_ptr->go_to_goal_angle_too_far && delta_y > params_ptr->dist_error_y_1 && (delta_x + delta_y) > params_ptr->dist_error_x_and_y) {
		// navigate_state_ = NavigateStates::ANGLE_TO_GOAL;
		// RCLCPP_DEBUG(logger_, " ******** change to state ANGLE_TO_GOAL ******** ");
		// If neither of above conditions met, drive towards goal
		// }
		else {
			// only use low speed for test
			// translate_velocity = params_ptr->go_to_goal_translation_min;

			if (gp.drive_backwards) {
				translate_velocity *= -1;
			}
			
			// double angle_dist = angles::shortest_angular_distance(current_angle, 0);
			if(std::abs(current_position.getX()) < (params_ptr->last_docked_distance_offset_ + params_ptr->distance_low_speed))
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

				if (std::abs(current_position.getX()) < (params_ptr->last_docked_distance_offset_ + params_ptr->last_goal_angle_to_x_positive_dis) )
				{					
					double ang2 = angles::shortest_angular_distance(current_angle, 0);
					if (ang2 < 0 && std::abs(ang2) > params_ptr->go_to_goal_apply_rotation_angle && current_position.getY() > -params_ptr->last_goal_angle_to_x_positive_y)
					{
						RCLCPP_DEBUG(logger_, "ang2: %f, y: %f, angle_to_x_positive direction", ang2, current_position.getY());
						servo_vel->linear.x = -0.05;
						ang = ang2;
					}
					else if (ang2 > 0 && std::abs(ang2) > params_ptr->go_to_goal_apply_rotation_angle && current_position.getY() < params_ptr->last_goal_angle_to_x_positive_y)
					{
						RCLCPP_DEBUG(logger_, "ang2: %f, y: %f, angle_to_x_positive direction", ang2, current_position.getY());
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
				RCLCPP_DEBUG(logger_, "diff angle_to_goal: %f", ang);
				RCLCPP_DEBUG(logger_, "abs_angle: %f", abs_ang);
				RCLCPP_DEBUG(logger_, "thre: %f", params_ptr->go_to_goal_apply_rotation_angle);
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
			RCLCPP_DEBUG(logger_, "linear_x: %f", servo_vel->linear.x);
			RCLCPP_DEBUG(logger_, "angular.z: %f", servo_vel->angular.z);

		}
		break;
	}
	case NavigateStates::GOAL_ANGLE:
	{
		RCLCPP_DEBUG(logger_, "***********************************");
		RCLCPP_DEBUG(logger_, "***********************************");
		RCLCPP_DEBUG(logger_, "------------- GOAL_ANGLE -------------");
		const GoalPoint & gp = goal_points_.front();
		RCLCPP_DEBUG(logger_, "goal =>  x: %f, y: %f, yaw: %f",
		             gp.x, gp.y, gp.theta);
		RCLCPP_DEBUG(logger_, "robot =>  x: %f, y: %f, yaw: %f",
		             current_pose.getOrigin().getX(), current_pose.getOrigin().getY(),
		             current_angle);
		double ang = angles::shortest_angular_distance(current_angle, gp.theta);
		bound_rotation(ang, params_ptr->go_to_goal_rotation_min, params_ptr->go_to_goal_rotation_max);
		RCLCPP_DEBUG(logger_, "diff angle: %f", ang);
		servo_vel = geometry_msgs::msg::Twist();

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
			navigate_state_ = NavigateStates::GO_TO_GOAL_POSITION;
			RCLCPP_DEBUG(logger_, "******** change to state GO_TO_GOAL_POSITION ******** ");
		}
		RCLCPP_DEBUG(logger_, " linear_x: %f", servo_vel->linear.x);
		RCLCPP_DEBUG(logger_, "angular.z: %f", servo_vel->angular.z);
		state = std::string("GOAL_ANGLE");
		infos = std::string("GOAL_ANGLE  ==> keep on rotating");
		break;
	}
	case NavigateStates::UNDOCK:
	{
		RCLCPP_DEBUG(logger_, "------------- UNDOCK -------------");
		
		servo_vel = geometry_msgs::msg::Twist();
		if(start_undock)
		{
			undock_start_time = clock_->now().seconds();
			start_undock = false;
			RCLCPP_DEBUG(logger_, "undock_start_time: %f", undock_start_time);
			undock_time = params_ptr->undock_time;
			undock_speed = params_ptr->undock_speed;
			RCLCPP_DEBUG(logger_, "undock_speed: %f", undock_speed);
			RCLCPP_DEBUG(logger_, "undock_time: %f", undock_time);
		}
		double now_time = clock_->now().seconds();
		double delta_time = now_time - undock_start_time;
		RCLCPP_DEBUG(logger_, "delta_time: %f", delta_time);
		if (delta_time < undock_time)
		{
			servo_vel->linear.x = undock_speed;
		}
		else
		{
			goal_points_.clear();
			RCLCPP_INFO(logger_, "undock end.");
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
motion_control_params *params_ptr;

private:
enum class NavigateStates
{
	LOOKUP_ARUCO_MARKER,
	ANGLE_TO_BUFFER_POINT,
	MOVE_TO_BUFFER_POINT,
	ANGLE_TO_X_POSITIVE_ORIENTATION,
	ANGLE_TO_GOAL,
	GO_TO_GOAL_POSITION,
	GOAL_ANGLE,
	UNDOCK
};

struct GoalPoint
{
	double x;
	double y;
	double theta;
	float radius;
	bool drive_backwards;
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
			// RCLCPP_DEBUG(logger_, "x: %f, y: %f, theta: %f", x, y, theta);
			// RCLCPP_DEBUG(logger_,"base footprint");
			// RCLCPP_DEBUG(logger_, "Point(%f, %f)", footprint[0].x, footprint[0].y);
			// RCLCPP_DEBUG(logger_, "Point(%f, %f)", footprint[1].x, footprint[1].y);
			// RCLCPP_DEBUG(logger_, "Point(%f, %f)", footprint[2].x, footprint[2].y);
			// RCLCPP_DEBUG(logger_, "Point(%f, %f)", footprint[3].x, footprint[3].y);
			double cost_value_tmp = collision_checker.footprintCostAtPose(x, y, theta, footprint);
			RCLCPP_DEBUG(logger_, "predict number %d cost_value: %f", i, cost_value_tmp);
			cost_value = std::max(cost_value, cost_value_tmp);
			if (cost_value == static_cast<double>(nav2_costmap_2d::LETHAL_OBSTACLE))
			{
				return cost_value;
			}
		}
	}
	else
	{
		double vel_linear = linear * scale;
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
		RCLCPP_DEBUG(logger_, "last_rotation          : %f", last_rotation);
		RCLCPP_DEBUG(logger_, "cur_rotation           : %f", cur_rotation);
		RCLCPP_DEBUG(logger_, "delta_time             : %f", delta_time);
		RCLCPP_DEBUG(logger_, "rotation_max_change_abs: %f", rotation_max_change_abs);
		RCLCPP_DEBUG(logger_, "rotation_cur_change_abs: %f", rotation_cur_change_abs);
		RCLCPP_DEBUG(logger_, "new_rotation_speed     : %f", new_rotation_speed);
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
	// RCLCPP_DEBUG(logger_, "gp.x: %f, gp.y: %f, cur.x: %f, cur.y: %f",
	//              goal_pt.x, goal_pt.y, cur_position.getX(), cur_position.getY());
	// RCLCPP_DEBUG(logger_, "y       => %f", y);
	// RCLCPP_DEBUG(logger_, "x       => %f", x);
	// RCLCPP_DEBUG(logger_, "atan2   => %f", atan2_value);
	// RCLCPP_DEBUG(logger_, "cur_ang => %f", cur_angle);
	RCLCPP_DEBUG(logger_, "dist    => %f", result);

	return result;
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

std::mutex mutex_;
std::deque<GoalPoint> goal_points_;
NavigateStates navigate_state_;

std::chrono::high_resolution_clock::time_point time_start;
std::chrono::high_resolution_clock::time_point time_end;
int64_t time_cost;

double dist_buffer_point;
double dist_buffer_point_yaw;
double pre_time;
double now_time;
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
bool  start_undock = true;
bool undocking = false;
double undock_start_time;
double undock_time;
double undock_speed;

bool using_localization{true};

bool pose_x_init_recoreded_{false};
double pose_x_init_;

// test for angle_to_buffer_point and move_to_buffer_point
double theta_angle_to_buffer_point;
double dist_move_to_buffer_point;
tf2::Transform tf_before_angle_to_buffer_point;
tf2::Transform tf_after_angle_to_buffer_point;
tf2::Transform tf_after_move_to_buffer_point;

float time_last_local_costmap_clear = 0.0;

};

}  // namespace capella_ros_dock
#endif   // CAPELLA_ROS_DOCK__SIMPLE_GOAL_CONTROLLER_HPP_
