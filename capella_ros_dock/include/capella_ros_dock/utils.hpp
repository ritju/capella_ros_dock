#ifndef CAPELLA_ROS_DOCK__UTILS_HPP_
#define CAPELLA_ROS_DOCK__UTILS_HPP_

#include <string>

namespace capella_ros_dock{

struct motion_control_params
{
	int max_dock_action_run_time;
	float min_rotation;
	float max_rotation;
	float min_translation;
	float max_translation;
	float angle_to_goal_angle_converged;
	float go_to_goal_angle_too_far;
	float go_to_goal_apply_rotation_angle;
	float goal_angle_converged;
	float dist_goal_converged;
	float last_docked_distance_offset_;
	float distance_low_speed;
	float translate_low_speed;
	float rotation_low_speed;
	float first_goal_distance;
	float second_goal_distance;
	float buffer_goal_distance;
	float camera_horizontal_view;
	float localization_converged_time;
	float tolerance_angle;
	float tolerance_r;
	float deviate_second_goal_x;
	float dist_error_y_1;
	float dist_error_x_and_y;
	std::string motion_control_log_level;
	int cmd_vel_hz;
	float angle_delta;
	float marker_size;
	float dock_valid_obstacle_x;
	float time_sleep;
	float go_to_goal_rotation_min;
	float go_to_goal_rotation_max;
	float speed_rotation_acceleration;
	float speed_rotation_init_abs;
	int max_action_runtime;
	float contacted_keep_move_time;
	float undock_speed;
	float undock_time;
	float undock_obstacle_lr;
	float undock_obstacle_front;
	int charger_contact_condition_type;
	float contact_state_change_time_delta;
	float charging_radius;
	float similarity_threshold;
	float camera_baselink_dis;
	float goal_y_correction;
	float go_to_goal_linear_acc;
	float go_to_goal_translation_min;
	float go_to_goal_translation_max;
	float score_weight_similarity;
	float robot_rotate_radius;
	float base_link_y_thr;
	float base_link_charge_dis;
	float last_goal_angle_to_x_positive_dis;
	float last_goal_angle_to_x_positive_y;
	bool rotation_collision_check;
	std::string footprint;
};
} // end namespace capella_ros_dock

#endif
