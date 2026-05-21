#ifndef CAPELLA_ROS_DOCK__UTILS_HPP_
#define CAPELLA_ROS_DOCK__UTILS_HPP_

#include <string>

namespace capella_ros_dock{

struct motion_control_params
{
	int timeout_dock_action;
	float min_rotation;
	float max_rotation;
	float min_translation;
	float max_translation;
	float angle_to_goal_angle_converged;
	float go_to_goal_apply_rotation_angle;
	float goal_angle_converged;
	float offset_last_docked_distance;
	float offset_low_speed;
	float offset_second_goal;
	float offset_buffer_goal;
	float camera_horizontal_view;
	float localization_converged_time;
	float tolerance_angle;
	float tolerance_r;
	float deviate_second_goal_x;
	float dist_error_y_1;
	std::string motion_control_log_level;
	int cmd_vel_hz;
	float marker_size;
	float dock_valid_obstacle_x;
	float time_sleep;
	float go_to_goal_rotation_min;
	float go_to_goal_rotation_max;
	float speed_rotation_acceleration;
	float speed_rotation_init_abs;	
	float contacted_keep_move_time;
	float undock_speed;
	float undock_timeout;
	float undock_dis;
	float undock_obstacle_lr;
	float undock_obstacle_front;
	int charger_contact_condition_type;
	float contact_state_change_time_delta;
	float similarity_threshold;
	float camera_baselink_dis;
	float goal_y_correction;
	float go_to_goal_linear_acc;
	float go_to_goal_translation_min;
	float go_to_goal_translation_max;
	float score_weight_similarity;
	float robot_rotate_radius;
	float base_link_y_thr;
	float base_link_dummy_dis;
	float last_goal_angle_to_x_positive_dis;
	float last_goal_angle_to_x_positive_y;
	bool collision_check;
	std::string footprint;
	float collision_predict_time;
	bool enable_clear_local_costmap;
	float timout_clear_local_costmap;
	float odom_twist_scale;
	float timeout_lookup_marker;
	float timeout_angle_to_buffer_point;
	float timeout_move_to_buffer_point;
	float timeout_angle_to_x_positive_orientation;
	float timeout_angle_to_goal;
	float timeout_go_to_goal_position;
	float timeout_goal_angle;
	bool garage_test;
	float offset_buffer_goal2_x;
	float offset_buffer_goal2_y;
	std::string robot_version;
	float footprint_zoom_factor;
	std::vector<std::string> robot_versions;
	std::vector<double> footprint_factors;
	bool use_odom_for_control = true; // true: 使用 Odom (里程计)， false: 使用 Map (全局定位)
};
} // end namespace capella_ros_dock

#endif
