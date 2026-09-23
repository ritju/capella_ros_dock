

#ifndef CAPELLA_ROS_DOCK__SIMPLE_GOAL_CONTROLLER_HPP_
#define CAPELLA_ROS_DOCK__SIMPLE_GOAL_CONTROLLER_HPP_

#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

#include "capella_ros_dock/behaviors_scheduler.hpp"
#include "capella_ros_dock/utils.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "tf2/utils.h"
#include "visualization_msgs/msg/marker.hpp"

namespace capella_ros_dock
{

/**
 * @brief This class provides an API to give velocity commands given a goal and robot position.
 */
class SimpleGoalController
{
public:
SimpleGoalController(rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node,
                     rclcpp::node_interfaces::NodeClockInterface::SharedPtr node_clock_interface,
                     rclcpp::node_interfaces::NodeLoggingInterface::SharedPtr node_logging_interface,
                     rclcpp::node_interfaces::NodeTopicsInterface::SharedPtr node_topics_interface,
                     motion_control_params *params_ptr);

void init(motion_control_params* params_ptr);

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
void initialize_goal(const CmdPath & cmd_path, const tf2::Transform& delta);

/// \brief Clear goal
void reset();

// ---- /charge/error_info: dock 侧不可重试错误的判定与上报 ----
// 由 DockingBehavior 注入: 只负责上报, 是否停止/收尾由上层(charge_manager/stop)决定
std::function<void(uint16_t, const std::string &)> charge_error_callback_;
// 蓝牙丢失起始时间(秒), <0 表示当前未丢失
double bluetooth_lost_since_ = -1.0;
// 碰撞阻塞累计时长(秒)与上次阻塞时刻(秒), 用于判定"持续被障碍物挡住"
double collision_blocked_accum_ = 0.0;
double last_collision_blocked_time_ = -1.0;
static constexpr double bluetooth_lost_report_delay_ = 40.0;
static constexpr double collision_blocked_report_delay_ = 40.0;

void set_charge_error_callback(std::function<void(uint16_t, const std::string &)> cb);

void report_charge_error(uint16_t code, const std::string & message);

void note_collision_blocked();

// D4: 复用碰撞检查入口, 记录"被障碍物挡住"的累计时长并上报 obstacle
double collision_cost(nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> & collision_checker,
                      tf2::Transform tf_robot, const std::vector<geometry_msgs::msg::Point> & footprint, bool rotation,
                      double linear, double angular, double predict_time, int hz, double scale);

// \brief Generate velocity based on current position and next goal point looking for convergence
// with goal point based on radius.
// \return empty optional if no goal or velocity command to get to next goal point
BehaviorsScheduler::optional_output_t get_velocity_for_position(
    const tf2::Transform & current_pose, const tf2::Transform & robot_pose_map, const tf2::Transform & charger_pose_map, bool sees_dock, bool is_docked, bool bluetooth_connected,
    const nav_msgs::msg::Odometry & odom_msg, std::string & state, std::string & infos, bool& b_timeout_current_state,
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> & collision_checker, const nav2_costmap_2d::Costmap2D & costmap, const std::vector<geometry_msgs::msg::Point> & footprint_vec, const rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr & client_clear_entire_local_costmap);


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
void change_state(NavigateStates& current_state, NavigateStates target_state, const double& current_state_start_time, const double& current_state_timeout);

bool check_current_state_timeout();

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

void bound_rotation(double & rotation_velocity, float min, float max);

// undock时，不计算后边的碰撞检查了，因为undock时机器人和充电桩是有接触的，必然会有碰撞，没必要检查碰撞值了
double get_cost_value_undock(rclcpp::Logger logger_,
                    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D*> & collision_checker, const nav2_costmap_2d::Costmap2D & costmap,
                    tf2::Transform tf_robot, const std::vector<geometry_msgs::msg::Point> & footprint,
                    double linear, double predict_time, int hz, double scale);

float degree_to_radian(float degree);

float radian_to_degree(float theta);

// angular unit: radian
float camera_horizontal_view_y_coord(float theta_robot_to_goal, float camera_horizontal_view,  float camera_baselink_dis, float goal_dis_x);

// smooth rotation speed
float generate_smooth_rotation_speed(const float & last_rotation, double & last_rotation_time, float cur_rotation, motion_control_params* params_ptr, rclcpp::Clock::SharedPtr clock_, rclcpp::Logger logger_);

double diff_angle(const GoalPoint & goal_pt, const tf2::Vector3 & cur_position, double cur_angle, rclcpp::Logger logger_);

bool clear_local_costmap(rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_clear_entire_local_costmap);

// 用于处理未知意外导致的两帧时间间隔与帧率严重不符的情况
// 更新now_time_, pre_time_, delta_time_
void update_time_smart();

void print_current_state_debug(const NavigateStates& state);

// ---- 调试输出 helper: 收敛重复的调试打印, 便于阅读与统一修改 ----
/// @brief 碰撞预测命中致命障碍时收尾: 记录日志, 置零速度轴, 必要时清理 local costmap
/// @param zero_rotation true 置零 angular.z, false 置零 linear.x
/// @return true 表示已阻塞, 调用方应立即 return servo_vel
bool stop_for_collision(double cost_value, BehaviorsScheduler::optional_output_t & servo_vel, bool zero_rotation,
                        const rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr & client_clear_entire_local_costmap);
void log_charger_relative_for_collision(double x_c2r, double yaw_c2r_abs) const;
void log_robot_in_charger_frame(double x, double y, double yaw) const;
void log_converged_3cond(double theta, double base_link_y, double distance_tmp) const;
void log_goal_and_robot(const GoalPoint & gp, const tf2::Vector3 & position, double angle) const;
void log_footprint_points(const std::vector<geometry_msgs::msg::Point> & footprint, double x, double y, double theta) const;
void log_robot_charger_pose_in_map(double robot_x, double robot_y, double charger_x, double charger_y) const;
void log_angle_to_charger_direction(double angle_robot, double angle_charger_to_robot, double dist_angle) const;
void log_base_link_offset(double base_link_x, double base_link_y) const;
void log_move_to_buffer_progress(double odom_linear_x, double dist_buffer_point, double dist_y, double robot_map_x, double robot_map_y) const;
void log_angle_to_x_progress(double robot_yaw_map, double theta_charger_to_robot, double dist_yaw_map) const;

// 保存充电桩在map下的位姿信息和机器人在map和充电桩下的位姿信息
void save_all_poses_infos(tf2::Transform tf_robot_map, tf2::Transform tf_robot_charger, tf2::Transform tf_charger_map, bool sees_dock);

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
tf2::Transform delta_offset_;
double charger_x_map_;
double charger_y_map_;
double charger_yaw_map_;
// 是否可以识别出充电桩码坐标
bool marker_visible_{false};

//  用于调试的 Makers
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_charger_pose_agent_pub_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_charger_pose_apriltag_pub_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_buffer_point2_pub_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_undock_collision_line_pub_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_undock_collision_point_pub_;
rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_undock_point_loop_pub_;

double buffer_point2_x_map, buffer_point2_y_map;

int marker_unseen_times = 0;

}; // end of class SimpleGoalController



}  // namespace capella_ros_dock
#endif   // CAPELLA_ROS_DOCK__SIMPLE_GOAL_CONTROLLER_HPP_
