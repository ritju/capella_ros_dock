#ifndef CAPELLA_ROS_DOCK__COORD_OPTIMIZE_HPP
#define CAPELLA_ROS_DOCK__COORD_OPTIMIZE_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "capella_ros_service_interfaces/msg/charge_marker_visible.hpp"
#include "aruco_msgs/msg/pose_with_id.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <tf2_ros/transform_broadcaster.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace capella_ros_dock
{
        using MarkerPose = aruco_msgs::msg::PoseWithId;
        using Twist = geometry_msgs::msg::Twist;
        using Odom = nav_msgs::msg::Odometry;
        using MarkerVisible = capella_ros_service_interfaces::msg::ChargeMarkerVisible;

/**
 * @brief This class generate coords in frame of charger marker
 */
        class CoordOptimize : public rclcpp::Node
        {
        public:

                explicit CoordOptimize(const rclcpp::NodeOptions &options);
                ~CoordOptimize() = default;

                // subs
                rclcpp::Subscription<MarkerPose>::SharedPtr marker_pose_sub_;
                rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
                rclcpp::Subscription<MarkerVisible>::SharedPtr marker_visible_sub_;

                // pubs
                rclcpp::Publisher<MarkerPose>::SharedPtr marker_pose_out_pub_;
                rclcpp::TimerBase::SharedPtr marker_pose_out_timer_;

                double get_score(double similarity, double radius);
                double sigmoid_(double value);
                
                // callbacks
                void marker_pose_sub_callback(MarkerPose::ConstSharedPtr msg);
                void odom_sub_callback(Odom::ConstSharedPtr msg);
                void marker_visible_callback(MarkerVisible::ConstSharedPtr msg);
                void marker_pose_out_timer_callback();

                bool getTransform(
                        const std::string & refFrame, const std::string & childFrame,
                        geometry_msgs::msg::TransformStamped & transform);


                // member variables
                MarkerPose marker_pose_in_;
                MarkerPose marker_pose_out_;
                double x_in_, y_in_, theta_in_;
                double x_out_, y_out_, theta_out_;
                double x_in_last_, y_in_last_, theta_in_last_;
                double x_out_last_, y_out_last_, theta_out_last_;

                // debug
                double x_in_delta_max_ = 0.0;
                double y_in_delta_max_ = 0.0;
                double theta_in_delta_max_ = 0.0;
                double x_out_delta_max_ = 0.0;
                double y_out_delta_max_ = 0.0;
                double theta_out_delta_max_ = 0.0;


                double x_best_, y_best_, theta_best_, similarity_best_, radius_best_;
                MarkerVisible marker_visible_;
                bool marker_pose_init_{false};
                float marker_pose_timeout_ = 60.0;
                float marker_pose_last_time_;
                double linear_, angular_;

                bool tf_baselink_to_baselink_dummy_bool_{false};
                
                double score_similarity_current_, score_radius_current_;
                double score_similarity_best_, score_radius_best_;

                // tf listener
                std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
                std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
                
                // tf broadcaster
                std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

                
                double score_decline_rate_ = 0.999;
                MarkerPose marker_pose_in_last_, marker_pose_out_last_;

                // scores
                double score_marker_best_motionless_;
                double score_last_{0.0};
                double score_marker_current_;
                double score_predict_;

                // odom msg
                Odom odom_current_;

                // tfs
                tf2::Transform tf_odom_last_, tf_odom_current_;
                tf2::Transform tf_odom_last_to_current_;
                tf2::Transform tf_baselink_to_baselink_dummy_;

                // threshold value for moving or not 
                double thre_moving_linear_ = 0.01;
                double thre_moving_angular_ = 0.01;

                bool using_predict_pose_{false};
                bool robot_state_moving_{false};
                bool robot_state_moving_last_{false};

                int thre_odom_data_valid_count_ = 3;
                int odom_data_valid_count_moving_ = 0;
                int odom_data_valid_count_stoping_ = 0;

                double similarity_, radius_;


        };
}



#endif







