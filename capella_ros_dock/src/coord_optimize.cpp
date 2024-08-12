#include "capella_ros_dock/coord_optimize.hpp"

using std::placeholders::_1;

namespace capella_ros_dock
{
        CoordOptimize::CoordOptimize(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : rclcpp::Node("coord_optimize", options)
        {
                RCLCPP_INFO(get_logger(), "CoordOptimize constructor.");

                // init 
                tf_odom_last_to_current_.setIdentity();
                tf_odom_last_.setIdentity();
                tf_baselink_to_baselink_dummy_.setIdentity();
                score_marker_best_ = 0.0;

                marker_pose_out_ = MarkerPose();
                marker_pose_out_.pose.header.stamp = this->get_clock()->now();
                marker_pose_out_.pose.header.frame_id = std::string("charger");
                marker_pose_out_.pose.pose.position.x = 0.0;
                marker_pose_out_.pose.pose.position.y = 0.0;
                marker_pose_out_.pose.pose.position.z = 0.0;
                marker_pose_out_.pose.pose.orientation.w = 1.0;
                marker_pose_out_.pose.pose.orientation.x = 0.0;
                marker_pose_out_.pose.pose.orientation.y = 0.0;
                marker_pose_out_.pose.pose.orientation.z = 0.0;

                //subs
                marker_pose_sub_ =     this->create_subscription<MarkerPose>("/pose_with_id", 20, std::bind(&CoordOptimize::marker_pose_sub_callback, this, _1));
                odom_sub_ =            this->create_subscription<Odom>("/odom", 50, std::bind(&CoordOptimize::odom_sub_callback, this, _1));
                marker_visible_sub_ =  this->create_subscription<MarkerVisible>("/marker_visible", 15, std::bind(&CoordOptimize::marker_visible_callback, this, _1));

                //pubs
                marker_pose_out_pub_ = this->create_publisher<MarkerPose>("/pose_with_id_optimize", 20);
                
                // timers
                marker_pose_out_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&CoordOptimize::marker_pose_out_timer_callback, this));

                // tf
                tf_buffer_   = std::make_unique<tf2_ros::Buffer>(this->get_clock());
                tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
                tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        }

        double CoordOptimize::sigmoid_(double value)
        {
                return (1.0 - 1 / ( 1 + exp(-value)));
        }

        double CoordOptimize::get_score(double similarity, double radius)
        {
                score_similarity_current_ = this->sigmoid_(1.0 - similarity);
                score_radius_current_ = this->sigmoid_(radius);
                return ( score_similarity_current_ + score_radius_current_);
        }

        
        void CoordOptimize::marker_pose_sub_callback(MarkerPose::ConstSharedPtr msg)
        {
                marker_pose_in_ = *msg;

                double similarity, radius;
                similarity = marker_pose_in_.similarity;
                radius = marker_pose_in_.radius;
                score_marker_current_ = get_score(similarity, radius);

                tf2::Transform tf;
                tf.setIdentity();
                tf2::convert(msg->pose.pose, tf);
                double x, y, theta;
                x = tf.getOrigin()[0];
                y = tf.getOrigin()[1];
                theta = tf2::getYaw(tf.getRotation());

                if (!robot_moved_)
                {
                        if (score_marker_best_ < score_marker_current_)
                        {
                                score_marker_best_ = score_marker_current_;
                                marker_pose_out_ = marker_pose_in_;
                                marker_pose_out_.pose.header.stamp = this->get_clock()->now();
                                x_best_ = x;
                                y_best_ = y;
                                theta_best_ = theta;
                                similarity_best_ = similarity;
                                radius_best_ = radius;
                                score_similarity_best_ = score_similarity_current_;
                                score_radius_best_ = score_radius_current_;
                                score_marker_best_ = score_similarity_best_ + score_radius_best_;
                                RCLCPP_INFO(get_logger(), "************************** update pose **************************");
                                RCLCPP_INFO(get_logger(), "x: %f, y: %f, theta: %f", x_best_, y_best_, theta_best_);
                                RCLCPP_INFO(get_logger(), "similarity: %f, radius: %f", similarity_best_, radius_best_);
                                RCLCPP_INFO(get_logger(), "score_similarity: %f, score_radius: %f, score_best: %f", score_similarity_best_, score_radius_best_, score_marker_best_);
                                RCLCPP_INFO(get_logger(), "******************************************************************\n");
                        }
                        else
                        {
                                marker_pose_out_.pose.header.stamp = this->get_clock()->now();
                                RCLCPP_DEBUG(get_logger(), "------------------------- discard -------------------------");
                                RCLCPP_DEBUG(get_logger(), "x: %f, y: %f, theta: %f", x, y, theta);
                                RCLCPP_DEBUG(get_logger(), "similarity: %f, radius: %f", similarity, radius);
                                RCLCPP_DEBUG(get_logger(), "score_similarity: %f, score_radius: %f, score_: %f", score_similarity_current_, score_radius_current_, score_marker_current_);

                                RCLCPP_DEBUG(get_logger(), "x: %f, y: %f, theta: %f", x_best_, y_best_, theta_best_);
                                RCLCPP_DEBUG(get_logger(), "similarity: %f, radius: %f", similarity_best_, radius_best_);
                                RCLCPP_DEBUG(get_logger(), "score_similarity: %f, score_radius: %f, score_best: %f", score_similarity_best_, score_radius_best_, score_marker_best_);
                                RCLCPP_DEBUG(get_logger(), "-----------------------------------------------------------\n");
                        }
                }
                else // robot moved
                {

                        marker_pose_out_ = marker_pose_in_;
                        marker_pose_out_.pose.header.stamp = this->get_clock()->now();
                }
                
        }

        void CoordOptimize::odom_sub_callback(Odom::ConstSharedPtr msg)
        {
                
                // get tf from base_link_dummy to odom
                geometry_msgs::msg::TransformStamped tf_stamped_msg;
                tf2::Stamped<tf2::Transform> tf_stamped;
                if (!tf_baselink_to_baselink_dummy_bool_)
                {
                        if (getTransform(std::string("base_link"), std::string("base_link_dummy"), tf_stamped_msg))
                        {
                                tf_baselink_to_baselink_dummy_bool_ = true;
                                tf2::fromMsg(tf_stamped_msg, tf_stamped);
                                tf_baselink_to_baselink_dummy_ = static_cast<tf2::Transform>(tf_stamped);
                        }
                        else
                        {
                                tf_baselink_to_baselink_dummy_bool_ = false;
                        }
                }
                else
                {
                        odom_current_ = *msg;

                        linear_ = odom_current_.twist.twist.linear.x;
                        angular_ = odom_current_.twist.twist.angular.z;
                        if (std::abs(linear_) < 0.01 && std::abs(angular_) < 0.01)
                        {
                                robot_moved_ = false;
                        }
                        else
                        {
                                robot_moved_ = true;
                                score_marker_best_ = 0.0;
                        }

                        tf2::convert(odom_current_.pose.pose, tf_odom_current_);
                        tf_odom_last_to_current_ = (tf_odom_last_ * tf_baselink_to_baselink_dummy_).inverse() * (tf_odom_current_ * tf_baselink_to_baselink_dummy_) ;
                        tf_odom_last_ = tf_odom_current_;

                        if (marker_visible_.marker_visible)
                        {

                        }
                        else
                        {
                                // predict  marker_pose_out_                  
                                tf2::Stamped<tf2::Transform> tf_stamped;
                                geometry_msgs::msg::TransformStamped msgStamped;
                                msgStamped.header.frame_id = std::string("charger");
                                msgStamped.header.stamp = this->get_clock()->now();
                                msgStamped.child_frame_id = std::string("base_link_dummy");
                                msgStamped.transform.translation.x = marker_pose_out_.pose.pose.position.x;
                                msgStamped.transform.translation.y = marker_pose_out_.pose.pose.position.y;
                                msgStamped.transform.translation.z = marker_pose_out_.pose.pose.position.z;
                                msgStamped.transform.rotation      = marker_pose_out_.pose.pose.orientation;
                                tf2::fromMsg(msgStamped, tf_stamped);
                                tf2::Transform tf_old, tf_new;
                                tf_old.setIdentity();
                                tf_new.setIdentity();
                                tf_old = static_cast<tf2::Transform>(tf_stamped);
                                tf_new = tf_old * tf_odom_last_to_current_;
                                // RCLCPP_INFO(get_logger(), "delta_tf => x: %f, y: %f, theta: %f", 
                                //         tf_odom_last_to_current_.getOrigin()[0],
                                //         tf_odom_last_to_current_.getOrigin()[1],
                                //         tf2::getYaw(tf_odom_last_to_current_.getRotation() 
                                //         ));
                                tf2::toMsg(tf_new, marker_pose_out_.pose.pose);
                                marker_pose_out_.pose.header.stamp = this->get_clock()->now();                        
                                
                        }
                }               
        }

        void CoordOptimize::marker_visible_callback(MarkerVisible::ConstSharedPtr msg)
        {
                marker_visible_ = *msg;
                if (marker_visible_.marker_visible)
                {
                        marker_pose_last_time_ = now().seconds();
                        marker_pose_init_ = true;
                }
        }

        void CoordOptimize::marker_pose_out_timer_callback()
        {
                marker_pose_out_pub_->publish(marker_pose_out_);

                geometry_msgs::msg::TransformStamped stampedTransform;
                stampedTransform.header = marker_pose_out_.pose.header;
                stampedTransform.child_frame_id = std::string("base_link_dummy_optimize");
                stampedTransform.transform.translation.x = marker_pose_out_.pose.pose.position.x;
                stampedTransform.transform.translation.y = marker_pose_out_.pose.pose.position.y;
                stampedTransform.transform.translation.z = marker_pose_out_.pose.pose.position.z;
                stampedTransform.transform.rotation      = marker_pose_out_.pose.pose.orientation;
                tf_broadcaster_->sendTransform(stampedTransform);
                
        }

        bool CoordOptimize::getTransform(
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
}

RCLCPP_COMPONENTS_REGISTER_NODE(capella_ros_dock::CoordOptimize)






