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
                score_marker_best_motionless_ = 0.0;

                // declare parameters
                this->declare_parameter("thre_odom_data_valid_count", 2);
                this->declare_parameter("thre_moving_linear", 0.01);
                this->declare_parameter("thre_moving_angular", 0.01);
                this->declare_parameter("score_decline_rate", 0.999);

                // get parameters
                this->thre_odom_data_valid_count_ = this->get_parameter("thre_odom_data_valid_count").get_value<int>();
                this->thre_moving_linear_         = this->get_parameter("thre_moving_linear").get_value<double>();
                this->thre_moving_angular_        = this->get_parameter("thre_moving_angular").get_value<double>();
                this->score_decline_rate_         = this->get_parameter("score_decline_rate").get_value<double>();

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

                this->similarity_ = marker_pose_in_.similarity;
                this->radius_ = marker_pose_in_.radius;
                score_marker_current_ = get_score(this->similarity_, this->radius_);

                tf2::Transform tf;
                tf.setIdentity();
                tf2::convert(marker_pose_in_.pose.pose, tf);
                this->x_in_ = tf.getOrigin()[0];
                this->y_in_= tf.getOrigin()[1];
                this->theta_in_ = tf2::getYaw(tf.getRotation());       
        }

        void CoordOptimize::odom_sub_callback(Odom::ConstSharedPtr msg)
        {
                
                geometry_msgs::msg::TransformStamped tf_stamped_msg;
                tf2::Stamped<tf2::Transform> tf_stamped;

                if (!tf_baselink_to_baselink_dummy_bool_)
                {
                        // get tf from base_link to base_link_dummy
                        if (getTransform(std::string("base_link"), std::string("base_link_dummy"), tf_stamped_msg))
                        {
                                tf_baselink_to_baselink_dummy_bool_ = true;
                                tf2::fromMsg(tf_stamped_msg, tf_stamped);
                                tf_baselink_to_baselink_dummy_ = static_cast<tf2::Transform>(tf_stamped);
                                RCLCPP_INFO(get_logger(), "Get TF from base_link frame to base_link_dummy frame.");
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

                        // generate value of robot_state_moving 
                        if (std::abs(linear_) < thre_moving_linear_ && std::abs(angular_) < thre_moving_angular_) // current data => stop
                        {
                                odom_data_valid_count_moving_ = 0;

                                if (robot_state_moving_last_) // last state => moving
                                {
                                        odom_data_valid_count_stoping_++;

                                        if (odom_data_valid_count_stoping_ >= thre_odom_data_valid_count_)
                                        {
                                                robot_state_moving_ = !robot_state_moving_last_; // change state
                                                
                                                // case: (marker_visible:true + change from moving to stopping) => update score_marker_best_motionless_
                                                if (robot_state_moving_last_ && marker_visible_.marker_visible)
                                                {
                                                        RCLCPP_DEBUG(get_logger(), "change value of marker_visible from true to false, decline score.");
                                                        score_marker_best_motionless_ = score_last_ * score_decline_rate_;
                                                }
                                        }
                                        else
                                        {
                                                robot_state_moving_ = robot_state_moving_last_; // keep state
                                        }

                                }
                                else // last state => stoping
                                {
                                        robot_state_moving_ = robot_state_moving_last_; // keep state
                                }
                        }
                        else // current data => moving
                        {
                                odom_data_valid_count_stoping_ = 0;

                                if (!robot_state_moving_last_) // last state => stoping
                                {
                                        odom_data_valid_count_moving_++;
                                        if (odom_data_valid_count_moving_ >= thre_odom_data_valid_count_) // change state
                                        {
                                                robot_state_moving_ = !robot_state_moving_last_;
                                        }
                                        else // keep state
                                        {
                                                robot_state_moving_ = robot_state_moving_last_;
                                        }

                                }
                                else // last state => moving
                                {
                                        robot_state_moving_ = robot_state_moving_last_; // keep state
                                }
                        } // end of generating state

                                 
                        // echo infos when robot moving state changed for debug;
                        if(robot_state_moving_last_ != robot_state_moving_)
                        {
                                RCLCPP_INFO(get_logger(), "Robot moving state changed from %s to %s", 
                                        robot_state_moving_last_?"true":"false",
                                        robot_state_moving_?"true":"false");
                                RCLCPP_DEBUG(get_logger(), "linear_x: %f, angular_z: %f", linear_, angular_);
                        }
                        robot_state_moving_last_ = robot_state_moving_;

                }  // end of getting tf             
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
                
                // get TF from pre_time to now_time
                tf2::convert(odom_current_.pose.pose, tf_odom_current_);
                tf_odom_last_to_current_ = (tf_odom_last_ * tf_baselink_to_baselink_dummy_).inverse() * (tf_odom_current_ * tf_baselink_to_baselink_dummy_) ;
                
                if (marker_visible_.marker_visible) // marker_visisble: true
                {
                        if (robot_state_moving_)   // visible: true  + moving
                        {
                                score_predict_ = score_last_ * score_decline_rate_;

                                if (score_marker_current_ > score_predict_)
                                {
                                        RCLCPP_DEBUG(get_logger(), "-----------------marker + moving => marker-----------------");
                                        RCLCPP_DEBUG(get_logger(), "score_marker : %f   %s", score_marker_current_, score_marker_current_ > score_predict_ ? "=> selected":"");
                                        RCLCPP_DEBUG(get_logger(), "score_predict: %f   %s", score_predict_,        score_predict_ > score_marker_current_ ? "=> selected":"");

                                        using_predict_pose_ = false;
                                        marker_pose_out_ = marker_pose_in_;
                                        score_predict_ = score_marker_current_;
                                }
                                else
                                {
                                        using_predict_pose_ = true;
                                        RCLCPP_DEBUG(get_logger(), "-----------------marker + moving => predict-----------------");
                                        RCLCPP_DEBUG(get_logger(), "score_marker : %f   %s", score_marker_current_, score_marker_current_ > score_predict_ ? "=> selected":"");
                                        RCLCPP_DEBUG(get_logger(), "score_predict: %f   %s", score_predict_,        score_predict_ > score_marker_current_ ? "=> selected":"");
                                }
                        } // end of visible: true  + moving
                        else // visible: true + stopping
                        {
                                RCLCPP_DEBUG(get_logger(), "-----------------marker + stop => converge-----------------");
                                RCLCPP_DEBUG(get_logger(), "last_score   : %f", score_marker_best_motionless_);
                                RCLCPP_DEBUG(get_logger(), "current_score: %f", score_marker_current_);
                                
                                score_predict_ = score_last_ * 1.0;
                                using_predict_pose_ = false;

                                if (score_marker_best_motionless_ < score_marker_current_) // update pose
                                {
                                        score_marker_best_motionless_ = score_marker_current_;
                                        score_predict_ = score_marker_current_;

                                        // update pose
                                        marker_pose_out_ = marker_pose_in_;
                                        marker_pose_out_.pose.header.stamp = this->get_clock()->now();

                                        x_best_ = this->x_in_;
                                        y_best_ = this->y_in_;
                                        theta_best_ = this->theta_in_;
                                        similarity_best_ = this->similarity_;
                                        radius_best_ = this->radius_;
                                        score_similarity_best_ = score_similarity_current_;
                                        score_radius_best_ = score_radius_current_;
                                        RCLCPP_DEBUG(get_logger(), "************************** update pose **************************");
                                        RCLCPP_DEBUG(get_logger(), "x: %f, y: %f, theta: %f", x_best_, y_best_, theta_best_);
                                        RCLCPP_DEBUG(get_logger(), "similarity: %f, radius: %f", similarity_best_, radius_best_);
                                        RCLCPP_DEBUG(get_logger(), "score_similarity: %f, score_radius: %f, score_best: %f", score_similarity_best_, score_radius_best_, score_marker_best_motionless_);
                                        RCLCPP_DEBUG(get_logger(), "******************************************************************\n");
                                }
                                else // don't update pose, only update stamp
                                {
                                        
                                        // update pose
                                        marker_pose_out_ = marker_pose_out_last_;
                                        marker_pose_out_.pose.header.stamp = this->get_clock()->now();

                                        RCLCPP_DEBUG(get_logger(), "------------------------- discard -------------------------");
                                        RCLCPP_DEBUG(get_logger(), "x: %f, y: %f, theta: %f", this->x_in_, this->y_in_, this->theta_in_);
                                        RCLCPP_DEBUG(get_logger(), "similarity: %f, radius: %f", this->similarity_, this->radius_);
                                        RCLCPP_DEBUG(get_logger(), "score_similarity: %f, score_radius: %f, score_: %f", score_similarity_current_, score_radius_current_, score_marker_current_);

                                        RCLCPP_DEBUG(get_logger(), "x: %f, y: %f, theta: %f", x_best_, y_best_, theta_best_);
                                        RCLCPP_DEBUG(get_logger(), "similarity: %f, radius: %f", similarity_best_, radius_best_);
                                        RCLCPP_DEBUG(get_logger(), "score_similarity: %f, score_radius: %f, score_best: %f", score_similarity_best_, score_radius_best_, score_marker_best_motionless_);
                                        RCLCPP_DEBUG(get_logger(), "-----------------------------------------------------------\n");
                                }
                        } // end of marker_visible:true + stopping
                } // end of marker_visible:true
                else // marker_visible: false
                {
                        RCLCPP_DEBUG(get_logger(), "marker visible: false, decline");
                        score_predict_ = score_last_ * score_decline_rate_;
                        using_predict_pose_ = true;
                        RCLCPP_DEBUG(get_logger(), "*****************No marker => predict*****************");
                        RCLCPP_DEBUG(get_logger(), "score_predict: %f", score_predict_);
                }
                
                if (using_predict_pose_)
                {
                        // get TF of pre_time from charger frame to base_link_dummy frame;          
                        tf2::Stamped<tf2::Transform> tf_stamped;
                        geometry_msgs::msg::TransformStamped msgStamped;

                        msgStamped.child_frame_id  = std::string("base_link_dummy");
                        msgStamped.header = marker_pose_out_last_.pose.header;
                        msgStamped.transform.translation.x = marker_pose_out_last_.pose.pose.position.x;
                        msgStamped.transform.translation.y = marker_pose_out_last_.pose.pose.position.y;
                        msgStamped.transform.translation.z = marker_pose_out_last_.pose.pose.position.z;
                        msgStamped.transform.rotation      = marker_pose_out_last_.pose.pose.orientation;
                        tf2::fromMsg(msgStamped, tf_stamped);
                        tf2::Transform tf_old, tf_new;
                        tf_old.setIdentity();
                        tf_new.setIdentity();
                        tf_old = static_cast<tf2::Transform>(tf_stamped);
                        tf_new = tf_old * tf_odom_last_to_current_;
                        tf2::toMsg(tf_new, marker_pose_out_.pose.pose);
                        marker_pose_out_.pose.header.frame_id = marker_pose_out_.pose.header.frame_id;
                        marker_pose_out_.pose.header.stamp = this->get_clock()->now();
                        marker_pose_out_.marker_id = marker_pose_out_.marker_id;
                        marker_pose_out_.similarity = marker_pose_out_.similarity;
                        marker_pose_out_.radius = marker_pose_out_.radius;
                }
        
                // publish topic /pose_with_id_optimize
                marker_pose_out_pub_->publish(marker_pose_out_);

                // send TF from charger frame to base_link_dummy_optimize frame
                geometry_msgs::msg::TransformStamped stampedTransform;
                stampedTransform.header = marker_pose_out_.pose.header;
                stampedTransform.child_frame_id = std::string("base_link_dummy_optimize");
                stampedTransform.transform.translation.x = marker_pose_out_.pose.pose.position.x;
                stampedTransform.transform.translation.y = marker_pose_out_.pose.pose.position.y;
                stampedTransform.transform.translation.z = marker_pose_out_.pose.pose.position.z;
                stampedTransform.transform.rotation      = marker_pose_out_.pose.pose.orientation;
                tf_broadcaster_->sendTransform(stampedTransform);

                tf2::Transform tf_output;
                tf2::convert(marker_pose_out_.pose.pose, tf_output);
                this->x_out_ = tf_output.getOrigin()[0];
                this->y_out_ = tf_output.getOrigin()[1];
                this->theta_out_ = tf2::getYaw(tf_output.getRotation());
                if(marker_visible_.marker_visible)
                {
                        RCLCPP_DEBUG(get_logger(), "x_in : %f, y_in : %f, theta_in : %f", this->x_in_, this->y_in_, this->theta_in_);
                }
                RCLCPP_INFO(get_logger(), "x_out: %f, y_out: %f, theta_out: %f", this->x_out_, this->y_out_, this->theta_out_);

                double x_in_delta, y_in_delta, theta_in_delta;
                double x_out_delta, y_out_delta, theta_out_delta;

                x_in_delta = std::abs(this->x_in_ - this->x_in_last_);
                y_in_delta = std::abs(this->y_in_ - this->y_in_last_);
                theta_in_delta = std::abs(this->theta_in_ - this->theta_in_last_);

                x_out_delta = std::abs(this->x_out_ - this->x_out_last_);
                y_out_delta = std::abs(this->y_out_ - this->y_out_last_);
                theta_out_delta = std::abs(this->theta_out_ - this->theta_out_last_);
                
                if (std::abs(this->x_in_) > 0.20 && 
                    std::abs(this->x_out_) > 0.20 &&
                    std::abs(this->x_in_last_) > 0.20 && 
                    std::abs(this->x_out_last_)> 0.20)
                {
                        if(x_in_delta > this->x_in_delta_max_)
                        {
                                this->x_in_delta_max_ = x_in_delta;
                                RCLCPP_DEBUG(get_logger(), "update x_in_delta_max_: %f", x_in_delta_max_);
                        }
                        
                        if(y_in_delta > this->y_in_delta_max_)
                        {
                                this->y_in_delta_max_ = y_in_delta;
                                RCLCPP_DEBUG(get_logger(), "update y_in_delta_max_: %f", y_in_delta_max_);
                        }
                        
                        if(theta_in_delta > this->theta_in_delta_max_)
                        {
                                this->theta_in_delta_max_ = theta_in_delta;
                                RCLCPP_DEBUG(get_logger(), "update theta_in_delta_max_: %f", theta_in_delta_max_);
                        }

                        if(x_out_delta > this->x_out_delta_max_)
                        {
                                this->x_out_delta_max_ = x_out_delta;
                                RCLCPP_INFO(get_logger(), "update x_out_delta_max_: %f", x_out_delta_max_);
                        }

                        if(y_out_delta > this->y_out_delta_max_)
                        {
                                this->y_out_delta_max_ = y_out_delta;
                                RCLCPP_INFO(get_logger(), "update y_out_delta_max_: %f", y_out_delta_max_);
                        }

                        if(theta_out_delta > this->theta_out_delta_max_)
                        {
                                this->theta_out_delta_max_ = theta_out_delta;
                                RCLCPP_INFO(get_logger(), "update theta_out_delta_max_: %f", theta_out_delta_max_);
                        }                        
                }               

                this->x_in_last_ = this->x_in_;
                this->y_in_last_ = this->y_in_;
                this->theta_in_last_ = this->theta_in_;

                this->x_out_last_ = this->x_out_;
                this->y_out_last_ = this->y_out_;
                this->theta_out_last_ = this->theta_out_;

                // save states
                tf_odom_last_ = tf_odom_current_;
                marker_pose_out_last_ = marker_pose_out_;
                score_last_ = score_predict_;
                using_predict_pose_ = false;
                
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






