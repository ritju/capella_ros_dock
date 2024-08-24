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
                this->map_code_string_.insert(std::make_pair(0, "INIT"));
                this->map_code_string_.insert(std::make_pair(1, "MARKER_MOVING_MARKER"));
                this->map_code_string_.insert(std::make_pair(2, "MARKER_MOVING_PREDICT"));
                this->map_code_string_.insert(std::make_pair(3, "MARKER_STOP_UPDATE"));
                this->map_code_string_.insert(std::make_pair(4, "MARKER_STOP_KEEP"));
                this->map_code_string_.insert(std::make_pair(5, "NO_MARKER_PREDICT"));
                this->coord_state_current_.code = CoordStateCode::INIT;
                this->coord_state_last_.code = CoordStateCode::INIT;
                RCLCPP_INFO(get_logger(), "coord state: INIT.");

                // declare parameters
                this->declare_parameter("thre_odom_data_valid_count", 2);
                this->declare_parameter("thre_moving_linear", 0.01);
                this->declare_parameter("thre_moving_angular", 0.01);
                this->declare_parameter("score_decline_rate", 0.999);
                this->declare_parameter("score_weight_similarity", 0.5);

                // get parameters
                this->thre_odom_data_valid_count_ = this->get_parameter("thre_odom_data_valid_count").get_value<int>();
                this->thre_moving_linear_         = this->get_parameter("thre_moving_linear").get_value<double>();
                this->thre_moving_angular_        = this->get_parameter("thre_moving_angular").get_value<double>();
                this->score_decline_rate_         = this->get_parameter("score_decline_rate").get_value<double>();
                this->score_weight_similarity_    = this->get_parameter("score_weight_similarity").get_value<double>();

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
                marker_pose_sub_    = this->create_subscription<MarkerPose>("/pose_with_id", 20, std::bind(&CoordOptimize::marker_pose_sub_callback, this, _1));
                odom_sub_           = this->create_subscription<Odom>("/odom", 50, std::bind(&CoordOptimize::odom_sub_callback, this, _1));
                marker_visible_sub_ = this->create_subscription<MarkerVisible>("/marker_visible", 15, std::bind(&CoordOptimize::marker_visible_callback, this, _1));
                cmd_vel_sub_        = this->create_subscription<CmdVel>("cmd_vel", 10, std::bind(&CoordOptimize::cmd_vel_sub_callback, this, _1));

                //pubs
                marker_pose_out_pub_ = this->create_publisher<MarkerPose>("/pose_with_id_optimize", 20);
                
                // timers
                marker_pose_out_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&CoordOptimize::marker_pose_out_timer_callback, this));

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
                this->score_marker_similarity_ = this->sigmoid_(1.0 - similarity) / 0.5 * this->score_weight_similarity_;
                this->score_marker_radius_ = this->sigmoid_(radius) / 0.5 * (1.0 - this->score_weight_similarity_);
                return ( score_marker_similarity_ + score_marker_radius_);
        }

        
        void CoordOptimize::marker_pose_sub_callback(MarkerPose::ConstSharedPtr msg)
        {                       
                marker_pose_in_ = *msg;

                this->similarity_marker_ = marker_pose_in_.similarity;
                this->radius_marker_ = marker_pose_in_.radius;
                score_marker_current_ = get_score(this->similarity_marker_, this->radius_marker_);

                tf2::Transform tf;
                tf.setIdentity();
                tf2::convert(marker_pose_in_.pose.pose, tf);
                this->x_in_ = tf.getOrigin()[0];
                this->y_in_= tf.getOrigin()[1];
                this->theta_in_ = tf2::getYaw(tf.getRotation());       
        }

        void CoordOptimize::odom_sub_callback(Odom::ConstSharedPtr msg)
        {               
                odom_current_ = *msg;
                linear_ = odom_current_.twist.twist.linear.x;
                angular_ = odom_current_.twist.twist.angular.z;
                
                // RCLCPP_DEBUG(get_logger(), "odom callback.");
                // RCLCPP_DEBUG(get_logger(), "odom twist => linear_x: %f, angular_z: %f", linear_, angular_);

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
                        // generate value of robot_state_moving 
                        if (std::abs(linear_) < thre_moving_linear_ && std::abs(angular_) < thre_moving_angular_) // current data => stop
                        {
                                // RCLCPP_DEBUG(get_logger(), "current state: stopping");
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
                                                        // RCLCPP_DEBUG(get_logger(), "change value of marker_visible from true to false, decline score.");
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
                        } // end of current data => stop
                        else // current data => moving
                        {
                                // RCLCPP_DEBUG(get_logger(), "current state: moving");
                                // RCLCPP_DEBUG(get_logger(), "x: %f, thre: %f", linear_, thre_moving_linear_);
                                // RCLCPP_DEBUG(get_logger(), "z: %f, thre: %f", angular_, thre_moving_angular_);
                                odom_data_valid_count_stoping_ = 0;

                                if (!robot_state_moving_last_) // last state => stoping
                                {
                                        odom_data_valid_count_moving_++;
                                        
                                        double now_time = this->get_clock()->now().seconds();

                                        if ((odom_data_valid_count_moving_ >= thre_odom_data_valid_count_) || (this->moving_via_cmd_vel_ && (now_time - this->moving_via_cmd_vel_time_ < 0.13))) // 10hz
                                        {
                                                // RCLCPP_DEBUG(get_logger(), "now_time: %f, moving_via_cmd_vel_time_: %f", now_time, this->moving_via_cmd_vel_time_);
                                                // RCLCPP_DEBUG(get_logger(), "moving_via_cmd_vel_: %s", this->moving_via_cmd_vel_?"true":"false"); 
                                                robot_state_moving_ = !robot_state_moving_last_; // change state
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
                                RCLCPP_DEBUG(get_logger(), "Robot moving state changed from %s to %s", 
                                        robot_state_moving_last_?"true":"false",
                                        robot_state_moving_?"true":"false");
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
                tf2::Transform tf_marker;
                tf2::convert(marker_pose_in_.pose.pose, tf_marker);
                double marker_x, marker_y, marker_theta;
                marker_x = tf_marker.getOrigin().getX();
                marker_y = tf_marker.getOrigin().getY();
                marker_theta = tf2::getYaw(tf_marker.getRotation());
                
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
                                        coord_state_current_.code = CoordStateCode::MARKER_MOVING_MARKER;
                                        if (this->coord_state_current_.code != this->coord_state_last_.code)
                                        {
                                                RCLCPP_DEBUG(get_logger(), "Coord state changed from %s to %s", this->map_code_string_[(int)this->coord_state_last_.code].c_str(), this->map_code_string_[(int)this->coord_state_current_.code].c_str());
                                                RCLCPP_DEBUG(get_logger(), "---------- marker + moving => marker ----------");
                                        }
                                        
                                        double delta_x, delta_y, delta_theta;
                                        delta_x = std::abs(this->x_out_last_ - marker_x);
                                        delta_y = std::abs(this->y_out_last_ - marker_y);
                                        delta_theta = std::abs(this->theta_out_last_ - marker_theta);
                                        if (delta_x > 0.05 || delta_y > 0.05 || delta_theta > 0.035)
                                        {
                                                this->predict_count_++; // fix bug for predict_count=0 when the algorithm is continuously using markers to update coordinates;
                                                RCLCPP_WARN(get_logger(), "********** Coords jumping occurs **********");
                                                double score_p_similarity, score_p_radius;
                                                score_p_similarity = this->sigmoid_(1.0 - this->similarity_out_last_) / 0.5 * this->score_weight_similarity_;
                                                score_p_radius = this->sigmoid_(this->radius_out_last_) / 0.5 * (1.0 - this->score_weight_similarity_);

                                                RCLCPP_DEBUG(get_logger(), "score_marker           :  %f", this->score_marker_current_);
                                                RCLCPP_DEBUG(get_logger(), "score_predict          :  %f", this->score_predict_);
                                                RCLCPP_DEBUG(get_logger(), "score_predict_b_decline:  %f  count_p:  %d", this->score_predict_b_decline_, this->predict_count_);
                                                RCLCPP_DEBUG(get_logger(), "similarity_marker      :  %f  delta_s: %s%f", this->similarity_marker_, (this->similarity_marker_ - this->similarity_out_last_)>0?" ":"", this->similarity_marker_ - this->similarity_out_last_);
                                                RCLCPP_DEBUG(get_logger(), "similarity_predict     :  %f", this->similarity_out_last_);
                                                RCLCPP_DEBUG(get_logger(), "score_m_similarity     :  %f  score_s: %s%f", this->score_marker_similarity_, (this->score_marker_similarity_ - score_p_similarity)>0?" ":"", this->score_marker_similarity_ - score_p_similarity);
                                                RCLCPP_DEBUG(get_logger(), "score_p_similarity     :  %f", score_p_similarity);
                                                RCLCPP_DEBUG(get_logger(), "radius_marker          :  %f  delta_r: %s%f", this->radius_marker_,(this->radius_out_last_ - this->radius_marker_)>0?" ":"", this->radius_out_last_ - this->radius_marker_);
                                                RCLCPP_DEBUG(get_logger(), "radius_predict         :  %f", this->radius_out_last_);
                                                RCLCPP_DEBUG(get_logger(), "score_m_radius         :  %f  score_r: %s%f", this->score_marker_radius_, (this->score_marker_radius_- score_p_radius)>0?" ":"", this->score_marker_radius_- score_p_radius);
                                                RCLCPP_DEBUG(get_logger(), "score_p_raidus         :  %f", score_p_radius);
                                                RCLCPP_DEBUG(get_logger(), "marker_x               : %f  delta_x: %s%f", marker_x, (marker_x - this->x_out_last_)>0?" ":"", marker_x - this->x_out_last_);
                                                RCLCPP_DEBUG(get_logger(), "marker_y               : %s%f  delta_y: %s%f", marker_y, marker_y>0?" ":"" , (marker_y - this->y_out_last_)>0?" ":"",marker_y - this->y_out_last_);
                                                RCLCPP_DEBUG(get_logger(), "marker_theta           : %s%f  delta_t: %s%f", marker_theta, marker_theta>0?" ":"", (marker_theta - this->theta_out_last_)>0?" ":"",marker_theta - this->theta_out_last_);
                                                RCLCPP_DEBUG(get_logger(), "last_x                 : %f", this->x_out_last_);
                                                RCLCPP_DEBUG(get_logger(), "last_y                 : %s%f", this->y_out_last_>0?" ": "", this->y_out_last_);
                                                RCLCPP_DEBUG(get_logger(), "last_theta             : %s%f", this->theta_in_last_>0?" ":"", this->theta_out_last_);
                                        }
                                        
                                        using_predict_pose_ = false;
                                        marker_pose_out_ = marker_pose_in_;
                                        score_predict_ = score_marker_current_;

                                        this->similarity_out_last_ = this->similarity_marker_;
                                        this->radius_out_last_ = this->radius_marker_;
                                        this->score_predict_b_decline_ = this->score_marker_current_;
                                        this->predict_count_ = 0;
                                }
                                else
                                {
                                        using_predict_pose_ = true;
                                        this->predict_count_++;
                                        coord_state_current_.code = CoordStateCode::MARKER_MOVING_PREDICT;
                                        
                                        if (this->coord_state_current_.code != this->coord_state_last_.code)
                                        {
                                                RCLCPP_DEBUG(get_logger(), "Coord state changed from %s to %s", this->map_code_string_[(int)this->coord_state_last_.code].c_str(), this->map_code_string_[(int)this->coord_state_current_.code].c_str());
                                                RCLCPP_DEBUG(get_logger(), "---------- marker + moving => predict ----------");
                                        }


                                }
                        } // end of visible: true  + moving
                        else // visible: true + stopping
                        {                                
                                
                                score_predict_ = score_last_ * 1.0;
                                using_predict_pose_ = false;

                                if (score_marker_best_motionless_ < score_marker_current_) // update pose
                                {
                                        coord_state_current_.code = CoordStateCode::MARKER_STOP_UPDATE;
                                        if (this->coord_state_current_.code != this->coord_state_last_.code)
                                        {
                                                RCLCPP_DEBUG(get_logger(), "Coord state changed from %s to %s", this->map_code_string_[(int)this->coord_state_last_.code].c_str(), this->map_code_string_[(int)this->coord_state_current_.code].c_str());
                                                RCLCPP_DEBUG(get_logger(), "---------- marker + stop => update ----------");
                                        }

                                        double score_p_similarity, score_p_radius;
                                        score_p_similarity = this->sigmoid_(1.0 - this->similarity_out_last_) / 0.5 * this->score_weight_similarity_;
                                        score_p_radius = this->sigmoid_(this->radius_out_last_) / 0.5 * (1.0 - this->score_weight_similarity_);

                                        RCLCPP_DEBUG(get_logger(), "score_marker           :  %f", this->score_marker_current_);
                                        RCLCPP_DEBUG(get_logger(), "score_predict          :  %f", this->score_predict_);
                                        RCLCPP_DEBUG(get_logger(), "score_predict_b_decline:  %f", this->score_predict_b_decline_);
                                        RCLCPP_DEBUG(get_logger(), "similarity_marker      :  %f  delta_s: %s%f", this->similarity_marker_, (this->similarity_marker_ - this->similarity_out_last_)>0?" ":"", this->similarity_marker_ - this->similarity_out_last_);
                                        RCLCPP_DEBUG(get_logger(), "similarity_predict     :  %f", this->similarity_out_last_);
                                        RCLCPP_DEBUG(get_logger(), "score_m_similarity     :  %f  score_s: %s%f", this->score_marker_similarity_, (this->score_marker_similarity_ - score_p_similarity)>0?" ":"", this->score_marker_similarity_ - score_p_similarity);
                                        RCLCPP_DEBUG(get_logger(), "score_p_similarity     :  %f", score_p_similarity);
                                        RCLCPP_DEBUG(get_logger(), "radius_marker          :  %f  delta_r: %s%f", this->radius_marker_,(this->radius_out_last_ - this->radius_marker_)>0?" ":"", this->radius_out_last_ - this->radius_marker_);
                                        RCLCPP_DEBUG(get_logger(), "radius_predict         :  %f", this->radius_out_last_);
                                        RCLCPP_DEBUG(get_logger(), "score_m_radius         :  %f  score_r: %s%f", this->score_marker_radius_, (this->score_marker_radius_- score_p_radius)>0?" ":"", this->score_marker_radius_- score_p_radius);
                                        RCLCPP_DEBUG(get_logger(), "score_p_raidus         :  %f", score_p_radius);
                                        RCLCPP_DEBUG(get_logger(), "marker_x               : %f  delta_x: %s%f", marker_x, (marker_x - this->x_out_last_)>0?" ":"", marker_x - this->x_out_last_);
                                        RCLCPP_DEBUG(get_logger(), "marker_y               : %s%f  delta_y: %s%f", marker_y, marker_y>0?" ":"" , (marker_y - this->y_out_last_)>0?" ":"",marker_y - this->y_out_last_);
                                        RCLCPP_DEBUG(get_logger(), "marker_theta           : %s%f  delta_t: %s%f", marker_theta, marker_theta>0?" ":"", (marker_theta - this->theta_out_last_)>0?" ":"",marker_theta - this->theta_out_last_);
                                        RCLCPP_DEBUG(get_logger(), "last_x                 : %f", this->x_out_last_);
                                        RCLCPP_DEBUG(get_logger(), "last_y                 : %s%f", this->y_out_last_>0?" ": "", this->y_out_last_);
                                        RCLCPP_DEBUG(get_logger(), "last_theta             : %s%f", this->theta_in_last_>0?" ":"", this->theta_out_last_);
                                        
                                        score_marker_best_motionless_ = score_marker_current_;
                                        score_predict_ = score_marker_current_;

                                        this->similarity_out_last_ = this->similarity_marker_;
                                        this->radius_out_last_ = this->radius_marker_;
                                        this->score_predict_b_decline_ = this->score_marker_current_;

                                        // update pose
                                        marker_pose_out_ = marker_pose_in_;
                                        marker_pose_out_.pose.header.stamp = this->get_clock()->now();

                                        x_best_ = this->x_in_;
                                        y_best_ = this->y_in_;
                                        theta_best_ = this->theta_in_;
                                        similarity_best_ = this->similarity_marker_;
                                        radius_best_ = this->radius_marker_;
                                        score_similarity_best_ = score_marker_similarity_;
                                        score_radius_best_ = score_marker_radius_;
                                }
                                else // don't update pose, only update stamp
                                {                                        
                                        coord_state_current_.code = CoordStateCode::MARKER_STOP_KEEP;
                                        if (this->coord_state_current_.code != this->coord_state_last_.code)
                                        {
                                                RCLCPP_DEBUG(get_logger(), "Coord state changed from %s to %s", this->map_code_string_[(int)this->coord_state_last_.code].c_str(), this->map_code_string_[(int)this->coord_state_current_.code].c_str());
                                                RCLCPP_DEBUG(get_logger(), "---------- marker + stop => keep ----------");
                                        }
                                        
                                        // update pose's stamp
                                        marker_pose_out_ = marker_pose_out_last_;
                                        marker_pose_out_.pose.header.stamp = this->get_clock()->now();
                                }
                                this->predict_count_ = 0;
                        } // end of marker_visible:true + stopping
                } // end of marker_visible:true
                else // marker_visible: false
                {
                        this->predict_count_++;
                        coord_state_current_.code = CoordStateCode::NO_MARKER_PREDICT;
                        if (this->coord_state_current_.code != this->coord_state_last_.code)
                        {
                                RCLCPP_DEBUG(get_logger(), "Coord state changed from %s to %s", this->map_code_string_[(int)this->coord_state_last_.code].c_str(), this->map_code_string_[(int)this->coord_state_current_.code].c_str());
                                RCLCPP_DEBUG(get_logger(), "---------- No marker => predict ----------");
                        }
                       
                        score_predict_ = score_last_ * score_decline_rate_;
                        using_predict_pose_ = true;
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
                
                if (using_predict_pose_)
                {
                        // RCLCPP_DEBUG(get_logger(), "score_predict          : %f", this->score_predict_);
                        // RCLCPP_DEBUG(get_logger(), "score_predict_b_decline: %f", this->score_predict_b_decline_);
                        // RCLCPP_DEBUG(get_logger(), "similarity_predict     : %f", this->similarity_out_last_);
                        // RCLCPP_DEBUG(get_logger(), "score_p_similarity     : %f", this->sigmoid_(1.0 - this->similarity_out_last_) / 0.5 * this->score_weight_similarity_);
                        // RCLCPP_DEBUG(get_logger(), "radius_predict         : %f", this->radius_out_last_);
                        // RCLCPP_DEBUG(get_logger(), "score_p_raidus         : %f", this->sigmoid_(this->radius_out_last_) / 0.5 * (1.0 - this->score_weight_similarity_));
                        // RCLCPP_DEBUG(get_logger(), "x_out                  : %f", this->x_out_);
                        // RCLCPP_DEBUG(get_logger(), "y_out                  : %f", this->y_out_);
                        // RCLCPP_DEBUG(get_logger(), "theta_out              : %f", this->theta_out_ );
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
                this->coord_state_last_ = this->coord_state_current_;
                
        }

        void CoordOptimize::cmd_vel_sub_callback(CmdVel::ConstSharedPtr msg)
        {
                double linear, angular;
                linear = std::abs(msg->linear.x);
                angular = std::abs(msg->angular.z);
                this->moving_via_cmd_vel_ = (linear > 0.01) || (angular > 0.01);
                this->moving_via_cmd_vel_time_ = this->get_clock()->now().seconds();
                // RCLCPP_DEBUG(get_logger(), "cmd_vel => linear_x: %f, angular_z: %f", linear, angular);
                // RCLCPP_DEBUG(get_logger(), "moving_via_cmd_vel_time_: %f", this->moving_via_cmd_vel_time_);
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






