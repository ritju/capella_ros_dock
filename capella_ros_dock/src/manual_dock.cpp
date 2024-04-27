#include "capella_ros_dock/manual_dock.hpp"
#include "rclcpp_components/register_node_macro.hpp"


using std::placeholders::_1, std::placeholders::_2;
using namespace std::chrono_literals;

namespace capella_ros_dock 
{
        ManualDock::ManualDock(const rclcpp::NodeOptions & options)
	: rclcpp::Node("manual_dock", options)
        {
                RCLCPP_INFO(this->get_logger(), "manual_dock node start.");
                init_params();

                // publisher
                charger_position_pub_ = this->create_publisher<std_msgs::msg::Bool>("charger_position_bool", rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

                // tf
                tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
	        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

                this->charge_client_ = rclcpp_action::create_client<charge_manager_msgs::action::Charge>(this, "charge");

                // client
                client_bluetooth = this->create_client<charge_manager_msgs::srv::ConnectBluetooth>("/connect_bluetooth");
                client_start_charging = this->create_client<std_srvs::srv::Empty>("/charger/start");

                rclcpp::CallbackGroup::SharedPtr cb_group1 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rclcpp::SubscriptionOptions sub_ops1 = rclcpp::SubscriptionOptions();
                sub_ops1.callback_group = cb_group1;

                rclcpp::CallbackGroup::SharedPtr cb_group2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rclcpp::SubscriptionOptions sub_ops2 = rclcpp::SubscriptionOptions();
                sub_ops2.callback_group = cb_group2;

                rclcpp::CallbackGroup::SharedPtr cb_group3 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rclcpp::SubscriptionOptions sub_ops3 = rclcpp::SubscriptionOptions();
                sub_ops3.callback_group = cb_group3;

                rclcpp::CallbackGroup::SharedPtr cb_group4 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rclcpp::SubscriptionOptions sub_ops4 = rclcpp::SubscriptionOptions();
                sub_ops4.callback_group = cb_group4;

                rclcpp::CallbackGroup::SharedPtr cb_group5 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rclcpp::SubscriptionOptions sub_ops5 = rclcpp::SubscriptionOptions();
                sub_ops5.callback_group = cb_group5;

                rclcpp::CallbackGroup::SharedPtr cb_group6 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rclcpp::SubscriptionOptions sub_ops6 = rclcpp::SubscriptionOptions();
                sub_ops6.callback_group = cb_group6;

                rclcpp::CallbackGroup::SharedPtr cb_group7 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rclcpp::SubscriptionOptions sub_ops7 = rclcpp::SubscriptionOptions();
                sub_ops7.callback_group = cb_group7;

                rclcpp::CallbackGroup::SharedPtr cb_group8 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rclcpp::SubscriptionOptions sub_ops8 = rclcpp::SubscriptionOptions();
                sub_ops8.callback_group = cb_group8;

                rclcpp::CallbackGroup::SharedPtr cb_group9 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                rclcpp::SubscriptionOptions sub_ops9 = rclcpp::SubscriptionOptions();
                sub_ops9.callback_group = cb_group9;

                marker_and_mac_sub_ = this->create_subscription<aruco_msgs::msg::MarkerAndMacVector>(
                        "/id_mac",
                        30,
                        std::bind(&ManualDock::marker_and_mac_sub_callback, this, std::placeholders::_1),
                        sub_ops1
                );
                charger_state_sub_ = this->create_subscription<capella_ros_service_interfaces::msg::ChargeState>(
                        "/charger/state",
                        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile(),
                        std::bind(&ManualDock::charger_state_sub_callback, this, std::placeholders::_1),
                        sub_ops2
                );
                pose_with_id_sub_ = this->create_subscription<aruco_msgs::msg::PoseWithId>(
                        "pose_with_id",
                        30,
                        std::bind(&ManualDock::pose_with_id_sub_callback, this, std::placeholders::_1),
                        sub_ops3
                );

                charger_visible_sub_ = this->create_subscription<capella_ros_service_interfaces::msg::ChargeMarkerVisible>(
                        "marker_visible",
                        30,
                        std::bind(&ManualDock::charger_visible_sub_callback, this, std::placeholders::_1),
                        sub_ops4
                );

                is_undocking_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                        "is_undocking_state",
                        1,
                        std::bind(&ManualDock::is_undocking_state_callback, this, std::placeholders::_1),
                        sub_ops5
                );

                charger_pose_map_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                        "charger/pose",
                        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
                        std::bind(&ManualDock::charger_pose_map_sub_callback, this, std::placeholders::_1),
                        sub_ops6
                );

                localization_score_sub_ = this->create_subscription<std_msgs::msg::Float32>(
                        "/localization_score",
                        5,
                        std::bind(&ManualDock::localization_sub_callback, this, std::placeholders::_1),
                        sub_ops7
                );

                is_docking_state_sub_ = this->create_subscription<std_msgs::msg::Bool>(
                        "is_docking_state",
                        1,
                        std::bind(&ManualDock::is_docking_state_callback, this, std::placeholders::_1),
                        sub_ops8
                );

                odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                        "odom_plicp",
                        1,
                        std::bind(&ManualDock::odom_sub_callback, this, std::placeholders::_1),
                        sub_ops9
                );

                std::thread thread1(std::bind(&ManualDock::manual_dock_check_callback, this));
                thread1.detach();
                // auto cb_group_check_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                // auto charger_position_timer_ = this->create_wall_timer(std::chrono::seconds(10), std::bind(&ManualDock::manual_dock_check_callback, this));

                sleep(1); // important !!! do not delete.
                // publish false for init
                std_msgs::msg::Bool msg_bool;
                msg_bool.data = false;
                charger_position_pub_->publish(msg_bool);
                is_in_charger_range_last = msg_bool.data;
                RCLCPP_INFO(get_logger(), "publish /charger_position_bool false for init.");
        }

        void ManualDock::get_robot_pose_map(float& robot_x, float& robot_y, float& yaw)
        {
                if (localization_score > 0.8)
                {
                        tf2::Stamped<tf2::Transform> robotToMapStamped;
                        robotToMapStamped.setIdentity();
                        tf2::Transform robotToMap;
                        robotToMap.setIdentity();
                        geometry_msgs::msg::TransformStamped transform_stamped;
                        if (getTransform(std::string("map"), std::string("base_link"), transform_stamped))
                        {
                                tf2::fromMsg(transform_stamped, robotToMapStamped);
                                robotToMap = static_cast<tf2::Transform>(robotToMapStamped);
                                auto robot_position = robotToMap.getOrigin();
                                robot_x = robot_position.getX();
                                robot_y = robot_position.getY();
                                yaw = tf2::getYaw(robotToMap.getRotation());
                        }
                        else
                        {                               
                                robot_x = 9999.0; // can not get tf 
                                robot_y = 9999.0;
                                yaw = 0.0;
                        }
                }
                else
                {
                        robot_x = 8888.0;  // low score
                        robot_y = 8888.0;
                        yaw = 0.0;
                }
                // RCLCPP_INFO_THROTTLE(get_logger(), *(this->get_clock()), 5000, "score: %f, robot(%f, %f, %f), charger(%f, %f, %f)", 
                //         localization_score, robot_x_map, robot_y_map, robot_yaw_map,
                //         charger_pose_x, charger_pose_y, charger_pose_yaw);
        }

        void ManualDock::get_robot_pose_charger(float& x, float& y, float& yaw)
        {
                tf2::Transform robot_pose;
                tf2::convert(this->pose_with_id.pose.pose, robot_pose);
                auto position = robot_pose.getOrigin();
                x = position.getX();
                y = position.getY();
                yaw = tf2::getYaw(robot_pose.getRotation());
                // RCLCPP_INFO(this->get_logger(), "pose_x_marker: %f", robot_x_charger);
                // RCLCPP_INFO(this->get_logger(), "pose_y_marker: %f", robot_y_charger);
                // RCLCPP_INFO(this->get_logger(), "yaw_marker: %f", robot_yaw_charger);
        }

        void ManualDock::localization_sub_callback(std_msgs::msg::Float32 msg)
        {
                localization_score = msg.data;
        }

        bool ManualDock::in_charger_range_map()
        {                
                bool ret = false;

                float dist_r, dist_yaw;
                dist_r = std::hypot(robot_x_map - charger_pose_x, robot_y_map - charger_pose_y);
                dist_yaw = std::abs(angles::shortest_angular_distance(robot_yaw_map, charger_pose_yaw));

                RCLCPP_INFO_THROTTLE(get_logger(), *(this->get_clock()), 5000, "dist_r: %f, dist_yaw: %f", dist_r, dist_yaw);
                
                if ( dist_r > 3.0)
                {
                        ret = false;
                }
                else
                {
                        if ( dist_yaw < 0.785)
                        {
                                ret = true;
                        }
                        else
                        {
                                ret = false;
                        }
                }

                return ret;
        }

        void ManualDock::charger_pose_map_sub_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
        {
                tf2::Transform tf;
                tf2::convert(msg->pose.pose, tf);
                charger_pose_x = tf.getOrigin().getX();
                charger_pose_y = tf.getOrigin().getY();
                charger_pose_yaw = tf2::getYaw(tf.getRotation());
                RCLCPP_INFO(this->get_logger(), "charger pose: (x: %f, y: %f)", charger_pose_x, charger_pose_y);
        }

        bool ManualDock::getTransform(
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

        void ManualDock::init_params()
        {
                this->declare_parameter<float>("pose_x_min", -0.9);
                this->declare_parameter<float>("pose_x_max", -0.4);
                this->declare_parameter<float>("pose_y_min", -0.1);
                this->declare_parameter<float>("pose_y_max", 0.1);
                this->declare_parameter<float>("yaw_min", -0.2);
                this->declare_parameter<float>("yaw_max", -0.2);

                this->get_parameter_or<float>("pose_x_min", pose_x_min, -0.9);
                this->get_parameter_or<float>("pose_x_max", pose_x_max, -0.4);
                this->get_parameter_or<float>("pose_y_min", pose_y_min, -0.1);
                this->get_parameter_or<float>("pose_y_max", pose_y_max, 0.1);
                this->get_parameter_or<float>("yaw_min", yaw_min, -0.2);
                this->get_parameter_or<float>("yaw_max", yaw_max, 0.2);
        }

        void ManualDock::marker_and_mac_sub_callback(aruco_msgs::msg::MarkerAndMacVector msg)
        {
                this->marker_and_mac = msg;
        }

        void ManualDock::charger_state_sub_callback(capella_ros_service_interfaces::msg::ChargeState msg)
        {
                this->charger_state = msg;
                get_robot_pose_map(robot_x_map, robot_y_map, robot_yaw_map);
                if(charger_state.has_contact && charger_state.is_charging)
                {
                        // RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 10000, "Display charging pose coordinates per 10s......");
                        odom_charging_x = odom_current_x;
                        odom_charging_y = odom_current_y;
                        odom_charging_yaw = odom_current_yaw;
                        
                        if(localization_score > score_max)
                        {
                                robot_x_map_charging = robot_x_map;
                                robot_y_map_charging = robot_y_map;
                                robot_yaw_map_charging = robot_yaw_map;
                                score_max = localization_score;
                        }                        

                        robot_moved_baselink = false;
                        robot_moved_odom = false;
                }
                else
                {
                        score_max = 0.8;
                        robot_moved_baselink = is_charging_position_moved(robot_x_map_charging, robot_y_map_charging, robot_yaw_map_charging, robot_x_map, robot_y_map, robot_yaw_map);
                        robot_moved_odom = is_charging_position_moved(odom_charging_x, odom_charging_y, odom_charging_yaw, odom_current_x, odom_current_y, odom_current_yaw);
                        RCLCPP_INFO_THROTTLE(get_logger(), *this->get_clock(), 10000, "robot_moved_base_link: %s, robot_moved_odom: %s", robot_moved_baselink ? "true":"false", robot_moved_odom ? "true" : "false" );
                }

                RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 15000, "******************************* charger/state callback *******************************");
                RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 15000,  "robot_map_charging: (%f, %f, %f)", robot_x_map_charging, robot_y_map_charging, robot_yaw_map_charging);
                RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 15000,  "robot_map_current : (%f, %f, %f), localization_score: %f,", robot_x_map, robot_y_map, robot_yaw_map, localization_score);
                RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 15000,  "dist_map: %f ", std::hypot(robot_x_map_charging - robot_x_map, robot_y_map_charging - robot_y_map));
                RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 15000,  "odom_map_charging : (%f, %f, %f)", odom_charging_x, odom_charging_y, odom_charging_yaw);
                RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 15000,  "odom_map_current  : (%f, %f, %f)", odom_current_x, odom_current_y, odom_current_yaw);
                RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 15000,  "dist_odom: %f ", std::hypot(odom_charging_x - odom_current_x, odom_charging_y - odom_current_y));
                RCLCPP_DEBUG_THROTTLE(get_logger(), *this->get_clock(), 15000, "******************************* charger/state callback *******************************");
        }

        void ManualDock::charger_visible_sub_callback(capella_ros_service_interfaces::msg::ChargeMarkerVisible msg)
        {
                this->charger_visible = msg.marker_visible;
                std_msgs::msg::Bool msg_pub;
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 60000, "charger_visible: %s", charger_visible ? "true" : "false");
                if(!charger_visible)
                {
                        use_marker = false;
                        if(localization_score < 0.8)
                        {
                                is_in_charger_range = !robot_moved_odom;
                        }
                        else
                        {
                                if(score_max > 0.8)
                                {
                                        is_in_charger_range =  (!robot_moved_odom) || (!robot_moved_baselink);
                                }
                                else
                                {
                                        is_in_charger_range = !robot_moved_odom;
                                }
                        }                                            
                }
                else
                {
                        use_marker = true;
                        get_robot_pose_charger(robot_x_charger, robot_y_charger, robot_yaw_charger);
                        is_in_charger_range_charger = in_charger_range_charger();
                        is_in_charger_range_map = false;

                        is_in_charger_range = is_in_charger_range_charger || is_in_charger_range_map;
                }

                if (is_in_charger_range != is_in_charger_range_last)
                {
                        msg_pub.data = is_in_charger_range;
                        RCLCPP_INFO(get_logger(), "/charger_position_bool state changed, publish one time ...... value: %s", is_in_charger_range ? "true" : "false");
                        charger_position_pub_->publish(msg_pub);
                        is_in_charger_range_last = is_in_charger_range;
                }
        }

        void ManualDock::pose_with_id_sub_callback(aruco_msgs::msg::PoseWithId msg)
        {
                this->pose_with_id = msg;
                this->marker_id = pose_with_id.marker_id;
                // RCLCPP_INFO(this->get_logger(), "marker_id: %d", marker_id);
                for(int i = 0; i < marker_and_mac.marker_and_mac_vector.size(); i++)
                {
                        if (marker_and_mac.marker_and_mac_vector[i].marker_id == marker_id)
                        {
                                this->bluetooth_mac = marker_and_mac.marker_and_mac_vector[i].bluetooth_mac;
                                break;
                        }
                }                
        }

        void ManualDock::is_undocking_state_callback(std_msgs::msg::Bool msg)
        {
                is_undocking_state = msg.data;
                is_undocking_state_last_time_sub = this->get_clock()->now().seconds();
        }

        void ManualDock::is_docking_state_callback(std_msgs::msg::Bool msg)
        {
                is_docking_state = msg.data;
                is_docking_state_last_time_sub = this->get_clock()->now().seconds();
        }

        bool ManualDock::in_charger_range_charger()
        {
                bool ret = false;
                if (robot_x_charger > pose_x_min && robot_x_charger < pose_x_max &&
                    robot_y_charger > pose_y_min && robot_y_charger < pose_y_max &&
                    robot_yaw_charger > yaw_min && robot_yaw_charger < yaw_max)
                {
                        ret = true;
                }
                return ret;
        }

        void ManualDock::manual_dock_check_callback()
        {
                while(true)
                {                        
                        if(!processing)
                        {
                                processing = true;
                                auto now_time = this->get_clock()->now().seconds();
                                if ((now_time - is_undocking_state_last_time_sub) > is_undocking_state_timeout)
                                {
                                        is_undocking_state = false;
                                }
                                if ((now_time - is_docking_state_last_time_sub) > is_docking_state_timeout)
                                {
                                        is_docking_state = false;
                                }
                                bool manual_charge_satisfied = false;
                                manual_charge_satisfied = charger_visible
                                        && is_in_charger_range_charger 
                                        && !charger_state.is_docking 
                                        && !charger_state.is_charging
                                        && !is_undocking_state
                                        && !is_docking_state;
                                if (manual_charge_satisfied)
                                {
                                        RCLCPP_INFO(this->get_logger(), " ====== manual charging satisfied ======");
                                        auto charge_goal = charge_manager_msgs::action::Charge::Goal();
                                        charge_goal.mac = this->bluetooth_mac;
                                        charge_goal.restore = 0;
                                        charge_goal.type = 1;
                                        auto ret = this->charge_client_->wait_for_action_server(2s);
                                        RCLCPP_INFO(get_logger(), "ret: %s", ret ? "true" : "false");
                                        if(!ret)
                                        {
                                                RCLCPP_INFO(this->get_logger(), "Charge action not on line, waiting ...");
                                        }
                                        else
                                        {
                                                RCLCPP_INFO(get_logger(), "charge_action_executing: %s", charge_action_executing ? "true" : "false");
                                                if (!this->charge_action_executing)
                                                {
                                                        RCLCPP_INFO(this->get_logger(), "------------------------ call /charge action ------------------------");
                                                        this->charge_action_executing = true;
                                                        auto send_goals_options = rclcpp_action::Client<charge_manager_msgs::action::Charge>::SendGoalOptions();
                                                        send_goals_options.result_callback = std::bind(&ManualDock::charge_result_callback, this, _1);
                                                        send_goals_options.goal_response_callback = std::bind(&ManualDock::charge_goal_response_callback, this, _1);
                                                        send_goals_options.feedback_callback = std::bind(&ManualDock::charge_feedback_callback, this, _1, _2);
                                                        this->charge_client_->async_send_goal(charge_goal, send_goals_options);
                                                }
                                                else
                                                {
                                                        RCLCPP_INFO(get_logger(), "Robot is manual-charging .....");
                                                }
                                        }

                                        // auto request1 = std::make_shared<charge_manager_msgs::srv::ConnectBluetooth::Request>();
                                        // request1->mac = this->bluetooth_mac;
                                        // if(!client_bluetooth->wait_for_service(1s))
                                        // {
                                        //         RCLCPP_INFO(this->get_logger(), "connect bluetooth service is not available, waiting...");
                                        // }
                                        // else
                                        // {
                                        //         if ((this->bluetooth_mac.compare(this->charger_state.pid) != 0))
                                        //         {
                                        //                 if(!bluetooth_connecting)
                                        //                 {
                                        //                         bluetooth_connecting = true;
                                        //                         RCLCPP_INFO(this->get_logger(), "*** bluetooth disconnected, manual call /connectbluetooth service. ***");
                                        //                         RCLCPP_INFO(this->get_logger(), "connect bluetooth %s(marker_id: %d)...", this->bluetooth_mac.c_str(), this->marker_id);
                                        //                         auto result1 = client_bluetooth->async_send_request(request1, std::bind(&ManualDock::client_bluetooth_callback, this, _1));
                                        //                 }
                                        //         }                                                
                                        //         else
                                        //         {
                                        //                 if(this->charger_state.has_contact)
                                        //                 {
                                        //                         RCLCPP_INFO(this->get_logger(), "*** bluetooth connected, has_contact: true => munual call service /charger/start service ***");
                                        //                         auto request2 = std::make_shared<std_srvs::srv::Empty::Request>();
                                        //                         auto result2 = client_start_charging->async_send_request(request2);
                                        //                 }
                                        //                 else
                                        //                 {
                                        //                         RCLCPP_INFO(get_logger(), "*** bluetooth connected, has_contact: false => keep on moving robot.");
                                        //                 }
                                        //         }
                                        // }                                  
                                }
                                else
                                { 
                                        if(!charger_visible)
                                        {
                                                RCLCPP_INFO(get_logger(), "Reason: robot can not see the marker.");                                      
                                        }
                                        if(use_marker && !is_in_charger_range_charger)
                                        {
                                                RCLCPP_INFO(get_logger(), "Reason: robot's position is not in charger_range_charger.(%f, %f, %f)", robot_x_charger, robot_y_charger, robot_yaw_charger);
                                        }
                                        if(!use_marker)
                                        {
                                                RCLCPP_INFO(get_logger(), "-----------------------------Reason: use_marker: %s -----------------------------", use_marker ? "true" : "false");
                                                RCLCPP_INFO(get_logger(), "robot_map_charging: (%f, %f, %f)", robot_x_map_charging, robot_y_map_charging, robot_yaw_map_charging);
                                                RCLCPP_INFO(get_logger(), "robot_map_current : (%f, %f, %f), localization_score: %f,", robot_x_map, robot_y_map, robot_yaw_map, localization_score);
                                                RCLCPP_INFO(get_logger(), "dist_map: %f", std::hypot(robot_x_map_charging - robot_x_map, robot_y_map_charging - robot_y_map));
                                                RCLCPP_INFO(get_logger(), "odom_map_charging : (%f, %f, %f)", odom_charging_x, odom_charging_y, odom_charging_yaw);
                                                RCLCPP_INFO(get_logger(), "odom_map_current  : (%f, %f, %f)", odom_current_x, odom_current_y, odom_current_yaw);
                                                RCLCPP_INFO(get_logger(), "dist_odom: %f", std::hypot(odom_current_x - odom_current_x, odom_charging_y - odom_current_y));
                                                RCLCPP_INFO(get_logger(), "-----------------------------Reason: use_marker: %s -----------------------------", use_marker ? "true" : "false");
                                        }
                                        if(is_docking_state)
                                        {
                                                RCLCPP_INFO(get_logger(), "Reason: robot's state is executing charge action.");
                                        }
                                        if (is_undocking_state)
                                        {
                                                RCLCPP_INFO(get_logger(), "Reason: robot's state is undocking.");
                                        }
                                        if(charger_state.is_charging)
                                        {
                                                RCLCPP_INFO(get_logger(), "Reason: robot is charging.");
                                        }                   
                
                                }
                        }
                        sleep(10);
                        processing = false;
                }                
        }

        void ManualDock::charge_result_callback(rclcpp_action::ClientGoalHandle<charge_manager_msgs::action::Charge>::WrappedResult result)
        {
                this->charge_action_executing = false;
                RCLCPP_INFO(get_logger(), "====== manual-charing ended! ======");
        }

        void ManualDock::charge_goal_response_callback(const rclcpp_action::ClientGoalHandle<charge_manager_msgs::action::Charge>::SharedPtr & goal_handle)
        {
                if (!goal_handle)
                {
                        RCLCPP_INFO(this->get_logger(), "Manual dock /charge action was rejected by server.");
                        this->charge_action_executing = false;
                }
                else
                {
                        RCLCPP_INFO(this->get_logger(), "Manual dock /charge action was accepted.");
                        this->charge_action_executing = true;
                }
        }

        void ManualDock::charge_feedback_callback(rclcpp_action::ClientGoalHandle<charge_manager_msgs::action::Charge>::SharedPtr, const std::shared_ptr<const charge_manager_msgs::action::Charge::Feedback> feedback)
        {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "/charge action feedback === state: %s", feedback->state.c_str());
        }

        void ManualDock::client_bluetooth_callback(const rclcpp::Client<charge_manager_msgs::srv::ConnectBluetooth>::SharedFuture future)
        {
                if(future.get()->success)
                {
                        RCLCPP_INFO(this->get_logger(), "bluetooth connection success");
                        if(this->charger_state.has_contact)
                        {
                                RCLCPP_INFO(this->get_logger(), "*** munual call service /charger/start service ***");
                                auto request2 = std::make_shared<std_srvs::srv::Empty::Request>();
                                auto result2 = client_start_charging->async_send_request(request2);
                        }
                        
                }
                else
                {
                       RCLCPP_INFO(this->get_logger(), "bluetooth connection failed"); 
                }
                bluetooth_connecting = false;
        }

        bool ManualDock::is_charging_position_moved(float x1, float y1, float yaw1, float x2, float y2, float yaw2)
        {
                bool ret = true;

                if(std::hypot(x1 - x2, y1 - y2) > 1.0)
                {
                        ret = true;
                }
                else
                {
                        float dist_yaw1 = std::abs(angles::shortest_angular_distance(yaw1, std::atan2(y2 - y1, x2 - x1)));
                        float dist_yaw2 = std::abs(angles::shortest_angular_distance(yaw1, yaw2));
                        if (dist_yaw1 < 0.5 && dist_yaw2 < 0.5)
                        {
                                ret = false;
                        }
                        else

                        {
                                ret = true;
                        }
                }


                return ret;
        }

        void ManualDock::odom_sub_callback(nav_msgs::msg::Odometry::SharedPtr msg)
        {
                tf2::Transform tf;
                tf2::convert(msg->pose.pose, tf);
                odom_current_x = tf.getOrigin().getX();
                odom_current_y = tf.getOrigin().getY();
                odom_current_yaw = tf2::getYaw(tf.getRotation());
        }


} // end of namespace


RCLCPP_COMPONENTS_REGISTER_NODE(capella_ros_dock::ManualDock)