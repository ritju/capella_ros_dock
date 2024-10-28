

#include "capella_ros_dock/hazards_vector_publisher.hpp"

#include <string>
#include <vector>

#include "rclcpp_components/register_node_macro.hpp"

namespace capella_ros_dock
{

HazardsVectorPublisher::HazardsVectorPublisher(const rclcpp::NodeOptions & options)
: rclcpp::Node("hazards_vector_publisher", options)
{
  RCLCPP_INFO(this->get_logger(), "hazard_detection_vector_node started.");
  
    this->declare_parameter<std::string>("publisher_topic", "/hazard_detection"); // Topic parameter to publish hazards vector to
    this->declare_parameter<std::vector<std::string>>("subscription_topics", std::vector<std::string>());   // Subscription topics parameter
    this->declare_parameter<int>("publish_rate", 30);   // Publish rate parameter in Hz

  
  publisher_topic_ = this->get_parameter_or<std::string>("publisher_topic", "");
  subscription_topics_ = this->get_parameter_or<std::vector<std::string>>("subscription_topics", std::vector<std::string>());
  const int publish_rate = this->get_parameter_or<int>("publish_rate", 30);
  
  RCLCPP_DEBUG(this->get_logger(), "publisher_topic: %s", publisher_topic_.c_str());
  RCLCPP_DEBUG(this->get_logger(), "publish_rate: %d", publish_rate);
  std::stringstream s;
  for(int i = 0; i < (int)subscription_topics_.size(); i++)
  {
    s << subscription_topics_[i] << " ";
  }
  RCLCPP_DEBUG_STREAM(this->get_logger(), "subscription_topics: " << s.str());

  publisher_ = create_publisher<capella_ros_dock_msgs::msg::HazardDetectionVector>(
    publisher_topic_, rclcpp::SensorDataQoS());
  RCLCPP_INFO_STREAM(get_logger(), "Advertised topic: " << publisher_topic_);

  timer_ = rclcpp::create_timer(
    this,
    this->get_clock(),
    rclcpp::Duration(std::chrono::duration<double>(1. / publish_rate)), [this]() {
      std::lock_guard<std::mutex> lock{this->mutex_};

      // Set header timestamp.
      this->msg_.header.stamp = now();

      // Publish detected vector.
      this->publisher_->publish(this->msg_);
      this->msg_.detections.clear();
    });

  // Set header frame_id.
  this->msg_.header.frame_id = "base_link";

  // Create subscriptions
  for (std::string topic : subscription_topics_) {
    subs_vector_.push_back(
      (create_subscription<capella_ros_dock_msgs::msg::HazardDetection>(
        topic, rclcpp::SensorDataQoS(),
        [this](const capella_ros_dock_msgs::msg::HazardDetection::SharedPtr msg) {
          std::lock_guard<std::mutex> lock{this->mutex_};
          this->msg_.detections.push_back(*msg);
        })));
    RCLCPP_INFO_STREAM(get_logger(), "Subscription to topic: " << topic);
  }
}

HazardsVectorPublisher::~HazardsVectorPublisher()
{  
}

}  // namespace capella_ros_dock

RCLCPP_COMPONENTS_REGISTER_NODE(capella_ros_dock::HazardsVectorPublisher)
