// Copyright 2026 Minh Nhan Gia
// Licensed under the MIT License

#include "tello_uwb/uwb_republisher.hpp"

#include <algorithm>
#include <stdexcept>

namespace tello_uwb
{

UwbRepublisher::UwbRepublisher(const rclcpp::NodeOptions & options)
: Node("uwb_republisher", options)
{
  // Declare parameters
  this->declare_parameter<std::string>("frame_id", "world");
  this->declare_parameter<int>("expected_role", 2);
  this->declare_parameter<std::string>("input_topic", "/nlink_linktrack_anchorframe0");
  this->declare_parameter<std::vector<std::string>>("drone_names", std::vector<std::string>{});
  this->declare_parameter<std::vector<int64_t>>("uwb_tag_ids", std::vector<int64_t>{});

  // Get parameters
  frame_id_ = this->get_parameter("frame_id").as_string();
  expected_role_ = this->get_parameter("expected_role").as_int();
  std::string input_topic = this->get_parameter("input_topic").as_string();

  RCLCPP_INFO(this->get_logger(), "UWB Republisher initializing...");
  RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Expected role: %d", expected_role_);
  RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic.c_str());

  // Load drone configuration from parameters
  if (!loadDroneConfig()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load drone configuration!");
    throw std::runtime_error("Failed to load drone configuration");
  }

  // Create publishers for each drone
  createPublishers();

  // Subscribe to LinkTrack anchor frame data
  anchor_sub_ = this->create_subscription<nlink_parser::msg::LinktrackAnchorframe0>(
    input_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&UwbRepublisher::anchorFrameCallback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "UWB Republisher initialized successfully");
}

bool UwbRepublisher::loadDroneConfig()
{
  std::vector<std::string> drone_names = 
    this->get_parameter("drone_names").as_string_array();
  std::vector<int64_t> uwb_tag_ids = 
    this->get_parameter("uwb_tag_ids").as_integer_array();

  if (drone_names.empty() || uwb_tag_ids.empty()) {
    RCLCPP_ERROR(this->get_logger(), 
      "drone_names and uwb_tag_ids parameters must be provided and non-empty");
    return false;
  }

  if (drone_names.size() != uwb_tag_ids.size()) {
    RCLCPP_ERROR(this->get_logger(),
      "drone_names size (%zu) must match uwb_tag_ids size (%zu)",
      drone_names.size(), uwb_tag_ids.size());
    return false;
  }

  // Build the tag_id to drone_name mapping
  for (size_t i = 0; i < drone_names.size(); ++i) {
    int tag_id = static_cast<int>(uwb_tag_ids[i]);
    
    // Check for duplicate tag IDs
    if (tag_to_drone_map_.find(tag_id) != tag_to_drone_map_.end()) {
      RCLCPP_ERROR(this->get_logger(),
        "Duplicate UWB tag ID %d found in configuration", tag_id);
      return false;
    }

    tag_to_drone_map_[tag_id] = drone_names[i];
    RCLCPP_INFO(this->get_logger(), 
      "  Mapped UWB tag ID %d -> drone '%s'", tag_id, drone_names[i].c_str());
  }

  RCLCPP_INFO(this->get_logger(), "Loaded configuration for %zu drones", 
    tag_to_drone_map_.size());
  return true;
}

void UwbRepublisher::createPublishers()
{
  for (const auto & [tag_id, drone_name] : tag_to_drone_map_) {
    std::string topic_name = "/" + drone_name + "/uwb/position";
    
    position_publishers_[tag_id] = 
      this->create_publisher<geometry_msgs::msg::PointStamped>(
        topic_name, 
        rclcpp::SensorDataQoS());

    RCLCPP_INFO(this->get_logger(), "  Created publisher: %s", topic_name.c_str());
  }
}

void UwbRepublisher::anchorFrameCallback(
  const nlink_parser::msg::LinktrackAnchorframe0::SharedPtr msg)
{
  rclcpp::Time stamp = this->now();

  for (const auto & tag : msg->nodes) {
    // Filter by expected role (typically role 2 for tags)
    if (tag.role != expected_role_) {
      continue;
    }

    // Check if this tag ID is configured
    auto it = tag_to_drone_map_.find(tag.id);
    if (it == tag_to_drone_map_.end()) {
      RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Received data for unconfigured UWB tag ID %d", tag.id);
      continue;
    }

    // Find the publisher for this tag
    auto pub_it = position_publishers_.find(tag.id);
    if (pub_it == position_publishers_.end()) {
      RCLCPP_WARN_ONCE(this->get_logger(),
        "No publisher found for UWB tag ID %d", tag.id);
      continue;
    }

    // Create and publish the PointStamped message
    auto point_msg = createPointStamped(tag, stamp);
    pub_it->second->publish(point_msg);
  }
}

geometry_msgs::msg::PointStamped UwbRepublisher::createPointStamped(
  const nlink_parser::msg::LinktrackTag & tag,
  const rclcpp::Time & stamp) const
{
  geometry_msgs::msg::PointStamped point_msg;
  
  point_msg.header.stamp = stamp;
  point_msg.header.frame_id = frame_id_;
  
  // pos_3d is a float32[3] array: [x, y, z]
  point_msg.point.x = static_cast<double>(tag.pos_3d[0]);
  point_msg.point.y = static_cast<double>(tag.pos_3d[1]);
  point_msg.point.z = static_cast<double>(tag.pos_3d[2]);

  return point_msg;
}

}  // namespace tello_uwb

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(tello_uwb::UwbRepublisher)

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<tello_uwb::UwbRepublisher>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("uwb_republisher"), 
      "Exception during node execution: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}
