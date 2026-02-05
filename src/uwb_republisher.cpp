// Copyright 2026 Minh Nhan Gia
// Licensed under the MIT License

#include "tello_uwb/uwb_republisher.hpp"

#include <algorithm>
#include <cmath>
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

  // Filter parameters
  this->declare_parameter<bool>("enable_filtering", true);
  this->declare_parameter<bool>("enable_raw_publishing", false);  // Default off to save bandwidth
  this->declare_parameter<double>("max_velocity", 8.0);  // m/s, Tello max speed
  this->declare_parameter<double>("velocity_gate_timeout", 0.5);  // seconds
  this->declare_parameter<int>("max_consecutive_rejections", 5);  // force-accept after N rejections
  this->declare_parameter<int>("median_window_size", 10);
  this->declare_parameter<int>("min_samples_for_median", 3);

  // Get parameters
  frame_id_ = this->get_parameter("frame_id").as_string();
  expected_role_ = this->get_parameter("expected_role").as_int();
  std::string input_topic = this->get_parameter("input_topic").as_string();

  // Get filter parameters
  enable_filtering_ = this->get_parameter("enable_filtering").as_bool();
  enable_raw_publishing_ = this->get_parameter("enable_raw_publishing").as_bool();
  max_velocity_ = this->get_parameter("max_velocity").as_double();
  velocity_gate_timeout_ = this->get_parameter("velocity_gate_timeout").as_double();
  max_consecutive_rejections_ = this->get_parameter("max_consecutive_rejections").as_int();
  median_window_size_ = this->get_parameter("median_window_size").as_int();
  min_samples_for_median_ = this->get_parameter("min_samples_for_median").as_int();

  // Validate filter parameters
  if (max_consecutive_rejections_ < 1) {
    RCLCPP_WARN(this->get_logger(), "max_consecutive_rejections must be >= 1, setting to 5");
    max_consecutive_rejections_ = 5;
  }
  if (median_window_size_ < 1) {
    RCLCPP_WARN(this->get_logger(), "median_window_size must be >= 1, setting to 1");
    median_window_size_ = 1;
  }
  if (min_samples_for_median_ < 1 || min_samples_for_median_ > median_window_size_) {
    RCLCPP_WARN(this->get_logger(),
      "min_samples_for_median must be in [1, %d], clamping", median_window_size_);
    min_samples_for_median_ = std::clamp(min_samples_for_median_, 1, median_window_size_);
  }

  RCLCPP_INFO(this->get_logger(), "UWB Republisher initializing...");
  RCLCPP_INFO(this->get_logger(), "  Frame ID: %s", frame_id_.c_str());
  RCLCPP_INFO(this->get_logger(), "  Expected role: %d", expected_role_);
  RCLCPP_INFO(this->get_logger(), "  Input topic: %s", input_topic.c_str());
  RCLCPP_INFO(this->get_logger(), "  Filtering: %s", enable_filtering_ ? "enabled" : "disabled");
  RCLCPP_INFO(this->get_logger(), "  Raw publishing: %s", enable_raw_publishing_ ? "enabled" : "disabled");
  if (enable_filtering_) {
    RCLCPP_INFO(this->get_logger(), "    Max velocity: %.2f m/s", max_velocity_);
    RCLCPP_INFO(this->get_logger(), "    Velocity gate timeout: %.2f s", velocity_gate_timeout_);
    RCLCPP_INFO(this->get_logger(), "    Max consecutive rejections: %d", max_consecutive_rejections_);
    RCLCPP_INFO(this->get_logger(), "    Median window size: %d", median_window_size_);
    RCLCPP_INFO(this->get_logger(), "    Min samples for median: %d", min_samples_for_median_);
  }

  // Load drone configuration from parameters
  if (!loadDroneConfig()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load drone configuration!");
    throw std::runtime_error("Failed to load drone configuration");
  }

  // Create publishers for each drone
  createPublishers();

  // Create aggregated positions publisher
  positions_array_pub_ = this->create_publisher<tello_uwb::msg::DronePositionArray>(
    "/uwb/positions",
    rclcpp::SensorDataQoS());
  RCLCPP_INFO(this->get_logger(), "  Created aggregated publisher: /uwb/positions");

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
    // Filtered position publisher
    std::string topic_name = "/" + drone_name + "/uwb/position";
    position_publishers_[tag_id] = 
      this->create_publisher<geometry_msgs::msg::PointStamped>(
        topic_name, 
        rclcpp::SensorDataQoS());
    RCLCPP_INFO(this->get_logger(), "  Created publisher: %s", topic_name.c_str());

    // Raw position publisher (conditional - for comparison/debugging)
    if (enable_raw_publishing_) {
      std::string raw_topic_name = "/" + drone_name + "/uwb/position_raw";
      raw_position_publishers_[tag_id] = 
        this->create_publisher<geometry_msgs::msg::PointStamped>(
          raw_topic_name, 
          rclcpp::SensorDataQoS());
      RCLCPP_INFO(this->get_logger(), "  Created publisher: %s", raw_topic_name.c_str());
    }
  }
}

void UwbRepublisher::anchorFrameCallback(
  const nlink_parser::msg::LinktrackAnchorframe0::SharedPtr msg)
{
  rclcpp::Time stamp = this->now();

  // Prepare aggregated message for all drones
  tello_uwb::msg::DronePositionArray array_msg;
  array_msg.drones.reserve(tag_to_drone_map_.size());

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

    // Extract raw position from tag
    geometry_msgs::msg::Point raw_position;
    raw_position.x = static_cast<double>(tag.pos_3d[0]);
    raw_position.y = static_cast<double>(tag.pos_3d[1]);
    raw_position.z = static_cast<double>(tag.pos_3d[2]);

    // Publish raw position if enabled (for comparison/debugging)
    if (enable_raw_publishing_) {
      auto raw_pub_it = raw_position_publishers_.find(tag.id);
      if (raw_pub_it != raw_position_publishers_.end()) {
        geometry_msgs::msg::PointStamped raw_msg;
        raw_msg.header.stamp = stamp;
        raw_msg.header.frame_id = frame_id_;
        raw_msg.point = raw_position;
        raw_pub_it->second->publish(raw_msg);
      }
    }

    // Apply filtering if enabled
    geometry_msgs::msg::Point output_position;
    if (enable_filtering_) {
      // Step 1: Velocity gating
      if (!velocityGateAccept(tag.id, raw_position, stamp)) {
        // Position rejected by velocity gate - skip this sample
        continue;
      }

      // Step 2: Median filtering
      auto filtered_pos = applyMedianFilter(tag.id, raw_position);
      if (!filtered_pos.has_value()) {
        // Buffer not yet filled to minimum - skip output
        continue;
      }
      output_position = filtered_pos.value();
    } else {
      // Filtering disabled - use raw position
      output_position = raw_position;
    }

    // Create and publish the PointStamped message to individual topic
    geometry_msgs::msg::PointStamped point_msg;
    point_msg.header.stamp = stamp;
    point_msg.header.frame_id = frame_id_;
    point_msg.point = output_position;
    pub_it->second->publish(point_msg);

    // Add to aggregated message
    tello_uwb::msg::DronePosition drone_pos;
    drone_pos.drone_id = it->second;  // drone name
    drone_pos.uwb_tag_id = tag.id;    // numeric UWB tag ID
    drone_pos.position = point_msg;
    array_msg.drones.push_back(drone_pos);
  }

  // Publish aggregated message if we have any valid drone positions
  if (!array_msg.drones.empty()) {
    positions_array_pub_->publish(array_msg);
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

bool UwbRepublisher::velocityGateAccept(
  int tag_id,
  const geometry_msgs::msg::Point & position,
  const rclcpp::Time & stamp)
{
  auto & state = filter_states_[tag_id];
  state.total_samples++;

  // First sample - always accept and initialize
  if (!state.initialized) {
    state.initialized = true;
    state.last_accepted_time = stamp;
    state.last_accepted_position = position;
    state.consecutive_rejections = 0;
    return true;
  }

  // Calculate time delta
  double dt = (stamp - state.last_accepted_time).seconds();

  // Timeout check - if too long since last valid sample, reset the gate
  if (dt > velocity_gate_timeout_) {
    RCLCPP_DEBUG(this->get_logger(),
      "Velocity gate timeout for tag %d (%.3fs), resetting", tag_id, dt);
    state.last_accepted_time = stamp;
    state.last_accepted_position = position;
    state.consecutive_rejections = 0;
    state.position_buffer.clear();  // Stale buffer after timeout
    return true;
  }

  // Consecutive rejection check - force accept to prevent indefinite rejection spiral
  if (state.consecutive_rejections >= max_consecutive_rejections_) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Force-accepting position for tag %d after %d consecutive rejections "
      "(possible UWB offset or fast maneuver)",
      tag_id, state.consecutive_rejections);
    state.last_accepted_time = stamp;
    state.last_accepted_position = position;
    state.consecutive_rejections = 0;
    state.position_buffer.clear();  // Old buffer invalid after position jump
    // Note: This sample bypasses velocity check but will still go through median filter
    return true;
  }

  // Guard against division by zero (simultaneous samples)
  constexpr double kMinDt = 0.001;  // 1ms minimum
  if (dt < kMinDt) {
    dt = kMinDt;
  }

  // Calculate displacement (Euclidean distance 2D)
  double dx = position.x - state.last_accepted_position.x;
  double dy = position.y - state.last_accepted_position.y;
  double displacement = std::sqrt(dx * dx + dy * dy);

  // Calculate implied velocity
  double implied_velocity = displacement / dt;

  // Velocity gate check
  if (implied_velocity > max_velocity_) {
    state.rejected_samples++;
    state.consecutive_rejections++;
    RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
      "Velocity gate rejected tag %d: %.2f m/s (threshold: %.2f m/s), "
      "consecutive: %d, rejected %lu/%lu samples (%.1f%%)",
      tag_id, implied_velocity, max_velocity_,
      state.consecutive_rejections,
      state.rejected_samples, state.total_samples,
      100.0 * state.rejected_samples / state.total_samples);
    return false;
  }

  // Accept the sample - update state and reset consecutive rejection counter
  state.last_accepted_time = stamp;
  state.last_accepted_position = position;
  state.consecutive_rejections = 0;
  return true;
}

std::optional<geometry_msgs::msg::Point> UwbRepublisher::applyMedianFilter(
  int tag_id,
  const geometry_msgs::msg::Point & accepted_position)
{
  auto & state = filter_states_[tag_id];

  // Add to circular buffer
  state.position_buffer.push_back(accepted_position);

  // Maintain window size
  while (static_cast<int>(state.position_buffer.size()) > median_window_size_) {
    state.position_buffer.pop_front();
  }

  // Check if we have enough samples
  if (static_cast<int>(state.position_buffer.size()) < min_samples_for_median_) {
    return std::nullopt;
  }

  // Extract x, y, z components into separate vectors
  std::vector<double> x_values, y_values, z_values;
  x_values.reserve(state.position_buffer.size());
  y_values.reserve(state.position_buffer.size());
  z_values.reserve(state.position_buffer.size());

  for (const auto & pos : state.position_buffer) {
    x_values.push_back(pos.x);
    y_values.push_back(pos.y);
    z_values.push_back(pos.z);
  }

  // Compute median for each axis independently
  geometry_msgs::msg::Point filtered;
  filtered.x = computeMedian(x_values);
  filtered.y = computeMedian(y_values);
  filtered.z = computeMedian(z_values);

  return filtered;
}

double UwbRepublisher::computeMedian(std::vector<double> & values)
{
  if (values.empty()) {
    return 0.0;
  }

  size_t n = values.size();
  size_t mid = n / 2;

  // Use nth_element for O(n) average complexity
  std::nth_element(values.begin(), values.begin() + mid, values.end());

  if (n % 2 == 0) {
    // Even number of elements - average of two middle values
    double upper = values[mid];
    auto max_it = std::max_element(values.begin(), values.begin() + mid);
    return (*max_it + upper) / 2.0;
  } else {
    // Odd number of elements - middle value
    return values[mid];
  }
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
