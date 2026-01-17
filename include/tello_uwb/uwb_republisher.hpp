// Copyright 2026 Minh Nhan Gia
// Licensed under the MIT License

#ifndef TELLO_UWB__UWB_REPUBLISHER_HPP_
#define TELLO_UWB__UWB_REPUBLISHER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nlink_parser/msg/linktrack_anchorframe0.hpp"
#include "tello_uwb/msg/drone_position.hpp"
#include "tello_uwb/msg/drone_position_array.hpp"

namespace tello_uwb
{

/**
 * @brief ROS2 Node that republishes UWB position data from LinkTrack system
 *        to individual drone topics as PointStamped messages.
 *
 * This node subscribes to the aggregated UWB data from the LinkTrack anchor
 * and republishes individual tag positions to their respective drone topics
 * based on the configuration provided via YAML file.
 */
class UwbRepublisher : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new UwbRepublisher node
   * @param options Node options for configuration
   */
  explicit UwbRepublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor
   */
  ~UwbRepublisher() override = default;

private:
  /**
   * @brief Load drone configurations from parameters
   * @return true if configuration loaded successfully, false otherwise
   */
  bool loadDroneConfig();

  /**
   * @brief Create publishers for each configured drone
   */
  void createPublishers();

  /**
   * @brief Callback for incoming LinkTrack anchor frame data
   * @param msg The incoming anchor frame message
   */
  void anchorFrameCallback(const nlink_parser::msg::LinktrackAnchorframe0::SharedPtr msg);

  /**
   * @brief Convert LinktrackTag position to PointStamped message
   * @param tag The LinkTrack tag data
   * @param stamp Timestamp for the message
   * @return PointStamped message with position data
   */
  geometry_msgs::msg::PointStamped createPointStamped(
    const nlink_parser::msg::LinktrackTag & tag,
    const rclcpp::Time & stamp) const;

  // Parameters
  std::string frame_id_;
  int expected_role_;

  // Drone configurations: maps uwb_tag_id -> drone_name
  std::unordered_map<int, std::string> tag_to_drone_map_;

  // Maps uwb_tag_id -> publisher
  std::unordered_map<int, rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr>
    position_publishers_;

  // Aggregated publisher for all drone positions
  rclcpp::Publisher<tello_uwb::msg::DronePositionArray>::SharedPtr positions_array_pub_;

  // Subscription to LinkTrack data
  rclcpp::Subscription<nlink_parser::msg::LinktrackAnchorframe0>::SharedPtr anchor_sub_;
};

}  // namespace tello_uwb

#endif  // TELLO_UWB__UWB_REPUBLISHER_HPP_