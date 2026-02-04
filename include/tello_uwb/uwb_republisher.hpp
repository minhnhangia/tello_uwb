// Copyright 2026 Minh Nhan Gia
// Licensed under the MIT License

#ifndef TELLO_UWB__UWB_REPUBLISHER_HPP_
#define TELLO_UWB__UWB_REPUBLISHER_HPP_

#include <deque>
#include <memory>
#include <optional>
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
 * @brief Per-drone filter state for velocity gating and median filtering
 */
struct DroneFilterState
{
  /// Whether the filter has been initialized with a valid sample
  bool initialized{false};

  /// Timestamp of the last accepted position (for velocity gating)
  rclcpp::Time last_accepted_time{0, 0, RCL_ROS_TIME};

  /// Last accepted position (for velocity gating)
  geometry_msgs::msg::Point last_accepted_position;

  /// Circular buffer of accepted positions for median filtering
  std::deque<geometry_msgs::msg::Point> position_buffer;

  /// Counter for consecutive rejections (reset on accept)
  int consecutive_rejections{0};

  /// Statistics: total samples received
  uint64_t total_samples{0};

  /// Statistics: samples rejected by velocity gate
  uint64_t rejected_samples{0};
};

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

  /**
   * @brief Apply velocity gating to reject physically impossible position jumps
   * @param tag_id UWB tag identifier
   * @param position Current position measurement
   * @param stamp Current timestamp
   * @return true if position passes the velocity gate, false if rejected
   */
  bool velocityGateAccept(
    int tag_id,
    const geometry_msgs::msg::Point & position,
    const rclcpp::Time & stamp);

  /**
   * @brief Apply moving median filter to smooth accepted positions
   * @param tag_id UWB tag identifier
   * @param accepted_position Position that passed velocity gating
   * @return Filtered position, or nullopt if buffer not yet filled to minimum
   */
  std::optional<geometry_msgs::msg::Point> applyMedianFilter(
    int tag_id,
    const geometry_msgs::msg::Point & accepted_position);

  /**
   * @brief Compute median of a vector of values (modifies input vector)
   * @param values Vector of values (will be partially sorted)
   * @return Median value
   */
  static double computeMedian(std::vector<double> & values);

  // Parameters
  std::string frame_id_;
  int expected_role_;

  // Filter parameters
  double max_velocity_;             ///< Velocity gate threshold (m/s)
  double velocity_gate_timeout_;    ///< Reset gate after this duration of no valid data (s)
  int max_consecutive_rejections_;  ///< Force-accept after this many consecutive rejections
  int median_window_size_;          ///< Number of samples for median filter
  int min_samples_for_median_;      ///< Minimum samples before outputting median
  bool enable_filtering_;           ///< Master switch for filtering

  // Per-drone filter state
  std::unordered_map<int, DroneFilterState> filter_states_;

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