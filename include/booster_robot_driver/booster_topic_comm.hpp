#ifndef BOOSTER_ROBOT_DRIVER__BOOSTER_TOPIC_COMM_HPP_
#define BOOSTER_ROBOT_DRIVER__BOOSTER_TOPIC_COMM_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <vector>
#include <memory>
#include <chrono>

#include "booster_interface/msg/low_cmd.hpp"
#include "booster_interface/msg/low_state.hpp"
#include "booster_interface/msg/motor_cmd.hpp"
#include "booster_interface/msg/motor_state.hpp"

namespace booster_robot_driver
{

/**
 * @brief Native ROS2 topic communication for Booster robots
 *
 * This class provides direct ROS2 topic communication with Booster robots
 * using standard ROS2 topics, eliminating external SDK dependencies.
 *
 * Communication Topics:
 * - /joint_ctrl (publishes LowCmd): Position commands to robot motors
 * - /low_state (subscribes LowState): Motor states from robot
 *
 * Data format:
 * - Uses serial motor array format (23 joints in SDK order)
 * - Position control mode with configurable PD gains
 * - Thread-safe state access via mutex protection
 */

struct JointState
{
  std::vector<double> positions;
  std::vector<double> velocities;
  std::vector<double> efforts;

  JointState() : positions(23, 0.0), velocities(23, 0.0), efforts(23, 0.0) {}
};

class BoosterTopicComm
{
public:
  explicit BoosterTopicComm(rclcpp::Node::SharedPtr node);
  ~BoosterTopicComm() = default;

  bool connect();
  bool disconnect();
  bool isConnected() const;

  bool sendJointCommands(const std::vector<double>& positions);
  bool getJointStates(JointState& joint_states);

private:
  rclcpp::Node::SharedPtr node_;

  // Publishers and subscribers
  rclcpp::Publisher<booster_interface::msg::LowCmd>::SharedPtr cmd_publisher_;
  rclcpp::Subscription<booster_interface::msg::LowState>::SharedPtr state_subscriber_;

  // Latest received state
  booster_interface::msg::LowState::SharedPtr latest_state_;
  std::mutex state_mutex_;

  // Connection state
  bool connected_;
  std::chrono::steady_clock::time_point last_state_time_;

  // State callback
  void stateCallback(const booster_interface::msg::LowState::SharedPtr msg);

  // Helper methods
  booster_interface::msg::LowCmd createLowCmd(const std::vector<double>& positions);
  bool extractJointStates(const booster_interface::msg::LowState& state, JointState& joint_states);
};

}  // namespace booster_robot_driver

#endif  // BOOSTER_ROBOT_DRIVER__BOOSTER_TOPIC_COMM_HPP_