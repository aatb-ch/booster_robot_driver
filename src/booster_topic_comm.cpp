#include "booster_robot_driver/booster_topic_comm.hpp"

#include <rclcpp/rclcpp.hpp>
#include <mutex>

namespace booster_robot_driver
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("BoosterTopicComm");

BoosterTopicComm::BoosterTopicComm(rclcpp::Node::SharedPtr node)
: node_(node), connected_(false)
{
}

bool BoosterTopicComm::connect()
{
  try {
    // Create QoS profile for reliable communication
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    // Create publisher for joint commands (/joint_ctrl)
    cmd_publisher_ = node_->create_publisher<booster_interface::msg::LowCmd>(
        "/joint_ctrl", qos);

    // Create subscriber for joint states (/low_state)
    state_subscriber_ = node_->create_subscription<booster_interface::msg::LowState>(
        "/low_state", qos,
        std::bind(&BoosterTopicComm::stateCallback, this, std::placeholders::_1));

    connected_ = true;
    last_state_time_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(LOGGER, "Connected to Booster robot via ROS2 topics");
    RCLCPP_INFO(LOGGER, "Publishing commands to: /joint_ctrl");
    RCLCPP_INFO(LOGGER, "Subscribing to states from: /low_state");

    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Failed to connect via topics: %s", e.what());
    return false;
  }
}

bool BoosterTopicComm::disconnect()
{
  connected_ = false;

  // Reset publishers and subscribers
  cmd_publisher_.reset();
  state_subscriber_.reset();

  RCLCPP_INFO(LOGGER, "Disconnected from Booster robot topics");
  return true;
}

bool BoosterTopicComm::isConnected() const
{
  if (!connected_) {
    return false;
  }

  // Check if we've received recent state data (within last 2 seconds)
  auto now = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_state_time_);

  // Consider connected if we have recent data OR if we just connected
  return duration.count() < 2000 || latest_state_ == nullptr;
}

bool BoosterTopicComm::sendJointCommands(const std::vector<double>& positions)
{
  if (!connected_ || !cmd_publisher_) {
    RCLCPP_ERROR(LOGGER, "Not connected - cannot send commands");
    return false;
  }

  if (positions.size() != 23) {
    RCLCPP_ERROR(LOGGER, "Invalid positions array size: %zu (expected 23)", positions.size());
    return false;
  }

  try {
    auto cmd_msg = createLowCmd(positions);
    cmd_publisher_->publish(cmd_msg);

    return true;

  } catch (const std::exception& e) {
    RCLCPP_ERROR(LOGGER, "Failed to send joint commands: %s", e.what());
    return false;
  }
}

bool BoosterTopicComm::getJointStates(JointState& joint_states)
{
  if (!connected_) {
    RCLCPP_ERROR(LOGGER, "Not connected - cannot get joint states");
    return false;
  }

  std::lock_guard<std::mutex> lock(state_mutex_);

  if (!latest_state_) {
    // No data received yet - return false to prevent dangerous zero position commands
    RCLCPP_WARN(LOGGER, "No joint state data received yet from /low_state topic");
    return false;
  }

  return extractJointStates(*latest_state_, joint_states);
}

void BoosterTopicComm::stateCallback(const booster_interface::msg::LowState::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  latest_state_ = msg;
  last_state_time_ = std::chrono::steady_clock::now();
}

booster_interface::msg::LowCmd BoosterTopicComm::createLowCmd(const std::vector<double>& positions)
{
  booster_interface::msg::LowCmd cmd;

  // Use serial mode for 23-joint array communication
  cmd.cmd_type = booster_interface::msg::LowCmd::CMD_TYPE_SERIAL;

  // Create motor commands for all 23 joints
  cmd.motor_cmd.resize(23);

  for (size_t i = 0; i < 23; ++i) {
    auto& motor_cmd = cmd.motor_cmd[i];

    // Position control mode
    motor_cmd.mode = 0;  // Position control mode
    motor_cmd.q = static_cast<float>(positions[i]);
    motor_cmd.dq = 0.0f;  // No velocity command
    motor_cmd.tau = 0.0f; // No torque command
    motor_cmd.kp = 20.0f; // Position gain
    motor_cmd.kd = 2.0f;  // Damping gain
    motor_cmd.weight = 1.0f; // Full weight
  }

  return cmd;
}

bool BoosterTopicComm::extractJointStates(const booster_interface::msg::LowState& state, JointState& joint_states)
{
  // Use serial motor states (23 joints in order)
  if (state.motor_state_serial.size() != 23) {
    RCLCPP_ERROR(LOGGER, "Invalid serial motor state size: %zu (expected 23)",
                 state.motor_state_serial.size());
    return false;
  }

  // Extract data from serial motor states
  for (size_t i = 0; i < 23; ++i) {
    const auto& motor_state = state.motor_state_serial[i];

    joint_states.positions[i] = static_cast<double>(motor_state.q);
    joint_states.velocities[i] = static_cast<double>(motor_state.dq);
    joint_states.efforts[i] = static_cast<double>(motor_state.tau_est);
  }

  return true;
}

}  // namespace booster_robot_driver