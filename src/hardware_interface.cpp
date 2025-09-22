#include "booster_robot_driver/hardware_interface.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <chrono>
#include <thread>
#include <limits>
#include <cmath>
#include <yaml-cpp/yaml.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

namespace booster_robot_driver
{

static const rclcpp::Logger LOGGER = rclcpp::get_logger("BoosterHardwareInterface");

BoosterHardwareInterface::BoosterHardwareInterface()
: hardware_interface::SystemInterface(),
  booster_comm_(nullptr),
  node_(nullptr),
  use_mock_hardware_(false),
  initial_state_received_(false)
{
}

BoosterHardwareInterface::~BoosterHardwareInterface()
{
  if (booster_comm_ && booster_comm_->isConnected())
  {
    booster_comm_->disconnect();
  }
}

hardware_interface::CallbackReturn BoosterHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  num_joints_ = info_.joints.size();

  if (num_joints_ == 0)
  {
    RCLCPP_ERROR(LOGGER, "No joints found in robot description");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize with NaN to indicate data is not yet valid
  // This is standard ros2_control practice to prevent controllers from using uninitialized data
  hw_states_positions_.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());

  for (size_t i = 0; i < num_joints_; i++)
  {
    joint_names_.push_back(info_.joints[i].name);

    if (info_.joints[i].command_interfaces.size() != 1)
    {
      RCLCPP_ERROR(LOGGER, "Joint '%s' has %zu command interfaces. 1 expected.",
                   info_.joints[i].name.c_str(),
                   info_.joints[i].command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.joints[i].command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(LOGGER, "Joint '%s' has '%s' command interface. Expected '%s'.",
                   info_.joints[i].name.c_str(),
                   info_.joints[i].command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (info_.joints[i].state_interfaces.size() != 3)
    {
      RCLCPP_ERROR(LOGGER, "Joint '%s' has %zu state interfaces. 3 expected.",
                   info_.joints[i].name.c_str(),
                   info_.joints[i].state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    const std::vector<std::string> expected_state_interfaces = {
      hardware_interface::HW_IF_POSITION,
      hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_EFFORT
    };

    for (size_t j = 0; j < expected_state_interfaces.size(); ++j)
    {
      if (info_.joints[i].state_interfaces[j].name != expected_state_interfaces[j])
      {
        RCLCPP_ERROR(LOGGER, "Joint '%s' state interface %zu has '%s'. Expected '%s'.",
                     info_.joints[i].name.c_str(), j,
                     info_.joints[i].state_interfaces[j].name.c_str(),
                     expected_state_interfaces[j].c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  // Get mock hardware parameter
  auto it = info_.hardware_parameters.find("use_mock_hardware");
  if (it != info_.hardware_parameters.end())
  {
    use_mock_hardware_ = (it->second == "true");
  }
  else
  {
    use_mock_hardware_ = false;
    RCLCPP_INFO(LOGGER, "No use_mock_hardware parameter found. Using real robot mode.");
  }

  // Note: SDK and URDF joint orders are identical, no mapping needed

  // Load joint limits for software safety
  if (!loadJointLimits())
  {
    RCLCPP_WARN(LOGGER, "Failed to load joint limits - continuing without software limits");
  }

  RCLCPP_INFO(LOGGER, "Successfully initialized Booster hardware interface with %zu joints",
              num_joints_);
  RCLCPP_INFO(LOGGER, "SDK and URDF joint orders are identical - no mapping needed");
  RCLCPP_INFO(LOGGER, "Mock hardware mode: %s", use_mock_hardware_ ? "ENABLED" : "DISABLED");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
BoosterHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < num_joints_; i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
BoosterHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < num_joints_; i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn BoosterHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Configuring hardware interface...");

  if (use_mock_hardware_)
  {
    // Mock mode: Initialize at zero positions without any hardware communication
    RCLCPP_INFO(LOGGER, "Mock mode: Initializing all joints at zero positions (no hardware communication)");
    std::fill(hw_states_positions_.begin(), hw_states_positions_.end(), 0.0);
    std::fill(hw_states_velocities_.begin(), hw_states_velocities_.end(), 0.0);
    std::fill(hw_states_efforts_.begin(), hw_states_efforts_.end(), 0.0);
    std::fill(hw_commands_positions_.begin(), hw_commands_positions_.end(), 0.0);
    initial_state_received_ = true;
  }
  else
  {
    // Real robot mode: Create communication and wait for initial state
    node_ = rclcpp::Node::make_shared("booster_hardware_interface_node");
    booster_comm_ = std::make_unique<BoosterTopicComm>(node_);

    if (!booster_comm_->connect())
    {
      RCLCPP_ERROR(LOGGER, "Failed to connect to robot topics");
      return hardware_interface::CallbackReturn::ERROR;
    }

    RCLCPP_INFO(LOGGER, "Real robot mode: Waiting for initial state from /low_state topic");

    auto timeout = std::chrono::seconds(10);
    auto start = std::chrono::steady_clock::now();

    while (!initial_state_received_ &&
           (std::chrono::steady_clock::now() - start) < timeout)
    {
      // Spin to process callbacks
      rclcpp::spin_some(node_);

      // Check if we received the first state
      JointState joint_states;
      if (booster_comm_->getJointStates(joint_states))
      {
        // SDK and URDF orders are identical - direct assignment
        hw_states_positions_ = joint_states.positions;
        hw_states_velocities_ = joint_states.velocities;
        hw_states_efforts_ = joint_states.efforts;

        // Initialize commands with current positions to prevent jumps
        hw_commands_positions_ = hw_states_positions_;

        initial_state_received_ = true;
        RCLCPP_INFO(LOGGER, "Received initial robot state - joints initialized from hardware");

        // Log the initial positions for debugging
        for (size_t i = 0; i < joint_names_.size() && i < 5; ++i) {
          RCLCPP_INFO(LOGGER, "  %s: state=%.3f, cmd=%.3f",
                      joint_names_[i].c_str(),
                      hw_states_positions_[i],
                      hw_commands_positions_[i]);
        }
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!initial_state_received_)
    {
      RCLCPP_ERROR(LOGGER, "Timeout waiting for initial robot state from /low_state topic");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(LOGGER, "Hardware interface configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BoosterHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Activating hardware interface...");

  if (use_mock_hardware_)
  {
    // Mock mode: Always ready, no hardware checks needed
    RCLCPP_INFO(LOGGER, "Hardware interface activated successfully (mock mode)");
    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // Real robot mode: Verify connection
  if (!booster_comm_ || !booster_comm_->isConnected())
  {
    RCLCPP_ERROR(LOGGER, "Hardware interface not properly configured");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!initial_state_received_)
  {
    RCLCPP_ERROR(LOGGER, "Initial state not received during configuration");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(LOGGER, "Hardware interface activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BoosterHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(LOGGER, "Deactivating hardware interface...");

  // Only disconnect if we're in real robot mode and connected
  if (!use_mock_hardware_ && booster_comm_ && booster_comm_->isConnected())
  {
    booster_comm_->disconnect();
  }

  RCLCPP_INFO(LOGGER, "Hardware interface deactivated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type BoosterHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (use_mock_hardware_)
  {
    // Mock mode: Simple loopback - states follow commands
    // No hardware communication at all
    hw_states_positions_ = hw_commands_positions_;
    std::fill(hw_states_velocities_.begin(), hw_states_velocities_.end(), 0.0);
    std::fill(hw_states_efforts_.begin(), hw_states_efforts_.end(), 0.0);
    return hardware_interface::return_type::OK;
  }

  // Real robot mode: Process callbacks and read from hardware
  if (node_) {
    rclcpp::spin_some(node_);
  }

  if (!booster_comm_->isConnected())
  {
    RCLCPP_ERROR(LOGGER, "Robot connection lost");
    return hardware_interface::return_type::ERROR;
  }

  JointState joint_states;
  if (!booster_comm_->getJointStates(joint_states))
  {
    RCLCPP_ERROR(LOGGER, "Failed to read joint states from robot");
    return hardware_interface::return_type::ERROR;
  }

  // SDK and URDF orders are identical - direct assignment
  hw_states_positions_ = joint_states.positions;
  hw_states_velocities_ = joint_states.velocities;
  hw_states_efforts_ = joint_states.efforts;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BoosterHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  static bool first_write = true;

  if (use_mock_hardware_)
  {
    // Mock mode: Do nothing - the loopback happens in read()
    // No hardware communication at all
    return hardware_interface::return_type::OK;
  }

  // Real robot mode: Send commands to hardware
  if (!booster_comm_->isConnected())
  {
    RCLCPP_ERROR(LOGGER, "Robot connection lost");
    return hardware_interface::return_type::ERROR;
  }

  // Safety check: Don't send NaN commands (indicates uninitialized data)
  for (size_t i = 0; i < hw_commands_positions_.size(); ++i) {
    if (std::isnan(hw_commands_positions_[i])) {
      if (first_write) {
        RCLCPP_WARN(LOGGER, "Commands contain NaN values - using current state positions instead");
        hw_commands_positions_ = hw_states_positions_;
        break;
      } else {
        RCLCPP_ERROR(LOGGER, "Commands contain NaN values after initialization!");
        return hardware_interface::return_type::ERROR;
      }
    }
  }

  // Log first write for debugging
  if (first_write) {
    RCLCPP_INFO(LOGGER, "First write() call - sending initial commands:");
    for (size_t i = 0; i < joint_names_.size() && i < 5; ++i) {
      RCLCPP_INFO(LOGGER, "  %s: cmd=%.3f, state=%.3f",
                  joint_names_[i].c_str(),
                  hw_commands_positions_[i],
                  hw_states_positions_[i]);
    }
    first_write = false;
  }

  // Apply joint limits clamping to commands
  std::vector<double> clamped_commands = hw_commands_positions_;
  for (size_t i = 0; i < num_joints_; ++i)
  {
    // clamped_commands[i] = clampToLimits(joint_names_[i], hw_commands_positions_[i]);
  }

  // SDK and URDF orders are identical - direct assignment
  std::vector<double> serial_commands = clamped_commands;

  if (!booster_comm_->sendJointCommands(serial_commands))
  {
    RCLCPP_ERROR(LOGGER, "Failed to send joint commands");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool BoosterHardwareInterface::loadJointLimits()
{
  try
  {
    // Get the path to the joint limits YAML file
    std::string package_share_dir = ament_index_cpp::get_package_share_directory("booster_robot_driver");
    std::string limits_file = package_share_dir + "/config/t1_joint_limits.yaml";

    // Load the YAML file
    YAML::Node config = YAML::LoadFile(limits_file);

    if (!config["joint_limits"])
    {
      RCLCPP_ERROR(LOGGER, "No 'joint_limits' section found in %s", limits_file.c_str());
      return false;
    }

    YAML::Node joint_limits_config = config["joint_limits"];

    // Parse joint limits for each joint
    for (const auto& joint_name : joint_names_)
    {
      if (joint_limits_config[joint_name])
      {
        JointLimits limits;
        limits.min_position = joint_limits_config[joint_name]["min_position"].as<double>();
        limits.max_position = joint_limits_config[joint_name]["max_position"].as<double>();

        joint_limits_[joint_name] = limits;

        RCLCPP_DEBUG(LOGGER, "Loaded limits for joint %s: min=%.3f, max=%.3f",
                    joint_name.c_str(), limits.min_position, limits.max_position);
      }
      else
      {
        RCLCPP_WARN(LOGGER, "No limits found for joint: %s", joint_name.c_str());
      }
    }

    RCLCPP_INFO(LOGGER, "Successfully loaded joint limits for %zu joints", joint_limits_.size());
    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(LOGGER, "Failed to load joint limits: %s", e.what());
    return false;
  }
}

double BoosterHardwareInterface::clampToLimits(const std::string& joint_name, double command)
{
  // If no limits are defined for this joint, return the command as-is
  auto it = joint_limits_.find(joint_name);
  if (it == joint_limits_.end())
  {
    return command;
  }

  const JointLimits& limits = it->second;
  double original_command = command;

  // Clamp the command to the joint limits
  command = std::max(limits.min_position, std::min(limits.max_position, command));

  // Warn if the command was clamped
  if (std::abs(original_command - command) > 1e-6)
  {
    static std::map<std::string, int> warning_counts;
    static const int WARNING_THROTTLE = 100; // Warn every ~1 second at 100Hz

    if (++warning_counts[joint_name] % WARNING_THROTTLE == 0)
    {
      RCLCPP_WARN(LOGGER, "Joint '%s' command %.3f rad clamped to %.3f rad (limits: %.3f to %.3f)",
                  joint_name.c_str(), original_command, command,
                  limits.min_position, limits.max_position);
    }
  }

  return command;
}

}  // namespace booster_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(booster_robot_driver::BoosterHardwareInterface,
                       hardware_interface::SystemInterface)