#ifndef BOOSTER_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_
#define BOOSTER_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "booster_robot_driver/booster_topic_comm.hpp"

namespace booster_robot_driver
{

/**
 * @brief ROS2 Control hardware interface for Booster robots
 *
 * This hardware interface provides native ROS2 topic communication with Booster robots,
 * supporting both mock hardware mode for testing and real robot mode with safety features.
 *
 * Features:
 * - Direct topic communication via /joint_ctrl (commands) and /low_state (feedback)
 * - Direct communication with SDK (URDF and SDK joint orders are identical)
 * - Software joint limits with clamping and warnings from config file
 * - Safety: Synchronous wait for initial robot state during configuration
 * - Mock hardware support for development and testing
 *
 * Lifecycle:
 * - on_init: Validates joint configuration and loads parameters
 * - on_configure: Establishes connection and waits for initial robot state
 * - on_activate: Verifies readiness for operation
 * - read/write: Cyclic data exchange with robot
 *
 * Parameters:
 * - use_mock_hardware: Enable mock mode (default: false)
 */
class BoosterHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(BoosterHardwareInterface)

  BoosterHardwareInterface();
  virtual ~BoosterHardwareInterface();

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct JointLimits {
    double min_position;
    double max_position;
  };

  bool loadJointLimits();
  double clampToLimits(const std::string& joint_name, double command);
  std::unique_ptr<BoosterTopicComm> booster_comm_;
  rclcpp::Node::SharedPtr node_;

  bool use_mock_hardware_;
  bool initial_state_received_;

  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_states_positions_;
  std::vector<double> hw_states_velocities_;
  std::vector<double> hw_states_efforts_;

  std::vector<std::string> joint_names_;
  size_t num_joints_;

  std::map<std::string, JointLimits> joint_limits_;
};

}  // namespace booster_robot_driver

#endif  // BOOSTER_ROBOT_DRIVER__HARDWARE_INTERFACE_HPP_