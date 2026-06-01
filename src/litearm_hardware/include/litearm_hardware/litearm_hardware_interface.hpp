#ifndef LITEARM_HARDWARE__LITEARM_HARDWARE_INTERFACE_HPP_
#define LITEARM_HARDWARE__LITEARM_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

// Forward declaration
namespace litearm_robot {
class LiteArm;
}

namespace litearm_hardware
{
class LiteArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(LiteArmHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // LiteArm robot instance
  std::unique_ptr<litearm_robot::LiteArm> robot_;

  // Configuration
  std::string config_file_;

  // Joint states
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Joint commands
  std::vector<double> hw_commands_positions_;
  std::vector<double> hw_commands_velocities_;
  std::vector<double> hw_commands_efforts_;

  // Control parameters
  std::vector<double> max_torques_;
  std::vector<double> max_velocities_;
  std::vector<double> kp_gains_;
  std::vector<double> kd_gains_;

  // Per-joint direction sign (1.0 = normal, -1.0 = reversed)
  std::vector<double> joint_sign_;

  // Control mode
  std::string control_mode_;  // "position_velocity", "pd_control", or "full_control"

  // Gripper conversion: radians to meters
  double gripper_rad_to_m_;  // Conversion factor for gripper position (rad to m)

  // Use velocity and effort commands
  bool use_velocity_commands_;
  bool use_effort_commands_;

  // Persistent clock for throttled logging (avoid memory issues)
  std::shared_ptr<rclcpp::Clock> clock_;

  // Gravity compensation using Pinocchio RNEA
  bool use_gravity_compensation_;
  std::string gravity_compensation_urdf_;
  double gravity_gain_;               // global gravity compensation gain
  pinocchio::Model pin_model_;
  std::unique_ptr<pinocchio::Data> pin_data_;
  int pin_joint_offset_;              // 0 for left arm, 9 for right arm


};


}  // namespace litearm_hardware

#endif  // LITEARM_HARDWARE__LITEARM_HARDWARE_INTERFACE_HPP_
