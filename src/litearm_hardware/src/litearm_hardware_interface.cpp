#include "litearm_hardware/litearm_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "litearm_robot/LiteArm.hpp"
#include <rclcpp/clock.hpp>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace litearm_hardware
{

hardware_interface::CallbackReturn LiteArmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get configuration file path
  if (info_.hardware_parameters.find("config_file") != info_.hardware_parameters.end())
  {
    config_file_ = info_.hardware_parameters["config_file"];
    RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"),
                "Config file: %s", config_file_.c_str());
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("LiteArmHardwareInterface"),
                 "Parameter 'config_file' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get control mode (default: position_velocity)
  if (info_.hardware_parameters.find("control_mode") != info_.hardware_parameters.end())
  {
    control_mode_ = info_.hardware_parameters["control_mode"];
  }
  else
  {
    control_mode_ = "position_velocity";
  }
  RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"),
              "Control mode: %s", control_mode_.c_str());

  // Initialize clock for throttled logging
  clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);

  // Get gripper rad to meter conversion factor
  if (info_.hardware_parameters.find("gripper_rad_to_m") != info_.hardware_parameters.end())
  {
    gripper_rad_to_m_ = std::stod(info_.hardware_parameters["gripper_rad_to_m"]);
  }
  else
  {
    gripper_rad_to_m_ = 0.01;  // Default: 1 radian = 0.01 meter
  }
  RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"),
              "Gripper rad to meter conversion: %.4f", gripper_rad_to_m_);

  // Check if velocity and effort commands are enabled
  use_velocity_commands_ = (control_mode_ == "full_control" || control_mode_ == "position_velocity");
  use_effort_commands_ = (control_mode_ == "full_control");

  // Initialize state and command storage
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_efforts_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_efforts_.resize(info_.joints.size(), 0.0);

  // Initialize control parameters
  max_torques_.resize(info_.joints.size(), 0.0);
  max_velocities_.resize(info_.joints.size(), 0.5);
  kp_gains_.resize(info_.joints.size(), 0.0);
  kd_gains_.resize(info_.joints.size(), 0.0);

  // Load joint-specific parameters
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Max torque
    if (info_.joints[i].parameters.find("max_torque") != info_.joints[i].parameters.end())
    {
      max_torques_[i] = std::stod(info_.joints[i].parameters.at("max_torque"));
    }
    else
    {
      max_torques_[i] = 10.0;  // Default value
    }

    // Max velocity
    if (info_.joints[i].parameters.find("max_velocity") != info_.joints[i].parameters.end())
    {
      max_velocities_[i] = std::stod(info_.joints[i].parameters.at("max_velocity"));
    }
    else
    {
      max_velocities_[i] = 0.5;  // Default value
    }

    // PD gains
    if (info_.joints[i].parameters.find("kp") != info_.joints[i].parameters.end())
    {
      kp_gains_[i] = std::stod(info_.joints[i].parameters.at("kp"));
    }
    else
    {
      kp_gains_[i] = 4.0;  // Default value
    }

    if (info_.joints[i].parameters.find("kd") != info_.joints[i].parameters.end())
    {
      kd_gains_[i] = std::stod(info_.joints[i].parameters.at("kd"));
    }
    else
    {
      kd_gains_[i] = 0.5;  // Default value
    }

    RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"),
                "Joint %s: max_torque=%.2f, max_velocity=%.2f, kp=%.2f, kd=%.2f",
                info_.joints[i].name.c_str(), max_torques_[i], max_velocities_[i],
                kp_gains_[i], kd_gains_[i]);
    }

  // Initialize joint direction signs (default: normal)
  joint_sign_.resize(info_.joints.size(), 1.0);

  // Load per-joint reverse parameter
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    if (info_.joints[i].parameters.find("reverse") != info_.joints[i].parameters.end() &&
        info_.joints[i].parameters.at("reverse") == "true")
    {
      joint_sign_[i] = -1.0;
      RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"),
                  "Joint %s: direction REVERSED", info_.joints[i].name.c_str());
    }
  }

  // Gravity compensation setup
  use_gravity_compensation_ = false;
  gravity_gain_ = 0.85;
  pin_joint_offset_ = 0;
  if (info_.hardware_parameters.find("gravity_compensation_urdf") != info_.hardware_parameters.end())
  {
    gravity_compensation_urdf_ = info_.hardware_parameters["gravity_compensation_urdf"];
    use_gravity_compensation_ = true;

    if (info_.hardware_parameters.find("gravity_gain") != info_.hardware_parameters.end())
    {
      gravity_gain_ = std::stod(info_.hardware_parameters["gravity_gain"]);
    }

    // Determine arm side from joint name prefix (l_ -> left=0, r_ -> right=9)
    if (!info_.joints.empty())
    {
      if (info_.joints[0].name.find("r_") == 0)
        pin_joint_offset_ = 9;
      else
        pin_joint_offset_ = 0;
    }

    RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"),
                "Gravity compensation enabled: URDF=%s, offset=%d, gain=%.2f",
                gravity_compensation_urdf_.c_str(), pin_joint_offset_, gravity_gain_);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LiteArmHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"), "Configuring...");

  // Initialize LiteArm robot
  try
  {
    RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"),
                "Initializing LiteArm robot with config: %s", config_file_.c_str());

    robot_ = std::make_unique<litearm_robot::LiteArm>(config_file_);

    RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"),
                "LiteArm robot initialized successfully");
  }
  catch (const std::bad_alloc & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("LiteArmHardwareInterface"),
                 "Memory allocation failed during LiteArm initialization: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("LiteArmHardwareInterface"),
                 "Failed to initialize LiteArm robot: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  catch (...)
  {
    RCLCPP_ERROR(rclcpp::get_logger("LiteArmHardwareInterface"),
                 "Unknown exception during LiteArm initialization");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read initial joint states
  try
  {
    robot_->send_get_motor_state_cmd();
    robot_->motor_send_cmd();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // Read 7 arm joint states
    auto positions = robot_->getCurrentPos();
    auto velocities = robot_->getCurrentVel();
    auto torques = robot_->getCurrentTorque();

    for (size_t i = 0; i < 7; i++)
    {
      hw_positions_[i] = positions[i] * joint_sign_[i];
      hw_velocities_[i] = velocities[i] * joint_sign_[i];
      hw_efforts_[i] = torques[i] * joint_sign_[i];
      hw_commands_positions_[i] = positions[i] * joint_sign_[i];  // Initialize commands to current position
    }

    // Read gripper (8th joint, index 7, the controlled finger) if present
    // Convert from radians to meters for prismatic joint
    if (info_.joints.size() > 7)
    {
      double gripper_rad = robot_->getCurrentPosGripper();
      hw_positions_[7] = gripper_rad * gripper_rad_to_m_;
      hw_velocities_[7] = robot_->getCurrentVelGripper() * gripper_rad_to_m_;
      hw_efforts_[7] = robot_->getCurrentTorqueGripper();
      hw_commands_positions_[7] = hw_positions_[7];  // Initialize commands to current position
    }

    // Finger mimic joint (9th joint, index 8) mirrors the controlled finger (index 7)
    if (info_.joints.size() > 8)
    {
      hw_positions_[8] = -hw_positions_[7];  // Mimic position (negated, opposite direction)
      hw_velocities_[8] = -hw_velocities_[7];  // Mimic velocity (negated)
      hw_efforts_[8] = 0.0;  // Passive joint, no actuator
      hw_commands_positions_[8] = hw_positions_[8];
    }

    RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"),
                "Initial joint states read successfully");
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("LiteArmHardwareInterface"),
                 "Failed to read initial joint states: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Load Pinocchio model for gravity compensation
  if (use_gravity_compensation_)
  {
    try
    {
      pinocchio::urdf::buildModel(gravity_compensation_urdf_, pin_model_);
      pin_data_ = std::make_unique<pinocchio::Data>(pin_model_);
      RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"),
                  "Pinocchio model loaded: nq=%d, nv=%d, njoints=%d",
                  (int)pin_model_.nq, (int)pin_model_.nv, (int)pin_model_.njoints);
    }
    catch (const std::exception & e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("LiteArmHardwareInterface"),
                   "Failed to load Pinocchio model from %s: %s",
                   gravity_compensation_urdf_.c_str(), e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
LiteArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
LiteArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Skip command interfaces for mimic joints (joints defined without command_interfaces in URDF)
    if (info_.joints[i].command_interfaces.empty())
    {
      continue;
    }

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));

    if (use_velocity_commands_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
    }

    if (use_effort_commands_)
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_efforts_[i]));
    }
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn LiteArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"), "Activating...");

  // Read current state and set as command
  // Retry multiple times to wait for motor state to be received
  const int max_retries = 20;
  const int retry_delay_ms = 50;
  bool got_valid_state = false;

  try
  {
    for (int retry = 0; retry < max_retries; retry++)
    {
      robot_->send_get_motor_state_cmd();
      robot_->motor_send_cmd();
      std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));

      // Read 7 arm joint positions and set both hw_positions_ and hw_commands_positions_
      auto positions = robot_->getCurrentPos();
      auto velocities = robot_->getCurrentVel();
      auto torques = robot_->getCurrentTorque();

      // Check if we got valid data (not the initialization 999.0 value)
      got_valid_state = true;
      for (size_t i = 0; i < 7; i++)
      {
        if (std::abs(positions[i]) > 100.0)  // 999.0 is invalid
        {
          got_valid_state = false;
          break;
        }
      }

      if (got_valid_state)
      {
        for (size_t i = 0; i < 7; i++)
        {
          hw_positions_[i] = positions[i] * joint_sign_[i];
          hw_velocities_[i] = velocities[i] * joint_sign_[i];
          hw_efforts_[i] = torques[i] * joint_sign_[i];
          hw_commands_positions_[i] = positions[i] * joint_sign_[i];
        }

        // Read gripper position (8th joint, index 7, controlled finger) if present
        if (info_.joints.size() > 7)
        {
          double gripper_rad = robot_->getCurrentPosGripper();
          hw_positions_[7] = gripper_rad * gripper_rad_to_m_;
          hw_velocities_[7] = robot_->getCurrentVelGripper() * gripper_rad_to_m_;
          hw_efforts_[7] = robot_->getCurrentTorqueGripper();
          hw_commands_positions_[7] = gripper_rad * gripper_rad_to_m_;
        }

        // Finger mimic joint (9th joint, index 8) mirrors the controlled finger
        if (info_.joints.size() > 8)
        {
          hw_positions_[8] = -hw_positions_[7];
          hw_velocities_[8] = -hw_velocities_[7];
          hw_efforts_[8] = 0.0;
          hw_commands_positions_[8] = -hw_commands_positions_[7];
        }

        RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"),
                    "Hardware activated successfully. Initial positions: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                    positions[0], positions[1], positions[2], positions[3], positions[4], positions[5], positions[6]);
        break;
      }
    }

    if (!got_valid_state)
    {
      RCLCPP_WARN(rclcpp::get_logger("LiteArmHardwareInterface"),
                  "Could not get valid motor state after %d retries. Using default positions.",
                  max_retries);
      // Initialize with zeros as fallback
      for (size_t i = 0; i < 7; i++)
      {
        hw_positions_[i] = 0.0;
        hw_velocities_[i] = 0.0;
        hw_efforts_[i] = 0.0;
        hw_commands_positions_[i] = 0.0;
      }
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("LiteArmHardwareInterface"),
                 "Failed to activate hardware: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn LiteArmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"), "Deactivating...");

  // Optionally send stop command or hold position
  // For safety, we'll just log the deactivation

  RCLCPP_INFO(rclcpp::get_logger("LiteArmHardwareInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LiteArmHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read joint states from hardware
  try
  {
    robot_->send_get_motor_state_cmd();
    robot_->motor_send_cmd();

    // Small delay to allow response
    std::this_thread::sleep_for(std::chrono::milliseconds(1));

    // Read 7 arm joint states
    auto positions = robot_->getCurrentPos();
    auto velocities = robot_->getCurrentVel();
    auto torques = robot_->getCurrentTorque();

    for (size_t i = 0; i < 7; i++)
    {
      // Only update if we got valid data (not the initialization 999.0 value)
      if (std::abs(positions[i]) <= 100.0)
      {
        hw_positions_[i] = positions[i] * joint_sign_[i];
      }
      hw_velocities_[i] = velocities[i] * joint_sign_[i];
      hw_efforts_[i] = torques[i] * joint_sign_[i];
    }

    // Read gripper state (8th joint, index 7, the controlled finger)
    // Convert from radians to meters for prismatic joint
    if (info_.joints.size() > 7)
    {
      double gripper_rad = robot_->getCurrentPosGripper();
      if (std::abs(gripper_rad) <= 100.0)
      {
        hw_positions_[7] = gripper_rad * gripper_rad_to_m_;
      }
      hw_velocities_[7] = robot_->getCurrentVelGripper() * gripper_rad_to_m_;
      hw_efforts_[7] = robot_->getCurrentTorqueGripper();
    }

    // Finger mimic joint (9th joint, index 8) mirrors the controlled finger (index 7)
    if (info_.joints.size() > 8)
    {
      hw_positions_[8] = -hw_positions_[7];  // Mimic position (negated, opposite direction)
      hw_velocities_[8] = -hw_velocities_[7];  // Mimic velocity (negated)
      hw_efforts_[8] = 0.0;
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("LiteArmHardwareInterface"),
                          *clock_, 1000,
                          "Failed to read joint states: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LiteArmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Write commands to hardware
  try
  {
    // Extract first 7 joints for arm control (7 DOF arm)
    std::vector<double> arm_positions(hw_commands_positions_.begin(),
                                       hw_commands_positions_.begin() + 7);
    std::vector<double> arm_velocities(hw_commands_velocities_.begin(),
                                        hw_commands_velocities_.begin() + 7);
    std::vector<double> arm_efforts(hw_commands_efforts_.begin(),
                                     hw_commands_efforts_.begin() + 7);
    std::vector<double> arm_max_torques(max_torques_.begin(), max_torques_.begin() + 7);
    std::vector<double> arm_max_velocities(max_velocities_.begin(), max_velocities_.begin() + 7);
    std::vector<double> arm_kp(kp_gains_.begin(), kp_gains_.begin() + 7);
    std::vector<double> arm_kd(kd_gains_.begin(), kd_gains_.begin() + 7);

    // Replace NaN or invalid commands with current positions (avoids sending 999 values at startup)
    // The motor init value is 999.0, which is an invalid position
    const double INVALID_POS_THRESHOLD = 100.0;  // Positions > 100 rad are definitely invalid
    for (size_t i = 0; i < arm_positions.size(); i++)
    {
      if (std::isnan(arm_positions[i]) || std::abs(arm_positions[i]) > INVALID_POS_THRESHOLD)
      {
        arm_positions[i] = hw_positions_[i];
        // Also check if hw_positions_ is valid
        if (std::isnan(arm_positions[i]) || std::abs(arm_positions[i]) > INVALID_POS_THRESHOLD)
        {
          arm_positions[i] = 0.0;  // Safe fallback
        }
      }
      if (std::isnan(arm_velocities[i]))
      {
        arm_velocities[i] = 0.0;
      }
      if (std::isnan(arm_efforts[i]))
      {
        arm_efforts[i] = 0.0;
      }
    }

    // Gravity compensation feedforward (URDF convention)
    if (control_mode_ == "full_control")
    {
      std::vector<double> G(7, 0.0);
      if (use_gravity_compensation_)
      {
        Eigen::VectorXd q_pin = Eigen::VectorXd::Zero(pin_model_.nq);
        Eigen::VectorXd v_pin = Eigen::VectorXd::Zero(pin_model_.nv);
        Eigen::VectorXd a_pin = Eigen::VectorXd::Zero(pin_model_.nv);

        for (size_t i = 0; i < 7; i++)
        {
          int pin_idx = pin_joint_offset_ + i;
          if (pin_idx < (int)pin_model_.nq)
            q_pin[pin_idx] = hw_positions_[i];
        }

        Eigen::VectorXd tau = pinocchio::rnea(pin_model_, *pin_data_, q_pin, v_pin, a_pin);
        for (size_t i = 0; i < 7; i++)
        {
          int pin_idx = pin_joint_offset_ + i;
          if (pin_idx < (int)pin_model_.nq)
            G[i] = tau[pin_idx] * gravity_gain_;
        }
      }

      // Set gravity as feedforward torque
      for (size_t i = 0; i < 7; i++)
        arm_efforts[i] = G[i];
    }

    // Apply direction sign (URDF -> motor convention)
    for (size_t i = 0; i < 7; i++)
    {
      arm_positions[i] *= joint_sign_[i];
      arm_velocities[i] *= joint_sign_[i];
      arm_efforts[i] *= joint_sign_[i];
    }

    // Control 7 arm joints
    if (control_mode_ == "full_control")
    {
      // Motor internal PID (kHz) + gravity feedforward
      robot_->posVelTorqueKpKd(arm_positions, arm_velocities,
                               arm_efforts, arm_kp, arm_kd);
    }
    else if (control_mode_ == "position_velocity")
    {
      // Position-Velocity-MaxTorque control mode
      std::vector<double> velocities = use_velocity_commands_ ? arm_velocities : arm_max_velocities;

      robot_->posVelMaxTorque(arm_positions, velocities, arm_max_torques, false);
    }
    else if (control_mode_ == "pd_control")
    {
      // PD control mode (MIT mode with zero velocity and torque)
      std::vector<double> zero_vel(7, 0.0);
      std::vector<double> zero_torque(7, 0.0);
      robot_->posVelTorqueKpKd(arm_positions, zero_vel, zero_torque, arm_kp, arm_kd);
    }
    else
    {
      RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("LiteArmHardwareInterface"),
                            *clock_, 1000,
                            "Unknown control mode: %s", control_mode_.c_str());
      return hardware_interface::return_type::ERROR;
    }

    // Control gripper (8th joint, index 7) if present
    // Convert from meters to radians for hardware
    if (info_.joints.size() > 7)
    {
      double gripper_pos_m = hw_commands_positions_[7];
      double gripper_pos_rad = gripper_pos_m / gripper_rad_to_m_;
      double gripper_vel_m = use_velocity_commands_ ? hw_commands_velocities_[7] : max_velocities_[7];
      double gripper_vel_rad = gripper_vel_m / gripper_rad_to_m_;
      double gripper_max_torque = max_torques_[7];

      if (control_mode_ == "full_control")
      {
        // Use MIT mode for gripper
        double gripper_torque = hw_commands_efforts_[7];
        double gripper_kp = kp_gains_[7];
        double gripper_kd = kd_gains_[7];
        robot_->gripperControlMIT(gripper_pos_rad, gripper_vel_rad, gripper_torque,
                                   gripper_kp, gripper_kd);
      }
      else
      {
        // Use posVelMaxTorque mode for gripper
        robot_->gripperControl(gripper_pos_rad, gripper_vel_rad, gripper_max_torque);
      }
    }
  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("LiteArmHardwareInterface"),
                          *clock_, 1000,
                          "Failed to write commands: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace litearm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  litearm_hardware::LiteArmHardwareInterface, hardware_interface::SystemInterface)
