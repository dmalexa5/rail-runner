#include "rail_interface/rail_system.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace rail_interface
{

hardware_interface::CallbackReturn RailSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  position_.assign(info.joints.size(), 0.0);
  velocity_command_.assign(info.joints.size(), 0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RailSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> RailSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < info_.joints.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_command_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn RailSystem::on_activate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RailSystem::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RailSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // TODO: replace with reads from the real rail hardware.
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RailSystem::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // TODO: replace with writes to the real rail hardware.
  return hardware_interface::return_type::OK;
}

}  // namespace rail_interface

PLUGINLIB_EXPORT_CLASS(
  rail_interface::RailSystem, hardware_interface::SystemInterface)
