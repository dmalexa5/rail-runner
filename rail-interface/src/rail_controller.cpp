#include "rail_interface/rail_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

namespace rail_interface
{

controller_interface::CallbackReturn RailController::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration RailController::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, {"rail_joint/velocity"}};
}

controller_interface::InterfaceConfiguration RailController::state_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::INDIVIDUAL, {"rail_joint/position"}};
}

controller_interface::return_type RailController::update(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // TODO: implement command generation and feedback handling.
  return controller_interface::return_type::OK;
}

}  // namespace rail_interface

PLUGINLIB_EXPORT_CLASS(
  rail_interface::RailController, controller_interface::ControllerInterface)
