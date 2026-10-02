#ifndef RAIL_INTERFACE__RAIL_TELEOP_HPP_
#define RAIL_INTERFACE__RAIL_TELEOP_HPP_

#include <memory>

#include <rclcpp/rclcpp.hpp>

namespace rail_interface
{

/** Receives joystick setpoints and forwards them in SI units. */
class RailTeleop final : public rclcpp::Node
{
public:
  explicit RailTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RailTeleop() override;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rail_interface

#endif  // RAIL_INTERFACE__RAIL_TELEOP_HPP_
