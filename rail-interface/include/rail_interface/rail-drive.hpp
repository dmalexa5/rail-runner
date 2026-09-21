#ifndef RAIL_INTERFACE__RAIL_DRIVE_HPP_
#define RAIL_INTERFACE__RAIL_DRIVE_HPP_

#include <memory>

#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace rail_interface
{

/** Lifecycle node that owns the drive firmware serial connection. */
class RailDrive final : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit RailDrive(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~RailDrive() override;

private:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & state) override;

  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rail_interface

#endif  // RAIL_INTERFACE__RAIL_DRIVE_HPP_
