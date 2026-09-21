#include "rail_interface/rail-drive.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <thread>
#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rail_interface/rail_drive_parameters.hpp"

namespace rail_interface
{
namespace
{

using namespace std::chrono_literals;

constexpr auto kUpdatePeriod = 2ms;
constexpr auto kReplyTimeout = 5ms;
constexpr auto kCommandTimeout = 20ms;
constexpr double kMaximumVelocityMps = 0.032;
constexpr std::size_t kMaximumReplyLength = 95U;

class SerialPort
{
public:
  ~SerialPort()
  {
    close_port();
  }

  bool open_port(const std::string & path, std::string & error)
  {
    close_port();
    fd_ = ::open(path.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd_ < 0) {
      error = std::strerror(errno);
      return false;
    }

    if (::ioctl(fd_, TIOCEXCL) < 0) {
      error = std::strerror(errno);
      close_port();
      return false;
    }

    termios settings{};
    if (::tcgetattr(fd_, &settings) < 0) {
      error = std::strerror(errno);
      close_port();
      return false;
    }
    ::cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    settings.c_cflag = (settings.c_cflag & ~CSIZE) | CS8;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (::cfsetispeed(&settings, B230400) < 0 ||
      ::cfsetospeed(&settings, B230400) < 0 ||
      ::tcsetattr(fd_, TCSANOW, &settings) < 0 ||
      ::tcflush(fd_, TCIOFLUSH) < 0)
    {
      error = std::strerror(errno);
      close_port();
      return false;
    }
    return true;
  }

  bool is_open() const
  {
    return fd_ >= 0;
  }

  void close_port()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  bool transact(const std::string & request, std::string & reply, std::string & error)
  {
    if (fd_ < 0) {
      error = "serial port is closed";
      return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + kReplyTimeout;
    std::size_t written = 0U;
    while (written < request.size()) {
      const ssize_t count = ::write(fd_, request.data() + written, request.size() - written);
      if (count > 0) {
        written += static_cast<std::size_t>(count);
      } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        error = std::strerror(errno);
        return false;
      } else if (!wait_for(POLLOUT, deadline, error)) {
        return false;
      }
    }

    reply.clear();
    while (reply.size() <= kMaximumReplyLength) {
      char ch = '\0';
      const ssize_t count = ::read(fd_, &ch, 1U);
      if (count == 1) {
        if (ch == '\n') {
          if (reply.empty()) {
            error = "empty reply";
            return false;
          }
          return true;
        }
        reply.push_back(ch);
      } else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        error = std::strerror(errno);
        return false;
      } else if (!wait_for(POLLIN, deadline, error)) {
        return false;
      }
    }

    error = "reply exceeds maximum length";
    return false;
  }

private:
  bool wait_for(
    short events, const std::chrono::steady_clock::time_point & deadline,
    std::string & error)
  {
    while (true) {
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        error = "reply timeout";
        return false;
      }
      const auto remaining = deadline - now;
      const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(remaining + 999us).count();
      pollfd descriptor{fd_, events, 0};
      const int result = ::poll(&descriptor, 1U, static_cast<int>(milliseconds));
      if (result > 0) {
        if ((descriptor.revents & events) != 0) {
          return true;
        }
        error = "serial poll failure";
        return false;
      }
      if (result == 0) {
        error = "reply timeout";
        return false;
      }
      if (errno != EINTR) {
        error = std::strerror(errno);
        return false;
      }
    }
  }

  int fd_{-1};
};

struct Feedback
{
  double position_mm;
  double velocity_mm_s;
};

bool parse_decimal(std::string_view text, double & value)
{
  if (text.empty()) {
    return false;
  }

  std::size_t index = 0U;
  double sign = 1.0;
  if (text[index] == '+' || text[index] == '-') {
    sign = text[index] == '-' ? -1.0 : 1.0;
    if (++index == text.size()) {
      return false;
    }
  }

  double whole = 0.0;
  const std::size_t first_digit = index;
  while (index < text.size() && text[index] >= '0' && text[index] <= '9') {
    whole = whole * 10.0 + static_cast<double>(text[index] - '0');
    ++index;
  }
  if (index == first_digit || index + 2U != text.size() || text[index] != '.' ||
    text[index + 1U] < '0' || text[index + 1U] > '9')
  {
    return false;
  }

  value = sign * (whole + static_cast<double>(text[index + 1U] - '0') / 10.0);
  return std::isfinite(value);
}

bool parse_ack(const std::string & reply, Feedback & feedback)
{
  constexpr std::string_view prefix{"ack "};
  if (reply.size() <= prefix.size() || reply.compare(0U, prefix.size(), prefix) != 0) {
    return false;
  }

  const std::string_view fields(reply.data() + prefix.size(), reply.size() - prefix.size());
  const std::size_t first_space = fields.find(' ');
  if (first_space == std::string_view::npos) {
    return false;
  }
  const std::size_t second_space = fields.find(' ', first_space + 1U);
  if (second_space == std::string_view::npos ||
    fields.find(' ', second_space + 1U) != std::string_view::npos)
  {
    return false;
  }

  double acceleration_mm_s2 = 0.0;
  return parse_decimal(fields.substr(0U, first_space), feedback.position_mm) &&
         parse_decimal(
    fields.substr(first_space + 1U, second_space - first_space - 1U), feedback.velocity_mm_s) &&
         parse_decimal(fields.substr(second_space + 1U), acceleration_mm_s2);
}

std::string format_setpoint(double velocity_m_s)
{
  const long tenths_mm_s = std::lround(velocity_m_s * 10000.0);
  const unsigned long magnitude = static_cast<unsigned long>(
    tenths_mm_s < 0 ? -tenths_mm_s : tenths_mm_s);
  char request[32];
  std::snprintf(
    request, sizeof(request), "sp %s%lu.%lu\n", tenths_mm_s < 0 ? "-" : "",
    magnitude / 10UL, magnitude % 10UL);
  return request;
}

bool is_known_error(const std::string & reply)
{
  return reply == "err cal" || reply == "err dis" || reply == "err lim" ||
         reply == "err hrd" || reply == "err est" || reply == "err pos" ||
         reply == "err com" || reply == "err mot" || reply == "err can" ||
         reply == "err cmd" || reply == "err sys";
}

}  // namespace

struct RailDrive::Impl
{
  explicit Impl(RailDrive & owner)
  : node(owner), parameter_listener(
      std::make_shared<rail_drive::ParamListener>(owner.get_node_parameters_interface()))
  {
  }

  ~Impl()
  {
    stop_active_loop();
    if (serial.is_open()) {
      (void)disable();
      serial.close_port();
    }
  }

  CallbackReturn configure()
  {
    recover_to_unconfigured = false;
    try {
      parameters = parameter_listener->get_params();
    } catch (const std::exception & exception) {
      RCLCPP_ERROR(node.get_logger(), "Failed to load parameters: %s", exception.what());
      return CallbackReturn::ERROR;
    }

    if (parameters->serial.port.empty() || parameters->joint_name.empty() ||
      parameters->command_topic.empty() || parameters->state_topic.empty())
    {
      RCLCPP_ERROR(node.get_logger(), "Serial port, joint, and topic names must not be empty");
      return CallbackReturn::ERROR;
    }

    create_interfaces();
    std::string error;
    if (!serial.open_port(parameters->serial.port, error)) {
      RCLCPP_ERROR(
        node.get_logger(), "Failed to open %s: %s", parameters->serial.port.c_str(), error.c_str());
      destroy_interfaces();
      return CallbackReturn::ERROR;
    }
    if (!disable()) {
      serial.close_port();
      destroy_interfaces();
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn activate()
  {
    recover_to_unconfigured = false;
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(parameters->calibration_timeout);
    auto next_request = std::chrono::steady_clock::now();
    bool calibrated = false;

    while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline) {
      std::string reply;
      std::string error;
      if (!serial.transact("cal 0\n", reply, error)) {
        RCLCPP_ERROR(node.get_logger(), "Calibration transaction failed: %s", error.c_str());
        break;
      }

      Feedback feedback{};
      if (parse_ack(reply, feedback)) {
        calibrated = true;
        break;
      }
      if (reply != "cal 0") {
        RCLCPP_ERROR(node.get_logger(), "Calibration failed with reply '%s'", reply.c_str());
        break;
      }

      next_request += kUpdatePeriod;
      const auto now = std::chrono::steady_clock::now();
      if (next_request > now) {
        std::this_thread::sleep_until(next_request);
      }
    }

    if (!calibrated) {
      if (rclcpp::ok() && std::chrono::steady_clock::now() >= deadline) {
        RCLCPP_ERROR(node.get_logger(), "Calibration timed out");
      }
      recover_to_unconfigured = disable();
      return CallbackReturn::ERROR;
    }

    latest_velocity_m_s = 0.0;
    command_received = false;
    active = true;
    state_publisher->on_activate();
    update_timer = node.create_wall_timer(kUpdatePeriod, [this]() {active_cycle();});
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn deactivate()
  {
    recover_to_unconfigured = false;
    stop_active_loop();
    deactivate_publisher();
    return disable() ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
  }

  CallbackReturn cleanup()
  {
    recover_to_unconfigured = false;
    stop_active_loop();
    if (!disable()) {
      serial.close_port();
      destroy_interfaces();
      return CallbackReturn::ERROR;
    }
    serial.close_port();
    destroy_interfaces();
    parameters.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn shutdown()
  {
    stop_active_loop();
    deactivate_publisher();
    if (serial.is_open() && !disable()) {
      RCLCPP_ERROR(node.get_logger(), "Could not verify disable during shutdown");
    }
    serial.close_port();
    destroy_interfaces();
    parameters.reset();
    recover_to_unconfigured = false;
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn error()
  {
    stop_active_loop();
    deactivate_publisher();
    serial.close_port();
    destroy_interfaces();
    parameters.reset();
    const bool recover = std::exchange(recover_to_unconfigured, false);
    return recover ? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
  }

  void create_interfaces()
  {
    const auto qos = rclcpp::SensorDataQoS().keep_last(1U);
    state_publisher = node.create_publisher<sensor_msgs::msg::JointState>(
      parameters->state_topic, qos);
    command_subscription = node.create_subscription<std_msgs::msg::Float64>(
      parameters->command_topic, qos,
      [this](const std_msgs::msg::Float64 & message) {receive_command(message.data);});
  }

  void destroy_interfaces()
  {
    if (update_timer) {
      update_timer->cancel();
    }
    update_timer.reset();
    command_subscription.reset();
    state_publisher.reset();
  }

  void deactivate_publisher()
  {
    if (state_publisher && state_publisher->is_activated()) {
      state_publisher->on_deactivate();
    }
  }

  void stop_active_loop()
  {
    active = false;
    if (update_timer) {
      update_timer->cancel();
    }
  }

  bool disable()
  {
    if (!serial.is_open()) {
      return false;
    }
    std::string reply;
    std::string error;
    if (!serial.transact("dis 0\n", reply, error)) {
      RCLCPP_ERROR(node.get_logger(), "Disable transaction failed: %s", error.c_str());
      return false;
    }
    if (reply != "dis 0") {
      RCLCPP_ERROR(node.get_logger(), "Disable returned '%s'", reply.c_str());
      return false;
    }
    return true;
  }

  void receive_command(double velocity_m_s)
  {
    if (!active) {
      return;
    }
    if (!std::isfinite(velocity_m_s) || std::abs(velocity_m_s) > kMaximumVelocityMps) {
      latest_velocity_m_s = 0.0;
      command_received = false;
      RCLCPP_WARN_THROTTLE(
        node.get_logger(), *node.get_clock(), 1000,
        "Invalid rail velocity command; commanding zero");
      return;
    }
    latest_velocity_m_s = velocity_m_s;
    last_command = std::chrono::steady_clock::now();
    command_received = true;
  }

  void active_cycle()
  {
    if (!active) {
      return;
    }

    const auto now = std::chrono::steady_clock::now();
    const double command = command_received && now - last_command < kCommandTimeout ?
      latest_velocity_m_s : 0.0;
    std::string reply;
    std::string error;
    if (!serial.transact(format_setpoint(command), reply, error)) {
      runtime_fault("Serial transaction failed: " + error);
      return;
    }

    Feedback feedback{};
    if (parse_ack(reply, feedback)) {
      sensor_msgs::msg::JointState message;
      message.header.stamp = node.now();
      message.name.push_back(parameters->joint_name);
      message.position.push_back(feedback.position_mm / 1000.0);
      message.velocity.push_back(feedback.velocity_mm_s / 1000.0);
      state_publisher->publish(message);
      return;
    }
    if (reply == "err pos") {
      RCLCPP_WARN_THROTTLE(
        node.get_logger(), *node.get_clock(), 1000,
        "Firmware rejected outward motion at the position limit");
      return;
    }
    if (is_known_error(reply)) {
      runtime_fault("Firmware returned " + reply);
    } else {
      runtime_fault("Malformed firmware reply: " + reply);
    }
  }

  void runtime_fault(const std::string & reason)
  {
    RCLCPP_ERROR(node.get_logger(), "%s; finalizing", reason.c_str());
    stop_active_loop();
    (void)node.shutdown();
  }

  RailDrive & node;
  std::shared_ptr<rail_drive::ParamListener> parameter_listener;
  std::optional<rail_drive::Params> parameters;
  SerialPort serial;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr state_publisher;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr command_subscription;
  rclcpp::TimerBase::SharedPtr update_timer;
  std::chrono::steady_clock::time_point last_command{};
  double latest_velocity_m_s{0.0};
  bool command_received{false};
  bool active{false};
  bool recover_to_unconfigured{false};
};

RailDrive::RailDrive(const rclcpp::NodeOptions & options)
: LifecycleNode("rail_drive", options), impl_(std::make_unique<Impl>(*this))
{
}

RailDrive::~RailDrive() = default;

RailDrive::CallbackReturn RailDrive::on_configure(const rclcpp_lifecycle::State &)
{
  return impl_->configure();
}

RailDrive::CallbackReturn RailDrive::on_activate(const rclcpp_lifecycle::State &)
{
  return impl_->activate();
}

RailDrive::CallbackReturn RailDrive::on_deactivate(const rclcpp_lifecycle::State &)
{
  return impl_->deactivate();
}

RailDrive::CallbackReturn RailDrive::on_cleanup(const rclcpp_lifecycle::State &)
{
  return impl_->cleanup();
}

RailDrive::CallbackReturn RailDrive::on_shutdown(const rclcpp_lifecycle::State &)
{
  return impl_->shutdown();
}

RailDrive::CallbackReturn RailDrive::on_error(const rclcpp_lifecycle::State &)
{
  return impl_->error();
}

}  // namespace rail_interface

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rail_interface::RailDrive>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  node.reset();
  rclcpp::shutdown();
  return 0;
}
