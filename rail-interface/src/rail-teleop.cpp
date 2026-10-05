#include "rail_interface/rail-teleop.hpp"

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <locale>
#include <sstream>
#include <stdexcept>
#include <string>

#include <std_msgs/msg/float64.hpp>

#include "rail_interface/rail_teleop_parameters.hpp"

namespace rail_interface
{
namespace
{

constexpr std::size_t kMaximumLineLength = 95U;

class SerialPort
{
public:
  ~SerialPort()
  {
    if (fd >= 0) {
      ::close(fd);
    }
  }

  void open_port(const std::string & path)
  {
    fd = ::open(path.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0 || ::ioctl(fd, TIOCEXCL) < 0) {
      throw std::runtime_error(std::strerror(errno));
    }
    termios settings{};
    if (::tcgetattr(fd, &settings) < 0) {
      throw std::runtime_error(std::strerror(errno));
    }
    ::cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
    settings.c_cflag = (settings.c_cflag & ~CSIZE) | CS8;
    settings.c_cc[VMIN] = 0;
    settings.c_cc[VTIME] = 0;
    if (::cfsetispeed(&settings, B115200) < 0 ||
      ::cfsetospeed(&settings, B115200) < 0 ||
      ::tcsetattr(fd, TCSANOW, &settings) < 0 ||
      ::tcflush(fd, TCIFLUSH) < 0)
    {
      throw std::runtime_error(std::strerror(errno));
    }
  }

  int fd{-1};
};

}  // namespace

struct RailTeleop::Impl
{
  explicit Impl(RailTeleop & owner)
  : node(owner), parameter_listener(owner.get_node_parameters_interface())
  {
    const auto parameters = parameter_listener.get_params();
    if (parameters.serial.port.empty() || parameters.command_topic.empty()) {
      throw std::runtime_error("Serial port and command topic must not be empty");
    }
    scale = parameters.scale;
    serial.open_port(parameters.serial.port);
    publisher = node.create_publisher<std_msgs::msg::Float64>(
      parameters.command_topic, rclcpp::SensorDataQoS().keep_last(1U));
    timer = node.create_wall_timer(std::chrono::milliseconds(2), [this]() {receive();});
  }

  void receive()
  {
    pollfd descriptor{serial.fd, POLLIN, 0};
    const int ready = ::poll(&descriptor, 1U, 0);
    if (ready < 0) {
      if (errno == EINTR) {
        return;
      }
      throw std::runtime_error(std::strerror(errno));
    }
    if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
      throw std::runtime_error("Teleop serial connection lost");
    }
    if ((descriptor.revents & POLLIN) == 0) {
      return;
    }

    char buffer[512];
    const ssize_t count = ::read(serial.fd, buffer, sizeof(buffer));
    if (count < 0) {
      if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
        return;
      }
      throw std::runtime_error(std::strerror(errno));
    }
    for (ssize_t i = 0; i < count; ++i) {
      const char ch = buffer[i];
      if (ch == '\n') {
        if (!discarding) {
          publish_line();
        }
        line.clear();
        discarding = false;
      } else if (!discarding) {
        if (line.size() == kMaximumLineLength) {
          warn_invalid();
          line.clear();
          discarding = true;
        } else {
          line.push_back(ch);
        }
      }
    }
  }

  void publish_line()
  {
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    if (line.compare(0U, 3U, "sp ") != 0) {
      warn_invalid();
      return;
    }
    std::istringstream input(line.substr(3U));
    input.imbue(std::locale::classic());
    double velocity;
    if (!(input >> std::noskipws >> velocity) || !input.eof() || !std::isfinite(velocity)) {
      warn_invalid();
      return;
    }
    std_msgs::msg::Float64 message;
    message.data = scale * velocity / 1000.0;
    publisher->publish(message);
  }

  void warn_invalid()
  {
    RCLCPP_WARN_THROTTLE(
      node.get_logger(), *node.get_clock(), 5000, "Ignoring malformed teleop serial line");
  }

  RailTeleop & node;
  rail_teleop::ParamListener parameter_listener;
  SerialPort serial;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer;
  double scale{};
  std::string line;
  bool discarding{false};
};

RailTeleop::RailTeleop(const rclcpp::NodeOptions & options)
: Node("rail_teleop", options), impl_(std::make_unique<Impl>(*this))
{
}

RailTeleop::~RailTeleop() = default;

}  // namespace rail_interface

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    auto node = std::make_shared<rail_interface::RailTeleop>();
    rclcpp::spin(node);
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(rclcpp::get_logger("rail_teleop"), "%s", exception.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
