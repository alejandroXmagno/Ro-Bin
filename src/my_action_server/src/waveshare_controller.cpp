#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <string.h>
#include <errno.h>
#include <string>
#include <memory>
#include <filesystem>
#include <iomanip>
#include <sstream>

class WaveshareController : public rclcpp::Node
{
public:
  WaveshareController()
  : Node("waveshare_controller")
  {
    // Open serial port
    const char* port = "/dev/ttyACM0";  // Change as needed
    fd_ = open(port, O_RDWR | O_NOCTTY);

    if (fd_ < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open port: %s", strerror(errno));
      rclcpp::shutdown();
      return;
    }

    // Configure serial: 115200 baud
    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_FATAL(this->get_logger(), "tcgetattr error: %s", strerror(errno));
      close(fd_);
      rclcpp::shutdown();
      return;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    
    // Configure 8N1
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_cflag |= CREAD | CLOCAL;                  // Turn on READ and ignore modem ctrl lines
    tty.c_cflag &= ~(PARENB | PARODD);              // No parity
    tty.c_cflag &= ~CSTOPB;                         // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                        // No hardware flow control

    // Raw input/output mode
    tty.c_lflag = 0;                                // No signaling chars, no echo
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // No software flow control
    tty.c_oflag = 0;                                // No remapping, raw output

    tty.c_cc[VMIN]  = 1;                            // Read blocks until 1 char
    tty.c_cc[VTIME] = 1;                            // 0.1s read timeout

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_FATAL(this->get_logger(), "Error from tcsetattr: %s", strerror(errno));
      close(fd_);
      rclcpp::shutdown();
      return;
    }
    
    // Init JSON parsing on the waveshare base by sending a new line
    std::string init = "\n";
    ssize_t written = write(fd_, init.c_str(), init.size());
    if (written < 0) {
      RCLCPP_ERROR(this->get_logger(), "Initialization write failed: %s", strerror(errno));
    }

    // Create subscription to cmd_vel topic
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&WaveshareController::cmd_vel_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(), "Waveshare controller node initialized");
  }

  ~WaveshareController()
  {
    if (fd_ >= 0) {
      // Send stop command before closing
      send_command(0.0, 0.0);
      close(fd_);
    }
  }

private:
  int fd_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

  std::string format_float(double value) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(1) << value;
    return ss.str();
  }

  void send_command(double linear_vel, double angular_vel)
  {
    if (fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "Cannot send command: serial port is not open");
      return;
    }

    // Form JSON command with 1 decimal place
    std::string json = R"({"T":13,"X":)" + format_float(linear_vel) + 
                      R"(,"Z":)" + format_float(angular_vel) + R"(})";
    json = json + "\n";

    // Send command
    ssize_t written = write(fd_, json.c_str(), json.size());
    RCLCPP_INFO(this->get_logger(), "Command was: %s", json.c_str());
    if (written < 0) {
      RCLCPP_ERROR(this->get_logger(), "Write failed: %s", strerror(errno));
      RCLCPP_INFO(this->get_logger(), "Command was: %s", json.c_str());
    } else {
      RCLCPP_DEBUG(this->get_logger(), "Command sent: %s", json.c_str());
      RCLCPP_INFO(this->get_logger(), "sent");
    }
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Limit linear velocity to 0.5 as per warning
    double linear_vel = std::min(std::max(msg->linear.x, -0.5), 0.5);
    
    // Send command to robot
    send_command(linear_vel, msg->angular.z);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WaveshareController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 