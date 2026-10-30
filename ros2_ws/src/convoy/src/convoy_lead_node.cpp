// Lead Pi 5: /odometry/filtered -> pose frames to the ESP32 at 20 Hz (ESP32 broadcasts at 10 Hz).
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <memory>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "uart_frame.h"

using namespace std::chrono_literals;

class ConvoyLead : public rclcpp::Node {
 public:
  ConvoyLead() : Node("convoy_lead") {
    fd_ = ::open(declare_parameter("serial", std::string("/dev/ttyAMA0")).c_str(), O_RDWR | O_NOCTTY);
    termios t{};
    tcgetattr(fd_, &t); cfmakeraw(&t); cfsetspeed(&t, B921600); tcsetattr(fd_, TCSANOW, &t);
    sub_ = create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {
      const auto& q = m->pose.pose.orientation;
      pose_ = {float(m->pose.pose.position.x), float(m->pose.pose.position.y),
               float(std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))),
               float(m->twist.twist.linear.x)};
      fresh_ = true;
    });
    timer_ = create_wall_timer(50ms, [this] {
      if (!fresh_ || fd_ < 0) return;            // no odometry -> send nothing -> ESP32 stops broadcasting
      uint8_t f[convoy::kPoseFrameLen];
      if (::write(fd_, f, convoy::encode_pose(pose_, f)) < 0) RCLCPP_WARN(get_logger(), "uart write failed");
      fresh_ = false;
    });
  }

 private:
  int fd_ = -1;
  convoy::Pose4 pose_{};
  bool fresh_ = false;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConvoyLead>());
  rclcpp::shutdown();
}
