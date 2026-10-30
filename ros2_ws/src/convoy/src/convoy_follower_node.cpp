// Follower Pi 5: UART frames from the ESP32 bridge -> Watchdog -> Follower -> /cmd_vel.
// The watchdog is the ONLY thing allowed to decide what the motors do; the diff-drive driver
// (BTS7960 on hardware PWM, from the GPS-denied rover stack) just executes /cmd_vel.
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <memory>
#include <string>

#include "follower.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "uart_frame.h"

using namespace std::chrono_literals;

static int open_serial(const std::string& dev) {
  int fd = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd < 0) return -1;
  termios t{};
  tcgetattr(fd, &t);
  cfmakeraw(&t);
  cfsetspeed(&t, B921600);
  tcsetattr(fd, TCSANOW, &t);
  return fd;
}

class ConvoyFollower : public rclcpp::Node {
 public:
  ConvoyFollower() : Node("convoy_follower") {
    convoy::WatchdogConfig wc;
    wc.grace_ms = declare_parameter("grace_ms", 4000);
    wc.desired_gap_m = declare_parameter("desired_gap_m", 3.0);
    wd_ = std::make_unique<convoy::Watchdog>(wc);
    fd_ = open_serial(declare_parameter("serial", std::string("/dev/ttyAMA0")));
    if (fd_ < 0) RCLCPP_FATAL(get_logger(), "cannot open ESP32 serial link");
    cmd_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    mode_ = create_publisher<std_msgs::msg::String>("convoy/mode", 10);
    odom_ = create_subscription<nav_msgs::msg::Odometry>("odometry/filtered", 10, [this](nav_msgs::msg::Odometry::SharedPtr m) {
      self_.x = m->pose.pose.position.x; self_.y = m->pose.pose.position.y;
      const auto& q = m->pose.pose.orientation;
      self_.heading = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
    });
    timer_ = create_wall_timer(20ms, [this] { tick(); });   // 50 Hz
  }

 private:
  uint32_t now_ms() { return uint32_t((now().nanoseconds() / 1000000) & 0xFFFFFFFF); }

  void tick() {
    uint8_t buf[256];
    ssize_t n;
    while (fd_ >= 0 && (n = ::read(fd_, buf, sizeof(buf))) > 0)
      for (ssize_t i = 0; i < n; ++i)
        if (parser_.push(buf[i]) == convoy::FrameParser::Kind::Rx) wd_->on_packet(parser_.pkt, now_ms());
    const auto goal = wd_->step(now_ms(), self_.x, self_.y);
    const auto c = fol_.step(self_, goal, 0.02f);
    geometry_msgs::msg::Twist t;
    t.linear.x = c.v;
    t.angular.z = c.w;
    cmd_->publish(t);
    static const char* names[] = {"FOLLOW", "HOLD", "SAFE_STOP", "REJOIN"};
    std_msgs::msg::String s;
    s.data = names[int(goal.mode)];
    mode_->publish(s);
  }

  int fd_ = -1;
  convoy::FrameParser parser_;
  std::unique_ptr<convoy::Watchdog> wd_;
  convoy::Follower fol_;
  convoy::Pose self_{0, 0, 0};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConvoyFollower>());
  rclcpp::shutdown();
}
