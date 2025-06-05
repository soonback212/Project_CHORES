#include "rclcpp/rclcpp.hpp"
#include "robot_monitoring/msg/robot_status.hpp"

#include <fstream>
#include <regex>
#include <string>

class RobotMonitorNode : public rclcpp::Node {
public:
  RobotMonitorNode()
  : Node("robot_monitor_node")
  {
    status_pub_ = this->create_publisher<robot_monitoring::msg::RobotStatus>("robot_status", 10);
    serial_.open("/dev/serial0");
    serial_.setf(std::ios::skipws);
    if (!serial_.is_open()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open /dev/serial0");
      rclcpp::shutdown();
    } else {
      RCLCPP_INFO(this->get_logger(), "Serial port /dev/serial0 opened successfully.");
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&RobotMonitorNode::readSerial, this));
  }

private:
  void readSerial() {
    char ch;
    while (serial_.get(ch)) {
      if (ch == '\n') {
        // 한 줄 완성
        RCLCPP_INFO(this->get_logger(), "Raw line: '%s'", buffer_.c_str());
  
        std::regex regex("SPEED:L(-?\\d+),R(-?\\d+);TRASH:(\\d);EMERGENCY:(\\d);ENCODER:L(-?\\d+),R(-?\\d+)");
        std::smatch match;
  
        if (std::regex_search(buffer_, match, regex)) {
          auto msg = robot_monitoring::msg::RobotStatus();
          msg.left_speed = std::stoi(match[1]);
          msg.right_speed = std::stoi(match[2]);
          msg.trash_full = std::stoi(match[3]) == 1;
          msg.emergency = std::stoi(match[4]) == 1;
          msg.left_encoder = std::stoi(match[5]);
          msg.right_encoder = std::stoi(match[6]);
  
          status_pub_->publish(msg);
          RCLCPP_INFO(this->get_logger(), "Published robot status.");
        } else {
          RCLCPP_WARN(this->get_logger(), "Regex match failed for line: '%s'", buffer_.c_str());
        }
  
        buffer_.clear();  // 버퍼 초기화
      } else {
        buffer_ += ch;  // 계속 이어붙임
      }
    }
  }

  std::ifstream serial_;
  std::string buffer_;
  rclcpp::Publisher<robot_monitoring::msg::RobotStatus>::SharedPtr status_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotMonitorNode>());
  rclcpp::shutdown();
  return 0;
}