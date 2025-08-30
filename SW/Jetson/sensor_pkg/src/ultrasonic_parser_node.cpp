#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <regex>
#include <string>
#include <vector>

class UltrasonicParserNode : public rclcpp::Node {
public:
  UltrasonicParserNode()
  : Node("ultrasonic_parser_node") {
    this->declare_parameter<std::string>("serial_port", "/dev/ttyCH341USB0");
    this->declare_parameter<int>("baud_rate", 9600);

    this->get_parameter("serial_port", serial_port_);
    this->get_parameter("baud_rate", baud_rate_);

    distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("/ultrasonic_distance", 10);
    trash_pub_ = this->create_publisher<std_msgs::msg::Float32>("/trash_fill_level", 10);

    openSerialPort();

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(20),  // 빠르게 체크 (50Hz)
      std::bind(&UltrasonicParserNode::readSerial, this)
    );

    RCLCPP_INFO(this->get_logger(), "UART parser started on %s (%d baud)", serial_port_.c_str(), baud_rate_);
  }

  ~UltrasonicParserNode() {
    if (fd_ != -1) close(fd_);
  }

private:
  void openSerialPort() {
    fd_ = open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ == -1) {
      RCLCPP_ERROR(this->get_logger(), "시리얼 포트 열기 실패: %s", serial_port_.c_str());
      throw std::runtime_error("Failed to open serial port");
    }

    struct termios tty;
    tcgetattr(fd_, &tty);
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);
    tty.c_cflag |= (CLOCAL | CREAD); // Enable receiver, set local mode
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;              // 8-bit
    tty.c_cflag &= ~PARENB;          // No parity
    tty.c_cflag &= ~CSTOPB;          // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;         // No flow control
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Raw input
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);          // No software flow ctrl
    tty.c_oflag &= ~OPOST;
    tcsetattr(fd_, TCSANOW, &tty);
  }

  void readSerial() {
    char buf[256];
    int n = read(fd_, buf, sizeof(buf));
    if (n > 0) {
      buffer_.append(buf, n);

      size_t newline;
      while ((newline = buffer_.find('\n')) != std::string::npos) {
        std::string line = buffer_.substr(0, newline);
        buffer_.erase(0, newline + 1);

        parseLine(line);
      }
    }
  }

  void parseLine(const std::string& line) {
    if (line.empty()) return;

    std::smatch match;
    std::regex pattern(R"(O:([0-9.]+)\s+T:([0-9.]+))");
    if (std::regex_search(line, match, pattern) && match.size() == 3) {
      try {
        float obstacle = std::stof(match[1].str());
        float trash = std::stof(match[2].str());

        std_msgs::msg::Float32 obs_msg;
        obs_msg.data = obstacle;
        distance_pub_->publish(obs_msg);

        std_msgs::msg::Float32 trash_msg;
        trash_msg.data = trash;
        trash_pub_->publish(trash_msg);

        RCLCPP_INFO(this->get_logger(), "Obstacle: %.2f cm | Trash: %.2f cm", obstacle, trash);

      } catch (...) {
        RCLCPP_WARN(this->get_logger(), "숫자 변환 실패: %s", line.c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(), "형식에 맞지 않는 데이터: %s", line.c_str());
    }
  }

  std::string serial_port_;
  int baud_rate_;
  int fd_ = -1;
  std::string buffer_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr trash_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UltrasonicParserNode>());
  rclcpp::shutdown();
  return 0;
}
