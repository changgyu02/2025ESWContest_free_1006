#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

class BatteryReaderNode : public rclcpp::Node {
public:
  BatteryReaderNode()
    : Node("battery_reader_node") {

    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/battery_status_array", 10);

    // 시리얼 포트 열기
    serial_fd_ = open("/dev/ttyCH341USB1", O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR(this->get_logger(), "시리얼 포트 열기 실패");
      throw std::runtime_error("시리얼 포트 실패");
    }

    configure_serial();

    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&BatteryReaderNode::read_and_publish, this)
    );

    RCLCPP_INFO(this->get_logger(), "BatteryReaderNode 시작");
  }

  ~BatteryReaderNode() {
    if (serial_fd_ >= 0) close(serial_fd_);
  }

private:
  int serial_fd_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  void configure_serial() {
    struct termios tty;
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "시리얼 설정 불러오기 실패");
      throw std::runtime_error("시리얼 설정 실패");
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;                                // no signaling chars, no echo
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // non-blocking read
    tty.c_cc[VTIME] = 10;                           // 1s read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // no flow control
    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls
    tty.c_cflag &= ~(PARENB | PARODD);              // no parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "시리얼 설정 적용 실패");
      throw std::runtime_error("시리얼 설정 실패");
    }
  }

  void read_and_publish() {
    char buf[128] = {0};
    int n = read(serial_fd_, buf, sizeof(buf));
    if (n > 0) {
      std::string line(buf);
      if (line.find("BAT1:") == 0) {
        try {
          float soc1 = 0.0f, soc2 = 0.0f;
          parse_battery_data(line, soc1, soc2);

          std_msgs::msg::Float32MultiArray msg;
          msg.data = {soc1, soc2};
          publisher_->publish(msg);

          RCLCPP_INFO(this->get_logger(), "Sensor1: %.2f %%, Sensor2: %.2f %%", soc1, soc2);
        } catch (const std::exception& e) {
          RCLCPP_WARN(this->get_logger(), "파싱 실패: %s", e.what());
        }
      }
    }
  }

  void parse_battery_data(const std::string &line, float &soc1, float &soc2) {
    size_t bat1_pos = line.find("BAT1:");
    size_t bat2_pos = line.find("BAT2:");

    if (bat1_pos == std::string::npos || bat2_pos == std::string::npos)
      throw std::runtime_error("BAT1 또는 BAT2 누락");

    std::string val1 = line.substr(bat1_pos + 5, bat2_pos - bat1_pos - 6);  // 5 = length of "BAT1:"
    std::string val2 = line.substr(bat2_pos + 5);

    soc1 = std::stof(val1);
    soc2 = std::stof(val2);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BatteryReaderNode>());
  rclcpp::shutdown();
  return 0;
}
