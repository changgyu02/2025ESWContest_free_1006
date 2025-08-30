#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <cstring>
#include <iostream>

#define UART_PORT "/dev/ttyCH341USB2"
#define BAUD_RATE B115200

class MDControl {
public:
    MDControl() {
        uart_fd = open(UART_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
        if (uart_fd == -1) {
            RCLCPP_ERROR(rclcpp::get_logger("MDControl"), "UART 포트 열기 실패 (%s)", UART_PORT);
        } else {
            fcntl(uart_fd, F_SETFL, 0);
            if (configureUART() == 0) {
                RCLCPP_INFO(rclcpp::get_logger("MDControl"), "UART 연결 완료 (포트: %s)", UART_PORT);
            }
        }
    }

    ~MDControl() {
        if (uart_fd != -1) {
            close(uart_fd);
        }
    }

    void sendSpeed(double linear_x, double angular_z) {
        // 기본 속도 변환: -1.0 ~ 1.0 → 0 ~ 255 (127 기준)
        int left_speed = static_cast<int>((linear_x - angular_z) * 50.0 + 127.0);
        int right_speed = static_cast<int>((linear_x + angular_z) * 50.0 + 127.0);

        left_speed = std::clamp(left_speed, 0, 255);
        right_speed = std::clamp(right_speed, 0, 255);

        sendPacket(0, false, left_speed);   // 주소 0, 왼쪽 모터
        sendPacket(0, true, right_speed);   // 주소 0, 오른쪽 모터

        RCLCPP_INFO(rclcpp::get_logger("MDControl"), "전송됨: left=%d, right=%d", left_speed, right_speed);
    }

private:
    int uart_fd;

    int configureUART() {
        struct termios tty;
        if (tcgetattr(uart_fd, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("MDControl"), "시리얼 속성 가져오기 실패");
            return -1;
        }

        cfsetospeed(&tty, BAUD_RATE);
        cfsetispeed(&tty, BAUD_RATE);

        tty.c_cflag = CS8 | CLOCAL | CREAD;
        tty.c_iflag = IGNPAR;
        tty.c_oflag = 0;
        tty.c_lflag = 0;

        tcflush(uart_fd, TCIFLUSH);
        if (tcsetattr(uart_fd, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(rclcpp::get_logger("MDControl"), "시리얼 속성 적용 실패");
            return -1;
        }
        return 0;
    }

    void sendPacket(uint8_t base_address, bool is_right_motor, uint8_t speed) {
        uint8_t header = 0x55;
        uint8_t address = base_address & 0x07;
        if (is_right_motor) address |= 0x08;

        uint8_t command = speed;
        uint8_t checksum = header + address + command;

        uint8_t packet[4] = {header, address, command, checksum};

        if (uart_fd != -1) {
            write(uart_fd, packet, 4);
        }
    }
};

class MDControlNode : public rclcpp::Node {
public:
    MDControlNode() : Node("md_control") {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&MDControlNode::cmdVelCallback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "MDControl Node 실행 중 (Listening to /cmd_vel)");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    MDControl md_control_;

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        md_control_.sendSpeed(msg->linear.x, msg->angular.z);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MDControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
