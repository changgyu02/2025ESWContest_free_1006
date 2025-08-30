#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <boost/asio.hpp>
#include <boost/asio/serial_port.hpp>
#include <sstream>
#include <vector>
#include <algorithm>

using namespace boost::asio;

class DWM1001Node : public rclcpp::Node {
public:
    DWM1001Node() : Node("uwb_to_ros"), io(), serial_(io) {
        // 퍼블리셔 (원본 데이터만)
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("uwb_pose", 10);

        // 시리얼 포트 설정
        try {
            serial_.open("/dev/serial/by-id/usb-SEGGER_J-Link_000760202242-if00");
            serial_.set_option(serial_port_base::baud_rate(115200));
            serial_.set_option(serial_port_base::character_size(8));
            serial_.set_option(serial_port_base::flow_control(serial_port_base::flow_control::none));
            serial_.set_option(serial_port_base::parity(serial_port_base::parity::none));
            serial_.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
            RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // 주기적 읽기 타이머
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                         std::bind(&DWM1001Node::read_position, this));
    }

private:
    void read_position() {
        try {
            boost::asio::streambuf buffer;
            boost::asio::read_until(serial_, buffer, "\n");
            std::istream is(&buffer);
            std::string line;
            std::getline(is, line);

            if (line.find("POS") != std::string::npos) {
                std::vector<std::string> tokens;
                std::stringstream ss(line);
                std::string token;
                while (std::getline(ss, token, ',')) {
                    tokens.push_back(token);
                }

                auto pos_index = std::find(tokens.begin(), tokens.end(), "POS");
                if (pos_index != tokens.end() && std::distance(pos_index, tokens.end()) >= 4) {
                    std::string x_str = *(pos_index + 1);
                    std::string y_str = *(pos_index + 2);
                    std::string z_str = *(pos_index + 3);

                    if (is_number(x_str) && is_number(y_str) && is_number(z_str)) {
                        double x = std::stod(x_str);
                        double y = std::stod(y_str);
                        double z = std::stod(z_str);

                        // 원본 퍼블리시
                        auto pose_msg = geometry_msgs::msg::PoseStamped();
                        pose_msg.header.stamp = this->now();
                        pose_msg.header.frame_id = "uwb_frame";
                        pose_msg.pose.position.x = x;
                        pose_msg.pose.position.y = y;
                        pose_msg.pose.position.z = z;
                        pose_msg.pose.orientation.w = 1.0;

                        publisher_->publish(pose_msg);

                        RCLCPP_INFO(this->get_logger(), "Raw: (%.2f, %.2f)", x, y);
                    }
                }
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error reading serial data: %s", e.what());
        }
    }

    // 문자열 숫자 여부 검사
    bool is_number(const std::string& s) {
        try {
            std::stod(s);
            return true;
        } catch (...) {
            return false;
        }
    }

    // 구성 요소
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    io_service io;
    serial_port serial_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DWM1001Node>());
    rclcpp::shutdown();
    return 0;
}

