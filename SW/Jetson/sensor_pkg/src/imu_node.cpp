#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "sensor_pkg/bno055.hpp"
#include <std_msgs/msg/float32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

class ImuNode : public rclcpp::Node {
public:
  ImuNode()
  : Node("imu_node"),
    imu_("/dev/i2c-7", 0x28)
  {
    pub_corr_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 10);
    pub_raw_  = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data_raw", 10);
    pub_yaw_  = this->create_publisher<std_msgs::msg::Float32>("/imu_yaw_deg", 10);

    timer_ = this->create_wall_timer(100ms, std::bind(&ImuNode::publish_imu_data, this));

    srv_set_east_ = this->create_service<std_srvs::srv::Trigger>(
      "imu_reset",
      std::bind(&ImuNode::on_set_current_as_east, this, std::placeholders::_1, std::placeholders::_2));

    srv_clear_ = this->create_service<std_srvs::srv::Trigger>(
      "imu_clear_offset",
      std::bind(&ImuNode::on_clear_offset, this, std::placeholders::_1, std::placeholders::_2));

    if (!imu_.initialize()) {
      RCLCPP_FATAL(this->get_logger(), "BNO055 초기화 실패. 노드 종료");
      rclcpp::shutdown();
      return;
    }
    if (imu_.loadCalibrationData("/home/changgyu/ros2_ws/src/bno055_calibration.json"))
      RCLCPP_INFO(this->get_logger(), "보정값 불러오기 성공");
    else
      RCLCPP_WARN(this->get_logger(), "보정값 불러오기 실패 또는 파일 없음");
  }

private:
  static double normalize180(double deg){
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
  }

  void publish_imu_data() {
    float w, x, y, z;
    if (!imu_.readQuaternion(w, x, y, z)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "쿼터니언 읽기 실패");
      return;
    }

    // 1) 원본 퍼블리시
    sensor_msgs::msg::Imu raw;
    raw.header.stamp = this->now();
    raw.header.frame_id = "imu_link";
    raw.orientation.w = w; raw.orientation.x = x; raw.orientation.y = y; raw.orientation.z = z;
    pub_raw_->publish(raw);

    // 2) 원본 → RPY
    tf2::Quaternion q_raw(x, y, z, w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_raw).getRPY(roll, pitch, yaw); // rad

    double yaw_deg = yaw * 180.0 / M_PI;
    double yaw_corr_deg = normalize180(yaw_deg + yaw_offset_deg_); // yaw만 보정
    double yaw_corr_rad = yaw_corr_deg * M_PI / 180.0;

    // 3) 보정된 RPY → 쿼터니언
    tf2::Quaternion q_corr;
    q_corr.setRPY(roll, pitch, yaw_corr_rad);

    sensor_msgs::msg::Imu corr = raw;
    corr.orientation.x = q_corr.x();
    corr.orientation.y = q_corr.y();
    corr.orientation.z = q_corr.z();
    corr.orientation.w = q_corr.w();
    pub_corr_->publish(corr);

    // 4) 확인용 yaw(deg) 퍼블리시
    std_msgs::msg::Float32 yaw_msg;
    yaw_msg.data = static_cast<float>(yaw_corr_deg);
    pub_yaw_->publish(yaw_msg);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 100,
      "yaw(raw)=%.2f°, offset=%.2f° -> yaw(corr)=%.2f°", yaw_deg, yaw_offset_deg_, yaw_corr_deg);
  }

  // 지금 방향을 동쪽(=0°) 기준으로 설정: offset = -현재 yaw
  void on_set_current_as_east(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    float w, x, y, z;
    if (!imu_.readQuaternion(w, x, y, z)) {
      resp->success = false; resp->message = "IMU 읽기 실패";
      return;
    }
    tf2::Quaternion q(x, y, z, w);
    double r, p, y_rad;
    tf2::Matrix3x3(q).getRPY(r, p, y_rad);
    double y_deg = y_rad * 180.0 / M_PI;

    yaw_offset_deg_ = normalize180(-y_deg); // 현재가 0이 되도록
    resp->success = true;
    resp->message = "현재 방향을 동쪽(0°)으로 설정 완료";
    RCLCPP_INFO(this->get_logger(), "새 오프셋: %.2f°", yaw_offset_deg_);
  }

  void on_clear_offset(
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
  {
    yaw_offset_deg_ = 0.0;
    resp->success = true;
    resp->message = "오프셋 제거";
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_corr_, pub_raw_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_yaw_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_set_east_, srv_clear_;
  BNO055 imu_;
  double yaw_offset_deg_{0.0};
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuNode>());
  rclcpp::shutdown();
  return 0;
}
