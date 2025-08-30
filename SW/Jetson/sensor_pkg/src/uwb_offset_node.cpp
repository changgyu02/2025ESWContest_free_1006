// sensor_pkg/src/uwb_offset_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

#include <cmath>
#include <string>
#include <vector>

class UwbOffsetNode : public rclcpp::Node {
public:
  UwbOffsetNode() : Node("uwb_offset_node") {
    // --- Parameters ---
    // left_offset_m: UWB 안테나가 로봇 중심 기준 '좌측'으로 떨어진 거리(+), '우측'이면 음수 [m]
    left_offset_m_   = this->declare_parameter<double>("left_offset_m", 0.07);
    // yaw_bias_deg: IMU 장착 오프셋(바이어스) [deg]
    yaw_bias_deg_    = this->declare_parameter<double>("yaw_bias_deg", 0.0);
    // imu_timeout_sec: IMU 최근 수신 후 유효로 간주할 시간 [s]
    imu_timeout_sec_ = this->declare_parameter<double>("imu_timeout_sec", 0.5);
    // frame_id_out: 출력 PoseStamped의 frame_id
    frame_id_out_    = this->declare_parameter<std::string>("frame_id_out", "uwb_frame");
    // use_imu_orientation_in_output: 출력 자세를 IMU로 대체할지 여부
    use_imu_orientation_in_output_ =
        this->declare_parameter<bool>("use_imu_orientation_in_output", true);

    yaw_bias_rad_ = yaw_bias_deg_ * M_PI / 180.0;

    // 파라미터 런타임 업데이트 콜백
    param_cb_ = this->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter>& params) {
        for (const auto& p : params) {
          const auto& name = p.get_name();
          if (name == "left_offset_m") {
            left_offset_m_ = p.as_double();
          } else if (name == "yaw_bias_deg") {
            yaw_bias_deg_ = p.as_double();
            yaw_bias_rad_ = yaw_bias_deg_ * M_PI / 180.0;
          } else if (name == "imu_timeout_sec") {
            imu_timeout_sec_ = p.as_double();
          } else if (name == "frame_id_out") {
            frame_id_out_ = p.as_string();
          } else if (name == "use_imu_orientation_in_output") {
            use_imu_orientation_in_output_ = p.as_bool();
          }
        }
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        return res;
      }
    );

    // --- Pub/Sub ---
    pub_offset_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/uwb_offset_pose", 10);

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu_data", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::Imu::SharedPtr msg){
        last_imu_ = *msg;
        has_imu_ = true;
        last_imu_time_ = this->now();
      }
    );

    sub_uwb_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/uwb_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
        handle_uwb(msg);
      }
    );

    RCLCPP_INFO(this->get_logger(),
      "uwb_offset_node ready. left_offset=%.3f m (양수=좌측, 음수=우측), frame_out='%s', imu_timeout=%.2fs",
      left_offset_m_, frame_id_out_.c_str(), imu_timeout_sec_);
  }

private:
  void handle_uwb(const geometry_msgs::msg::PoseStamped::SharedPtr& uwb)
  {
    if (!has_imu_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "IMU가 아직 수신되지 않았습니다. 오프셋 변환을 건너뜁니다.");
      return;
    }

    // IMU 타임아웃 체크
    const auto now = this->now();
    if ((now - last_imu_time_).seconds() > imu_timeout_sec_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
        "IMU 데이터가 오래되었습니다(> %.2fs). 오프셋 변환을 건너뜁니다.", imu_timeout_sec_);
      return;
    }

    // IMU quaternion -> yaw(rad)
    double roll, pitch, yaw;
    {
      tf2::Quaternion q(
        last_imu_.orientation.x,
        last_imu_.orientation.y,
        last_imu_.orientation.z,
        last_imu_.orientation.w
      );
      tf2::Matrix3x3 m(q);
      m.getRPY(roll, pitch, yaw);
    }

    // yaw 보정 적용
    const double yaw_corr = yaw + yaw_bias_rad_;

    // 좌측 단위벡터: heading yaw에서 left = (-sin(yaw), +cos(yaw))
    const double left_unit_x = -std::sin(yaw_corr);
    const double left_unit_y =  std::cos(yaw_corr);

    // 좌측 오프셋 벡터
    const double dx = left_offset_m_ * left_unit_x;
    const double dy = left_offset_m_ * left_unit_y;

    geometry_msgs::msg::PoseStamped out;
    out.header = uwb->header;            // 시간/스탬프 유지
    out.header.frame_id = frame_id_out_; // 출력 프레임 적용
    out.pose = uwb->pose;

    // 로봇 중심 = UWB 위치 - 좌측 오프셋 벡터
    out.pose.position.x -= dx;
    out.pose.position.y -= dy;

    // 자세는 옵션: IMU를 그대로 출력할지 유지할지
    if (use_imu_orientation_in_output_) {
      out.pose.orientation = last_imu_.orientation;
    }

    pub_offset_->publish(out);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "UWB(%.3f, %.3f) -> CENTER(%.3f, %.3f) | left=%.3fm, yaw=%.1fdeg",
      uwb->pose.position.x, uwb->pose.position.y,
      out.pose.position.x, out.pose.position.y,
      left_offset_m_, yaw_corr * 180.0 / M_PI);
  }

private:
  // Pub/Sub
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_offset_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_uwb_;

  // State
  sensor_msgs::msg::Imu last_imu_;
  bool has_imu_{false};
  rclcpp::Time last_imu_time_{0,0,RCL_ROS_TIME};

  // Params
  double left_offset_m_{0.07};
  double yaw_bias_deg_{0.0};
  double yaw_bias_rad_{0.0};
  double imu_timeout_sec_{0.5};
  std::string frame_id_out_{"uwb_frame"};
  bool use_imu_orientation_in_output_{true};

  // Parameter callback handle
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UwbOffsetNode>());
  rclcpp::shutdown();
  return 0;
}

