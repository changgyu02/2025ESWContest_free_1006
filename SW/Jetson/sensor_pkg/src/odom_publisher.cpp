#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

class OdomPublisher : public rclcpp::Node {
public:
  OdomPublisher() : Node("odom_publisher") {
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    uwb_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/uwb_pose", 10,
      [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        last_uwb_pose_ = *msg;
        update_odom();
      });

    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu_data", 10,
      [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
        last_imu_ = *msg;
        update_odom();
      });

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_vel_ = *msg;
        update_odom();
      });
  }

private:
  void update_odom() {
    if (last_uwb_pose_.header.stamp.sec == 0 || last_imu_.header.stamp.sec == 0)
      return;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position = last_uwb_pose_.pose.position;
    odom_msg.pose.pose.orientation = last_imu_.orientation;
    odom_msg.twist.twist = last_cmd_vel_;  // 엔코더 속도 사용

    odom_pub_->publish(odom_msg);

    // 쿼터니언 → Yaw(rad) → Yaw(deg)
    tf2::Quaternion q(
      odom_msg.pose.pose.orientation.x,
      odom_msg.pose.pose.orientation.y,
      odom_msg.pose.pose.orientation.z,
      odom_msg.pose.pose.orientation.w
    );
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
    double yaw_deg = yaw * 180.0 / M_PI;

    // 출력 (도 단위)
    RCLCPP_INFO(this->get_logger(),
                "[ODOM] x=%.3f, y=%.3f, yaw=%.1f deg",
                odom_msg.pose.pose.position.x,
                odom_msg.pose.pose.position.y,
                yaw_deg);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr uwb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  geometry_msgs::msg::PoseStamped last_uwb_pose_;
  sensor_msgs::msg::Imu last_imu_;
  geometry_msgs::msg::Twist last_cmd_vel_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomPublisher>());
  rclcpp::shutdown();
  return 0;
}

