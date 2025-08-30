#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "roarm_moveit/srv/get_pose_cmd.hpp"
#include <cmath>

class RobotPosePublisher : public rclcpp::Node
{
public:
    RobotPosePublisher() : Node("robot_pose_publisher")
    {
        // TF 버퍼/리스너
        tf_buffer_   = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // hand_pose 퍼블리셔 (1회 생성)
        pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose>("hand_pose", 1);

        // 그리퍼 명령 토픽 구독: 마지막 명령값을 저장(임시 피드백으로 사용)
        gripper_cmd_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/gripper_cmd", 10,
            [this](const std_msgs::msg::Float32::SharedPtr msg)
            {
                gripper_position_ = static_cast<double>(msg->data);
            });
    }

    // EE 포즈를 TF로 갱신하고 hand_pose 토픽으로 퍼블리시
    void update_end_effector_pose()
    {
        try
        {
            const auto tf = tf_buffer_->lookupTransform(base_frame_, end_effector_frame_, tf2::TimePointZero);

            // 필요하다면 보정값 적용 (기존 코드의 -0.13381 유지)
            geometry_msgs::msg::Pose hand_pose;
            hand_pose.position.x = tf.transform.translation.x;
            hand_pose.position.y = tf.transform.translation.y;
            hand_pose.position.z = tf.transform.translation.z - 0.13381;

            // 회전까지 쓰고 싶으면 아래 주석 해제
            // hand_pose.orientation = tf.transform.rotation;

            pose_pub_->publish(hand_pose);
            last_pose_ = hand_pose;
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Could not transform %s to %s: %s",
                        base_frame_.c_str(), end_effector_frame_.c_str(), ex.what());
        }
    }

    // 서비스 응답에서 사용할 현재 gripper 값(임시: 마지막 명령값)
    double get_gripper_position() const { return gripper_position_; }

    // 서비스 응답에서 사용할 마지막 EE 포즈
    const geometry_msgs::msg::Pose& last_pose() const { return last_pose_; }

private:
    // 프레임 이름
    std::string base_frame_ = "base_link";
    std::string end_effector_frame_ = "hand_tcp";

    // TF
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

    // 퍼블리셔/구독자
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_cmd_sub_;

    // 상태 저장
    geometry_msgs::msg::Pose last_pose_;
    double gripper_position_ = 0.0;  // 마지막 /gripper_cmd 명령값(대리 피드백)
};

// 서비스 콜백
void handle_service(
    const std::shared_ptr<roarm_moveit::srv::GetPoseCmd::Request> /*request*/,
    std::shared_ptr<roarm_moveit::srv::GetPoseCmd::Response> response,
    const std::shared_ptr<RobotPosePublisher> &node)
{
    // 최신 TF 반영
    node->update_end_effector_pose();

    // 응답 채우기
    const auto &p = node->last_pose();
    response->x = p.position.x;
    response->y = p.position.y;
    response->z = p.position.z;
    response->grip = node->get_gripper_position();  // 임시: 마지막 명령값
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotPosePublisher>();

    auto server = node->create_service<roarm_moveit::srv::GetPoseCmd>(
        "get_pose_cmd",
        std::bind(&handle_service,
                  std::placeholders::_1,
                  std::placeholders::_2,
                  node));

    RCLCPP_INFO(node->get_logger(), "Service is ready to receive requests.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
