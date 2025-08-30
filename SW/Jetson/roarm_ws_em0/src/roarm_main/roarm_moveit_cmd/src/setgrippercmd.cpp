#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

class GripperActionClient : public rclcpp::Node
{
public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using GoalHandleGripperCommand = rclcpp_action::ClientGoalHandle<GripperCommand>;

    GripperActionClient()
    : Node("gripper_action_client")
    {
        // GripperCommand 액션 클라이언트 생성
        this->action_client_ = rclcpp_action::create_client<GripperCommand>(this, "/gripper_controller/gripper_cmd");

        // /gripper_cmd 토픽 구독자 생성
        this->subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/gripper_cmd", 10,
            std::bind(&GripperActionClient::listener_callback, this, std::placeholders::_1));
    }

private:
    rclcpp_action::Client<GripperCommand>::SharedPtr action_client_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscription_;

    // /gripper_cmd 토픽 콜백 함수
    void listener_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        // 수신한 float32 값을 position으로 변환
        double position = static_cast<double>(msg->data);
        RCLCPP_INFO(this->get_logger(), "받은 위치 값: %f", position);

        // 목표값 전송
        send_goal(position);
    }

    // 액션 목표값 전송 함수
    void send_goal(double position)
    {
        // GripperCommand 액션 Goal 메시지 구성
        auto goal_msg = GripperCommand::Goal();
        goal_msg.command.position = position;
        goal_msg.command.max_effort = 10.0;  // 최대 힘(max_effort)은 고정값 사용

        // 액션 서버가 준비될 때까지 대기
        if (!this->action_client_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "액션 서버를 사용할 수 없습니다");
            return;
        }

        // 비동기 방식으로 Goal 전송
        auto send_goal_options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&GripperActionClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.result_callback =
            std::bind(&GripperActionClient::result_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&GripperActionClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

        this->action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    // 액션 서버로부터 Goal 수락 여부 콜백
    void goal_response_callback(GoalHandleGripperCommand::SharedPtr goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_INFO(this->get_logger(), "Goal이 서버에 의해 거부되었습니다");
        } else {
            RCLCPP_INFO(this->get_logger(), "Goal이 수락됨, 결과를 기다리는 중입니다");
        }
    }

    // 액션 수행 결과 콜백
    void result_callback(const GoalHandleGripperCommand::WrappedResult & result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Goal 성공적으로 완료됨");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal이 중단됨");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal이 취소됨");
                return;
            default:
                RCLCPP_ERROR(this->get_logger(), "알 수 없는 결과 코드");
                return;
        }
        RCLCPP_INFO(this->get_logger(), "결과 수신됨: position = %f", result.result->position);
    }

    // 액션 수행 중 피드백 콜백
    void feedback_callback(GoalHandleGripperCommand::SharedPtr,
                           const std::shared_ptr<const GripperCommand::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "피드백 수신: %f", feedback->position);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperActionClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
