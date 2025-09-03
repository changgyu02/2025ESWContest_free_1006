#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <vector>
#include <string>
#include <limits>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

/*
  twist_mux.cpp (ROS2 Humble)
  - 입력:  /cmd_vel_align, /cmd_vel_nav
  - 출력:  /cmd_vel (항상 최종 출력은 /cmd_vel)
  - 우선순위: align > nav
  - 타임아웃: align 200ms, nav 300ms (기본)
  - "새 메시지"의 정의: 값의 변화가 아니라 '도착 자체' (heartbeat)
*/

struct InputChannel {
  std::string topic;
  int priority;     // 높을수록 우선
  int timeout_ms;   // 마지막 수신으로부터 이 시간 이내면 "살아있음"
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub;

  geometry_msgs::msg::Twist last_msg;
  rclcpp::Time last_stamp;
  bool has_msg{false};
};

class TwistMuxNode : public rclcpp::Node {
public:
  TwistMuxNode()
  : Node("twist_mux_node")
  {
    // ===== 파라미터 선언 (기본값은 align > nav, 출력은 /cmd_vel) =====
    input_topics_ = this->declare_parameter<std::vector<std::string>>(
      "input_topics", {"/cmd_vel_align", "/cmd_vel_nav"});

    // int → int64_t 로 변경 (ROS2 파라미터 내부 표현과 호환)
    auto pri64 = this->declare_parameter<std::vector<int64_t>>(
      "priorities", {100, 50}); 
    auto to_ms64 = this->declare_parameter<std::vector<int64_t>>(
      "timeouts_ms", {200, 300}); 

    // int64_t → int 변환 저장
    priorities_.assign(pri64.begin(), pri64.end());
    timeouts_ms_.assign(to_ms64.begin(), to_ms64.end());

    output_topic_ = this->declare_parameter<std::string>("output_topic", "/cmd_vel");
    publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 50.0);
    stop_on_idle_ = this->declare_parameter<bool>("stop_on_idle", true);

    // 유효성 체크
    if (input_topics_.size() != priorities_.size() ||
        input_topics_.size() != timeouts_ms_.size())
    {
      RCLCPP_FATAL(get_logger(),
                   "Parameter length mismatch: input_topics(%zu), priorities(%zu), timeouts_ms(%zu)",
                   input_topics_.size(), priorities_.size(), timeouts_ms_.size());
      rclcpp::shutdown();
      return;
    }

    // 퍼블리셔 생성 (최종 출력은 항상 /cmd_vel 또는 지정한 output_topic_)
    pub_ = this->create_publisher<geometry_msgs::msg::Twist>(output_topic_, rclcpp::QoS(10));

    // 입력 채널 구독자 생성
    channels_.reserve(input_topics_.size());
    for (size_t i = 0; i < input_topics_.size(); ++i) {
      InputChannel ch;
      ch.topic = input_topics_[i];
      ch.priority = priorities_[i];
      ch.timeout_ms = timeouts_ms_[i];

      ch.sub = this->create_subscription<geometry_msgs::msg::Twist>(
        ch.topic, rclcpp::QoS(20),
        [this, idx = i](geometry_msgs::msg::Twist::ConstSharedPtr msg) {
          auto &c = channels_[idx];
          c.last_msg = *msg;
          c.last_stamp = this->now();
          c.has_msg = true;
        }
      );

      channels_.push_back(std::move(ch));
      RCLCPP_INFO(get_logger(), "Input[%zu] topic='%s', priority=%d, timeout=%d ms",
                  i, channels_.back().topic.c_str(),
                  channels_.back().priority, channels_.back().timeout_ms);
    }

    // 타이머 (주기 퍼블리시; 입력 없으면 옵션에 따라 0속도 정지 명령 퍼블리시)
    const double hz = std::max(1.0, publish_rate_hz_);
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / hz));
    timer_ = this->create_wall_timer(period, std::bind(&TwistMuxNode::onTimer, this));

    RCLCPP_INFO(get_logger(),
      "twist_mux_node ready. output='%s', rate=%.1f Hz, stop_on_idle=%s",
      output_topic_.c_str(), publish_rate_hz_, stop_on_idle_ ? "true" : "false");
  }

private:
  void onTimer() {
    // 유효(타임아웃 내) 입력 중 priority 최댓값 선택
    int best_idx = -1;
    int best_pri = std::numeric_limits<int>::min();
    const rclcpp::Time now = this->now();

    for (size_t i = 0; i < channels_.size(); ++i) {
      auto &c = channels_[i];
      if (!c.has_msg) continue;

      const int dt_ms = static_cast<int>((now - c.last_stamp).nanoseconds() / 1000000LL);
      if (dt_ms <= c.timeout_ms) {
        if (c.priority > best_pri) {
          best_pri = c.priority;
          best_idx = static_cast<int>(i);
        }
      }
    }

    bool publish = false;
    geometry_msgs::msg::Twist out;

    if (best_idx >= 0) {
      // 선택된(우선순위 최상) 입력을 그대로 사용
      out = channels_[best_idx].last_msg;
      publish = true;

      if (best_idx != last_selected_) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "Selected source -> '%s' (priority=%d)",
          channels_[best_idx].topic.c_str(), channels_[best_idx].priority);
        last_selected_ = best_idx;
      }
    } else {
      // 유효 입력 없음
      if (stop_on_idle_) {
        // 안전 정지 명령 퍼블리시
        out = geometry_msgs::msg::Twist{};
        publish = true;

        if (last_selected_ != -2) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "No valid input; publishing STOP on '%s'", output_topic_.c_str());
          last_selected_ = -2; // idle 표시
        }
      } else {
        // 아무 것도 퍼블리시하지 않음 (구독측이 워치독 갖고 있다면 주의)
        publish = false;

        if (last_selected_ != -3) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000,
            "No valid input; holding (no publish).");
          last_selected_ = -3;
        }
      }
    }

    if (publish) {
      pub_->publish(out);
    }
  }

private:
  // 파라미터
  std::vector<std::string> input_topics_;
  std::vector<int> priorities_;
  std::vector<int> timeouts_ms_;
  std::string output_topic_;
  double publish_rate_hz_;
  bool stop_on_idle_;

  // 내부
  std::vector<InputChannel> channels_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int last_selected_{-1};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TwistMuxNode>());
  rclcpp::shutdown();
  return 0;
}
