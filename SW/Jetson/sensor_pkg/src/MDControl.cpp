#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <thread>
#include <vector>
#include <algorithm>

using std::placeholders::_1;

// ===== UART 기본 설정 =====
static constexpr const char* DEFAULT_UART_PORT = "/dev/ttyCH341USB2";
static constexpr speed_t     DEFAULT_BAUD      = B115200;

// ===== 피드백 프레임 정의 (ESP32 → Jetson) =====
static constexpr uint8_t FB_HDR0      = 0xAA;
static constexpr uint8_t FB_HDR1      = 0x55;
static constexpr uint8_t FB_TYPE_VEL  = 0x20;   // linear_x, angular_z, dt_ms, seq
static constexpr uint8_t FB_VEL_LEN   = 12;     // payload 길이

// ===== 모터 명령 프레임 정의 (Jetson → ESP32) =====
class MDControl {
public:
  using VelFeedbackCb = std::function<void(float lin, float ang, uint16_t dt_ms, uint16_t seq)>;

  MDControl(const std::string& port, rclcpp::Logger logger, VelFeedbackCb fb_cb)
  : logger_(logger), port_(port), fb_cb_(std::move(fb_cb)), fd_(-1), stop_(false)
  {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd_ < 0) {
      RCLCPP_ERROR(logger_, "UART 포트 열기 실패: %s", port_.c_str());
      return;
    }
    fcntl(fd_, F_SETFL, 0); // 블로킹 모드
    if (configure()) {
      RCLCPP_INFO(logger_, "UART 연결 완료 (포트=%s, 115200-8N1)", port_.c_str());
      rx_thread_ = std::thread(&MDControl::rxLoop, this);
    } else {
      ::close(fd_); fd_ = -1;
    }
  }

  ~MDControl() {
    stop_ = true;
    if (rx_thread_.joinable()) rx_thread_.join();
    if (fd_ >= 0) ::close(fd_);
  }

  // ===== 기존: /cmd_vel → 모터 드라이버 패킷 송신 (원형 유지) =====
  void sendSpeed(double linear_x, double angular_z) {
    // -1.0~1.0 스케일 가정 → 0~255로 매핑 (중립 127)
    int left_speed  = static_cast<int>((linear_x - angular_z) * 50.0 + 127.0);
    int right_speed = static_cast<int>((linear_x + angular_z) * 50.0 + 127.0);

    left_speed  = std::clamp(left_speed,  0, 255);
    right_speed = std::clamp(right_speed, 0, 255);

    sendPacket(/*base_address=*/0, /*is_right=*/false, static_cast<uint8_t>(left_speed));
    sendPacket(/*base_address=*/0, /*is_right=*/true,  static_cast<uint8_t>(right_speed));

    RCLCPP_DEBUG(logger_, "TX cmd: left=%d, right=%d", left_speed, right_speed);
  }

private:
  rclcpp::Logger logger_;
  std::string    port_;
  VelFeedbackCb  fb_cb_;

  int            fd_;
  std::thread    rx_thread_;
  std::atomic<bool> stop_;
  std::vector<uint8_t> rxbuf_;

  bool configure() {
    struct termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      RCLCPP_ERROR(logger_, "시리얼 속성 가져오기 실패");
      return false;
    }

    cfsetospeed(&tty, DEFAULT_BAUD);
    cfsetispeed(&tty, DEFAULT_BAUD);

    // 115200-8N1, 로컬, 수신 허용
    tty.c_cflag = CS8 | CLOCAL | CREAD;
    tty.c_iflag = IGNPAR; // 패리티 에러 무시
    tty.c_oflag = 0;
    tty.c_lflag = 0;      // raw

    // read 타임아웃: 0.1s 단위 (1 = 0.1s), 최소 바이트수 0
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;

    tcflush(fd_, TCIFLUSH);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(logger_, "시리얼 속성 적용 실패");
      return false;
    }
    return true;
  }

  void sendPacket(uint8_t base_address, bool is_right_motor, uint8_t speed) {
    if (fd_ < 0) return;

    const uint8_t header   = 0x55;
    uint8_t address        = (base_address & 0x07);
    if (is_right_motor) address |= 0x08;
    const uint8_t command  = speed;
    const uint8_t checksum = static_cast<uint8_t>(header + address + command);

    uint8_t pkt[4] = { header, address, command, checksum };
    ssize_t n = ::write(fd_, pkt, 4);
    if (n != 4) {
      RCLCPP_WARN(logger_, "UART 부분 전송: %zd/4", n);
    }
  }

  static inline uint8_t sum8(const uint8_t* p, size_t n) {
    uint32_t s = 0;
    for (size_t i = 0; i < n; ++i) s += p[i];
    return static_cast<uint8_t>(s & 0xFF);
  }

  void rxLoop() {
    rxbuf_.reserve(256);

    while (!stop_) {
      uint8_t tmp[64];
      ssize_t n = ::read(fd_, tmp, sizeof(tmp));
      if (n > 0) {
        rxbuf_.insert(rxbuf_.end(), tmp, tmp + n);
        parseFrames();
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
  }

  void parseFrames() {
    size_t i = 0;
    while (rxbuf_.size() - i >= 4) {
      // 헤더 검색
      if (rxbuf_[i] != FB_HDR0 || rxbuf_[i+1] != FB_HDR1) { ++i; continue; }
      if (rxbuf_.size() - i < 5) break; // type/len 확인 전 최소 길이 체크

      const uint8_t type = rxbuf_[i+2];
      const uint8_t len  = rxbuf_[i+3];
      const size_t  frame_len = 2 + 1 + 1 + len + 1;
      if (rxbuf_.size() - i < frame_len) break; // 프레임 미완성

      // 체크섬
      const uint8_t calc = sum8(&rxbuf_[i+2], 1 + 1 + len);
      const uint8_t recv = rxbuf_[i + frame_len - 1];
      if (calc != recv) {
        RCLCPP_WARN(logger_, "피드백 체크섬 불일치: calc=%u recv=%u (frame dropped)", calc, recv);
        i += 2; // 헤더만 넘겨 재탐색
        continue;
      }

      const uint8_t* payload = &rxbuf_[i+4];

      if (type == FB_TYPE_VEL && len == FB_VEL_LEN) {
        float    lin = 0.0f, ang = 0.0f;
        uint16_t dt_ms = 0, seq = 0;

        // 리틀엔디안 float/uint16 파싱
        std::memcpy(&lin, &payload[0], 4);
        std::memcpy(&ang, &payload[4], 4);
        dt_ms = static_cast<uint16_t>(payload[8]  | (payload[9]  << 8));
        seq   = static_cast<uint16_t>(payload[10] | (payload[11] << 8));

        if (fb_cb_) fb_cb_(lin, ang, dt_ms, seq);
      } else {
        RCLCPP_DEBUG(logger_, "알 수 없는 type=0x%02X len=%u", type, len);
      }

      i += frame_len; // 다음 프레임으로
    }

    if (i > 0) {
      rxbuf_.erase(rxbuf_.begin(), rxbuf_.begin() + i);
    }
  }
};

// ===== ROS2 노드 =====
class MDControlNode : public rclcpp::Node {
public:
  MDControlNode() : Node("md_control") {
    // UART 포트 파라미터로 오버라이드 가능
    uart_port_ = this->declare_parameter<std::string>("uart_port", DEFAULT_UART_PORT);

    // 퍼블리셔: ESP32에서 계산한 속도 → /encoder_vel
    pub_encoder_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/encoder_vel", 10);

    // 구독자: /cmd_vel
    sub_cmd_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&MDControlNode::onCmdVel, this, _1));

    // MDControl 생성
    md_.reset(new MDControl(
      uart_port_,
      this->get_logger(),
      std::bind(&MDControlNode::onVelFeedback, this, std::placeholders::_1,
                                              std::placeholders::_2,
                                              std::placeholders::_3,
                                              std::placeholders::_4)));

    RCLCPP_INFO(this->get_logger(),
      "MDControl Node ready. (/cmd_vel → UART TX 유지, ESP32 속도 피드백 → /encoder_vel)");
  }

private:
  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr    pub_encoder_vel_;

  // UART
  std::unique_ptr<MDControl> md_;
  std::string uart_port_;

  uint16_t last_seq_ = 0xFFFF;
  uint64_t loss_cnt_ = 0;
  uint64_t dup_cnt_  = 0;
  uint64_t reset_cnt_= 0;

  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!md_) return;
    md_->sendSpeed(msg->linear.x, msg->angular.z);
  }

  static inline uint16_t mod16_diff(uint16_t a, uint16_t b) {
    return static_cast<uint16_t>(a - b);
  }

  void handleSeq(uint16_t seq) {
    if (last_seq_ == 0xFFFF) { last_seq_ = seq; return; }
    const uint16_t delta = mod16_diff(seq, last_seq_);

    if (delta == 0) {
      ++dup_cnt_;
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "DUP packet seq=%u (dup=%llu)", seq, static_cast<unsigned long long>(dup_cnt_));
      return;
    }

    if (delta < 32768) {
      if (delta > 1) {
        loss_cnt_ += (delta - 1);
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "LOSS detected: missed %u packets (last=%u, now=%u, total_loss=%llu)",
                             delta - 1, last_seq_, seq, static_cast<unsigned long long>(loss_cnt_));
      }
      last_seq_ = seq;
      return;
    }

    ++reset_cnt_;
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "SEQ reset/out-of-order (last=%u, now=%u, resets=%llu)",
                         last_seq_, seq, static_cast<unsigned long long>(reset_cnt_));
    last_seq_ = seq;
  }

  // ESP32 속도 피드백 수신시 호출
  void onVelFeedback(float lin_mps, float ang_rps, uint16_t dt_ms, uint16_t seq) {
    handleSeq(seq);

    geometry_msgs::msg::Twist tw;
    tw.linear.x  = static_cast<double>(lin_mps); // [m/s]
    tw.angular.z = static_cast<double>(ang_rps); // [rad/s]
    pub_encoder_vel_->publish(tw);

    RCLCPP_DEBUG(this->get_logger(), "RX vel: v=%.3f m/s, w=%.3f rad/s, dt=%u ms, seq=%u",
                 lin_mps, ang_rps, dt_ms, seq);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MDControlNode>());
  rclcpp::shutdown();
  return 0;
}
