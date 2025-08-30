#include <memory>
#include <vector>
#include <chrono>
#include <string>
#include <cerrno>
#include <cstring>

// ROS2 / MoveIt
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

// POSIX UART
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>

using namespace std::chrono_literals;

struct JointWaypoint {
  double j1_rad;
  double j2_rad;
  double j3_rad;
  double grip;
};

// 간단한 UART 헬퍼
class Uart {
public:
  Uart() : fd_(-1) {}

  bool open_port(const std::string& dev, int baudrate, rclcpp::Logger logger) {
    // 열기
    fd_ = ::open(dev.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_ERROR(logger, "시리얼 포트 열기 실패: %s (%d) : %s", dev.c_str(), errno, std::strerror(errno));
      return false;
    }

    // 블로킹 모드로 전환 (read/write 대기 허용)
    int flags = fcntl(fd_, F_GETFL, 0);
    fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);

    struct termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
      RCLCPP_ERROR(logger, "tcgetattr 실패: %s", std::strerror(errno));
      ::close(fd_); fd_ = -1;
      return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= CLOCAL | CREAD;  // 로컬, 수신허용
    tio.c_cflag &= ~CRTSCTS;        // 하드웨어 플로우 제어 끔
    tio.c_cflag &= ~PARENB;         // 패리티 없음
    tio.c_cflag &= ~CSTOPB;         // 1 stop bit
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;             // 8 bit

    speed_t speed = B115200;
    switch (baudrate) {
      case 9600: speed = B9600; break;
      case 19200: speed = B19200; break;
      case 38400: speed = B38400; break;
      case 57600: speed = B57600; break;
      case 115200: speed = B115200; break;
      default: RCLCPP_WARN(logger, "지원하지 않는 baud %d, 115200으로 설정", baudrate);
    }
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 1; // read timeout = 0.1s 단위

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
      RCLCPP_ERROR(logger, "tcsetattr 실패: %s", std::strerror(errno));
      ::close(fd_); fd_ = -1;
      return false;
    }

    // 입력버퍼 비우기
    tcflush(fd_, TCIFLUSH);
    RCLCPP_INFO(logger, "시리얼 포트 오픈 완료: %s, %d bps", dev.c_str(), baudrate);
    return true;
  }

  bool write_line(const std::string& line, int timeout_ms, rclcpp::Logger logger) {
    if (fd_ < 0) return false;

    // poll로 쓰기가능 대기
    struct pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLOUT;

    int pr = poll(&pfd, 1, timeout_ms);
    if (pr <= 0) {
      RCLCPP_ERROR(logger, "시리얼 쓰기 poll 타임아웃/에러: %d", pr);
      return false;
    }

    const char* buf = line.c_str();
    size_t total = 0;
    size_t len = line.size();

    while (total < len) {
      ssize_t w = ::write(fd_, buf + total, len - total);
      if (w < 0) {
        if (errno == EAGAIN || errno == EINTR) continue;
        RCLCPP_ERROR(logger, "시리얼 write 실패: %s", std::strerror(errno));
        return false;
      }
      total += static_cast<size_t>(w);
    }
    return true;
  }

  void close_port() {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

  ~Uart() { close_port(); }

private:
  int fd_;
};

class MovePointCmdNode : public rclcpp::Node {
public:
  MovePointCmdNode()
  : Node("move_point_cmd_node"),
    move_group_(std::make_shared<rclcpp::Node>(this->get_name()), "hand")
  {
    // ==== 파라미터 ====
    this->declare_parameter<std::string>("serial_port", "/dev/ttyCH341USB3");
    this->declare_parameter<int>("baud_rate", 115200);
    this->declare_parameter<int>("write_timeout_ms", 1000);
    this->declare_parameter<std::string>("trig_payload", "TRIG\n");
    this->declare_parameter<std::string>("done_payload", "DONE\n");
    this->declare_parameter<std::string>("fail_payload", "FAIL\n");

    this->get_parameter("serial_port", serial_port_);
    this->get_parameter("baud_rate", baud_rate_);
    this->get_parameter("write_timeout_ms", write_timeout_ms_);
    this->get_parameter("trig_payload", trig_payload_);
    this->get_parameter("done_payload", done_payload_);
    this->get_parameter("fail_payload", fail_payload_);

    // 퍼블리셔
    gripper_pub_ = this->create_publisher<std_msgs::msg::Float32>("/gripper_cmd", 10);

    rclcpp::QoS qos(1);
    qos.transient_local();
    qos.reliable();
    cleaning_done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/cleaning_done", qos);

    // MoveIt 조인트 로그
    const auto names = move_group_.getJointNames();
    std::string joined;
    for (size_t i=0;i<names.size();++i){ joined += (i? ", ":"") + names[i]; }
    RCLCPP_INFO(this->get_logger(), "MoveIt group 'hand' joints: [%s]", joined.c_str());

    // UART 오픈
    if (!uart_.open_port(serial_port_, baud_rate_, this->get_logger())) {
      RCLCPP_WARN(this->get_logger(), "UART 오픈 실패. 이후 TRIG 전송은 시도하되, 실패할 수 있습니다.");
    }

    // 서비스 서버
    server_ = this->create_service<std_srvs::srv::Trigger>(
      "/table_cleaning_service",
      std::bind(&MovePointCmdNode::handle_service, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Trigger 서비스 준비 완료: /table_cleaning_service");
  }

private:
  bool go_to_joint(double j1, double j2, double j3) {
    std::vector<double> target = { /*shoulder=*/j2, /*base=*/j1, /*elbow=*/j3 };
    move_group_.setJointValueTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto ok_plan = (move_group_.plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!ok_plan) {
      RCLCPP_ERROR(this->get_logger(), "경로 계획 실패 (j1=%.6f, j2=%.6f, j3=%.6f)", j1, j2, j3);
      return false;
    }
    auto ok_exec = (move_group_.execute(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    if (!ok_exec) {
      RCLCPP_ERROR(this->get_logger(), "경로 실행 실패 (j1=%.6f, j2=%.6f, j3=%.6f)", j1, j2, j3);
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "이동 완료 (mapped: shoulder=%.6f, base=%.6f, elbow=%.6f)", j2, j1, j3);
    return true;
  }

  void send_gripper(double grip) {
    std_msgs::msg::Float32 m; m.data = static_cast<float>(grip);
    gripper_pub_->publish(m);
    RCLCPP_INFO(this->get_logger(), "그리퍼 명령 전송: %.6f", grip);
  }

  void publish_cleaning_done(bool done) {
    std_msgs::msg::Bool msg; msg.data = done;
    cleaning_done_pub_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "publish /cleaning_done: %s", done ? "true" : "false");
  }

  bool uart_send_line(const std::string& line) {
    if (line.empty()) return true;
    bool ok = uart_.write_line(line, write_timeout_ms_, this->get_logger());
    if (!ok) RCLCPP_ERROR(this->get_logger(), "UART 전송 실패(내용='%s')", line.c_str());
    else     RCLCPP_INFO(this->get_logger(),  "UART 전송 성공(내용='%s')", line.c_str());
    return ok;
  }

  void handle_service(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    // 1) 서비스 수신: ESP32로 TRIG 전송
    (void)uart_send_line(trig_payload_);

    const std::vector<JointWaypoint> path = {
	{ -1.0317746911810726,  0.11533195528617422, 0.7856700207837095, 0.5293275422332511 },
	{ -1.3120296121792461,  0.11535326698646935, 0.6890476846950421, 0.5293275422332511 },
	{ -1.1119995177395339,  0.11540123872181221, 1.2921773316575622, 0.7898394828271714 },
	{ -0.9320669051030011,  0.11520326198184271, 1.5437059227160639, 1.0503321451353864 },
	{ -0.8825907847299382,  0.11525576719985543, 1.8622897448252898, 1.2142097946646021 },
	{ -0.7480224878372769,  0.11537572865402282, 2.1194052802519048, 1.3906722378685417 },
	{ -0.7644380794428693,  0.11535089542251352, 2.2952435948017671, 1.5999021376146005 },
	{ -0.7887440081710164, -0.14213307748046816, 2.3070661365236171, 1.6099021376146005 },
	
	{ -0.5999880460191186, -0.07487215430963266, 2.3087660419199381, 1.1099021376146005 },
	
	{ -0.9821024430386251, -0.30417394929832761, 0.8267634967080942, 0.5841196404739997 },
	{ -1.2192024430386251, -0.30952394929832761, 0.8189634967080942, 0.5841196404739997 },
	{ -1.0675283420139671, -0.30956476974007421, 1.1736601971649121, 0.7814149703528883 },
	{ -0.9170119305913726, -0.30960788687435437, 1.5638419783080424, 1.0167337455955632 },
	{ -0.8064461986764508, -0.30956262596436823, 1.8964613609437713, 1.2267931745053111 },
	{ -0.7866303965351114, -0.30945822190634625, 2.1115909959651811, 1.4160267642107147 },
	{ -0.7767987844318512, -0.30947964900636915, 2.3120888241588053, 1.5999048897067084 },
	{ -0.7767987844318512, -0.30947964900636915, 2.3120888241588053, 1.6099048897067084 },
	{ -0.7568987844318512, -0.12489864900636915, 2.3328698241588053, 1.6099048897067084 },
	
	{ -0.7568987844318512, -0.12489864900636915, 2.3328698241588053, 1.0099048897067084 },
	{ -0.6314987844318512, -0.07249864900636915, 2.5921698241588053, 0.7353048897067084 },
	{  0.0585987844318512, -0.07244864900636915, 2.9152898241588053, 1.2267048897067084 },
    };

    for (size_t i=0; i<path.size(); ++i) {
      RCLCPP_INFO(this->get_logger(),
                  "[%zu/%zu] 목표: j=[%.6f, %.6f, %.6f], grip=%.6f",
                  i+1, path.size(),
                  path[i].j1_rad, path[i].j2_rad, path[i].j3_rad, path[i].grip);

      if (!go_to_joint(path[i].j1_rad, path[i].j2_rad, path[i].j3_rad)) {
        res->success = false;
        res->message = "경로 계획/실행 실패 (index " + std::to_string(i) + ")";
        publish_cleaning_done(false);
        (void)uart_send_line(fail_payload_);
        return;
      }

      send_gripper(path[i].grip);
      rclcpp::sleep_for(500ms);
    }

    res->success = true;
    res->message = "주어진 좌표 " + std::to_string(path.size()) + "개 이동 및 그리퍼 제어 완료";
    publish_cleaning_done(true);
    (void)uart_send_line(done_payload_);
  }

private:
  // UART 파라미터
  std::string serial_port_;
  int baud_rate_{115200};
  int write_timeout_ms_{1000};
  std::string trig_payload_{"TRIG\n"};
  std::string done_payload_{"DONE\n"};
  std::string fail_payload_{"FAIL\n"};

  // ROS
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr    cleaning_done_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr   server_;
  moveit::planning_interface::MoveGroupInterface       move_group_;

  // UART
  Uart uart_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MovePointCmdNode>());
  rclcpp::shutdown();
  return 0;
}

