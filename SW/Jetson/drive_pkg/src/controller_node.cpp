#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

class ControllerNode : public rclcpp::Node {
public:
    ControllerNode() : Node("controller_node") {
        // --- Parameters (tunable at runtime) ---
        lookahead_dist_      = this->declare_parameter("lookahead_dist",      0.30); // [m]
        max_lin_speed_       = this->declare_parameter("max_lin_speed",       1.0); // [m/s]
        max_ang_speed_       = this->declare_parameter("max_ang_speed",       1.20); // [rad/s]
        ang_kp_align_        = this->declare_parameter("ang_kp_align",        1.50); // FinalAlign / Pivot P gain
        yaw_thresh_deg_      = this->declare_parameter("yaw_thresh_deg",      10.0); // done if |err| < 10 deg (FinalAlign)
        align_hold_sec_      = this->declare_parameter("align_hold_sec",      2.0);  // must hold for 2s (FinalAlign)
        control_rate_hz_     = this->declare_parameter("control_rate_hz",     20.0); // control loop
        alpha_deadband_deg_  = this->declare_parameter("alpha_deadband_deg",  15.0); // small heading -> straight
        min_w_align_         = this->declare_parameter("min_w_align",         0.20); // overcome static friction [rad/s]

        // ▶ Pivot(제자리 회전) 히스테리시스 파라미터 추가
        pivot_enter_deg_     = this->declare_parameter("pivot_enter_deg",    100.0); // 이 값보다 크면 TRACK→PIVOT
        pivot_exit_deg_      = this->declare_parameter("pivot_exit_deg",      10.0); // 이 값보다 작으면 PIVOT→TRACK

        yaw_thresh_rad_       = yaw_thresh_deg_ * M_PI / 180.0;
        alpha_deadband_rad_   = alpha_deadband_deg_ * M_PI / 180.0;
        control_period_       = 1.0 / std::max(1.0, control_rate_hz_);
        align_hold_ticks_req_ = static_cast<int>(align_hold_sec_ / control_period_);
        if (align_hold_ticks_req_ < 1) align_hold_ticks_req_ = 1;

        pivot_enter_rad_ = pivot_enter_deg_ * M_PI / 180.0;
        pivot_exit_rad_  = pivot_exit_deg_  * M_PI / 180.0;

        // --- Subscriptions & Publications ---
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planned_path", 10, std::bind(&ControllerNode::path_cb, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 50, std::bind(&ControllerNode::odom_cb, this, std::placeholders::_1));

        obstacle_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/obstacle_detected", 10, std::bind(&ControllerNode::obstacle_cb, this, std::placeholders::_1));

        position_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/position_done", 10, std::bind(&ControllerNode::position_done_cb, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10, std::bind(&ControllerNode::goal_cb, this, std::placeholders::_1));

        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // --- Control loop timer ---
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(control_period_),
            std::bind(&ControllerNode::control_loop, this));

        RCLCPP_INFO(this->get_logger(),
                    "controller_node started (lookahead=%.2f, max v=%.2f, max w=%.2f, rate=%.1f Hz, pivot_enter=%.1f deg, pivot_exit=%.1f deg)",
                    lookahead_dist_, max_lin_speed_, max_ang_speed_, control_rate_hz_, pivot_enter_deg_, pivot_exit_deg_);
    }

private:
    // ------------ ROS I/O ------------
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr obstacle_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr position_done_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ------------ State ------------
    nav_msgs::msg::Path path_;
    bool has_path_{false};

    nav_msgs::msg::Odometry odom_;
    bool has_odom_{false};

    geometry_msgs::msg::PoseStamped goal_pose_;
    bool has_goal_pose_{false};

    bool obstacle_{false};

    enum class Mode { TRACK, FINAL_ALIGN, STOP };
    Mode mode_{Mode::STOP}; // 시작은 STOP, 경로가 오면 TRACK로 전환

    // Pivot 상태 플래그
    bool pivoting_{false};

    // ------------ Params & control vars ------------
    double lookahead_dist_;
    double max_lin_speed_;
    double max_ang_speed_;
    double ang_kp_align_;
    double yaw_thresh_deg_;
    double yaw_thresh_rad_;
    double align_hold_sec_;
    double control_rate_hz_;
    double control_period_;
    double alpha_deadband_deg_;
    double alpha_deadband_rad_;
    double min_w_align_;

    // Pivot 파라미터
    double pivot_enter_deg_;
    double pivot_exit_deg_;
    double pivot_enter_rad_;
    double pivot_exit_rad_;

    int    align_hold_ticks_req_{40}; // derived
    int    align_hold_ticks_{0};

    // ------------ Callbacks ------------
    void path_cb(const nav_msgs::msg::Path::SharedPtr msg) {
        path_ = *msg;
        has_path_ = !path_.poses.empty();

        // 새 경로가 들어오고 FINAL_ALIGN 중이 아니면 TRACK으로 전환
        if (has_path_ && mode_ != Mode::FINAL_ALIGN) {
            mode_ = Mode::TRACK;
            pivoting_ = false;
            align_hold_ticks_ = 0;
            RCLCPP_INFO(this->get_logger(), "new path -> TRACK (poses=%zu)", path_.poses.size());
        } else {
            RCLCPP_INFO(this->get_logger(), "planned_path received: %zu poses (mode kept)", path_.poses.size());
        }
    }

    void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_ = *msg;
        has_odom_ = true;
    }

    void obstacle_cb(const std_msgs::msg::Bool::SharedPtr msg) {
        obstacle_ = msg->data;
        if (obstacle_) {
            publish_zero();
            RCLCPP_WARN(this->get_logger(), "Obstacle detected -> STOP");
            mode_ = Mode::STOP;
            pivoting_ = false;
        } else {
            RCLCPP_INFO(this->get_logger(), "Obstacle cleared");
        }
    }

    void position_done_cb(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data) {
            mode_ = Mode::FINAL_ALIGN;
            pivoting_ = false;
            align_hold_ticks_ = 0;
            RCLCPP_INFO(this->get_logger(), "/position_done true -> FINAL_ALIGN");
        }
    }

    void goal_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        goal_pose_ = *msg;
        has_goal_pose_ = true;
    }

    // ------------ Control loop ------------
    void control_loop() {
        if (!has_odom_) {
            publish_zero();
            return;
        }
        if (obstacle_) {
            publish_zero();
            return;
        }

        switch (mode_) {
            case Mode::TRACK:       track_step();       break;
            case Mode::FINAL_ALIGN: final_align_step(); break;
            case Mode::STOP:        publish_zero();     break;
        }
    }

    // TRACK: Pure Pursuit (+ Pivot 분기)  ←★★ 여기만 수정 ★★
    void track_step() {
        if (!has_path_) {
            publish_zero();
            return;
        }

        const auto &p = odom_.pose.pose.position;
        const auto &q = odom_.pose.pose.orientation;
        double yaw = yaw_from_quat(q);

        geometry_msgs::msg::Point target_pt;
        if (!get_lookahead_target(p.x, p.y, lookahead_dist_, target_pt)) {
            publish_zero();
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[TRACK] no lookahead target -> stop (waiting /position_done or new path)");
            return;
        }

        double heading   = std::atan2(target_pt.y - p.y, target_pt.x - p.x);
        double alpha     = normalize_angle(heading - yaw);
        double abs_alpha = std::fabs(alpha);

        // --- Pivot hysteresis ---
        if (!pivoting_ && abs_alpha > pivot_enter_rad_) {
            pivoting_ = true;
            RCLCPP_INFO(this->get_logger(), "[TRACK->PIVOT] abs(alpha)=%.1f deg > %.1f deg",
                        abs_alpha * 180.0 / M_PI, pivot_enter_deg_);
        } else if (pivoting_ && abs_alpha < pivot_exit_rad_) {
            pivoting_ = false;
            RCLCPP_INFO(this->get_logger(), "[PIVOT->TRACK] abs(alpha)=%.1f deg < %.1f deg",
                        abs_alpha * 180.0 / M_PI, pivot_exit_deg_);
        }

        // -------------------------
        // 부드러운 가감속(램핑) 적용 (track_step 내부에서만)
        // -------------------------
        // 램핑 상태(직전 명령)를 보관: 멤버 추가 없이 static 지역변수 사용
        static double last_v = 0.0;
        static double last_w = 0.0;

        // 가감속 한계(멤버/파라미터 추가 없이, 여기서만 사용)
        const double dt = control_period_;
        const double LIN_ACCEL = 0.50; // m/s^2
        const double LIN_DECEL = 0.80; // m/s^2 (감속은 더 크게)
        const double ANG_ACCEL = 3.0; // rad/s^2
        const double ANG_DECEL = 2.50; // rad/s^2

        auto slew_to = [&](double target, double current,
                           double accel_limit, double decel_limit) -> double {
            double delta = target - current;
            if (delta > 0.0) {
                double max_up = accel_limit * dt;
                if (delta > max_up) delta = max_up;
            } else {
                double max_down = decel_limit * dt;
                if (delta < -max_down) delta = -max_down;
            }
            return current + delta;
        };

        auto clamp_vw = [&](double &v, double &w){
            v = std::clamp(v, -max_lin_speed_, max_lin_speed_);
            w = std::clamp(w, -max_ang_speed_, max_ang_speed_);
        };

        // --- Pivot mode: in-place spin (v_target=0), w_target = 기존 로직
        if (pivoting_) {
            double w = ang_kp_align_ * alpha;
            if (std::fabs(alpha) > (2.0 * M_PI / 180.0) && std::fabs(w) < min_w_align_) {
                w = (w >= 0.0) ? min_w_align_ : -min_w_align_;
            }
            double v_target = 0.0;
            double w_target = std::clamp(w, -max_ang_speed_, max_ang_speed_);

            // 램핑 적용
            clamp_vw(v_target, w_target);
            last_v = slew_to(v_target, last_v, LIN_ACCEL, LIN_DECEL);
            last_w = slew_to(w_target, last_w, ANG_ACCEL, ANG_DECEL);

            geometry_msgs::msg::Twist cmd;
            cmd.linear.x  = last_v;
            cmd.angular.z = last_w;
            cmd_pub_->publish(cmd);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[TRACK-PIVOT] alpha=%.1f deg, w_tgt=%.2f | v=%.2f, w=%.2f",
                alpha * 180.0 / M_PI, w_target, last_v, last_w);
            return;
        }

        // --- Normal Pure Pursuit ---
        if (std::fabs(alpha) < alpha_deadband_rad_) alpha = 0.0;

        double curvature = (lookahead_dist_ > 1e-6) ? (2.0 * std::sin(alpha) / lookahead_dist_) : 0.0;
        double w_target = std::clamp(max_lin_speed_ * curvature, -max_ang_speed_, max_ang_speed_);
        double v_target = max_lin_speed_;

        // 램핑 적용
        clamp_vw(v_target, w_target);
        last_v = slew_to(v_target, last_v, LIN_ACCEL, LIN_DECEL);
        last_w = slew_to(w_target, last_w, ANG_ACCEL, ANG_DECEL);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = last_v;
        cmd.angular.z = last_w;
        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[TRACK] alpha=%.1f deg, v_tgt=%.2f, w_tgt=%.2f | v=%.2f, w=%.2f",
            alpha * 180.0 / M_PI, v_target, w_target, last_v, last_w);
    }

    // FINAL_ALIGN: in-place spin for differential drive (linear.x=0)
    void final_align_step() {
        if (!has_goal_pose_) {
            publish_zero();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                 "[FINAL_ALIGN] goal_pose not available");
            return;
        }

        const auto &q_cur = odom_.pose.pose.orientation;
        double yaw_cur   = yaw_from_quat(q_cur);
        double yaw_goal  = yaw_from_quat(goal_pose_.pose.orientation);
        double yaw_err   = normalize_angle(yaw_goal - yaw_cur);

        double w = ang_kp_align_ * yaw_err;

        if (std::fabs(yaw_err) > (2.0 * M_PI / 180.0) && std::fabs(w) < min_w_align_) {
            w = (w >= 0.0) ? min_w_align_ : -min_w_align_;
        }

        w = std::clamp(w, -max_ang_speed_, max_ang_speed_);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = 0.0;
        cmd.angular.z = w;
        cmd_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "[FINAL_ALIGN] yaw_err=%.2f deg, w=%.2f",
            yaw_err * 180.0 / M_PI, w);

        // Done if |yaw_err| < threshold, held for align_hold_sec
        if (std::fabs(yaw_err) < yaw_thresh_rad_) {
            if (++align_hold_ticks_ >= align_hold_ticks_req_) {
                publish_zero();
                mode_ = Mode::STOP;
                RCLCPP_INFO(this->get_logger(), "[FINAL_ALIGN] done -> STOP");
            }
        } else {
            align_hold_ticks_ = 0;
        }
    }

    // ------------ Helpers ------------
    static double normalize_angle(double a) {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    static double yaw_from_quat(const geometry_msgs::msg::Quaternion &qmsg) {
        tf2::Quaternion q;
        tf2::fromMsg(qmsg, q);
        double r, p, y;
        tf2::Matrix3x3(q).getRPY(r, p, y);
        return y;
    }

    bool get_lookahead_target(double x, double y, double Ld, geometry_msgs::msg::Point &target) {
        if (!has_path_ || path_.poses.size() < 2) return false;

        size_t nearest_idx = 0;
        double nearest_dist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < path_.poses.size(); ++i) {
            const auto &pp = path_.poses[i].pose.position;
            double d = std::hypot(pp.x - x, pp.y - y);
            if (d < nearest_dist) { nearest_dist = d; nearest_idx = i; }
        }

        for (size_t i = nearest_idx; i < path_.poses.size(); ++i) {
            const auto &pp = path_.poses[i].pose.position;
            double d = std::hypot(pp.x - x, pp.y - y);
            if (d >= Ld) {
                target = pp;
                return true;
            }
        }

        target = path_.poses.back().pose.position;
        return true;
    }

    void publish_zero() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
