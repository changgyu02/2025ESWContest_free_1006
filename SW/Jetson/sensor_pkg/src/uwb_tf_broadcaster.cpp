#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

class UwbTFBroadcaster : public rclcpp::Node {
public:
    UwbTFBroadcaster() : Node("uwb_tf_broadcaster") {
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "uwb_pose", 10, std::bind(&UwbTFBroadcaster::pose_callback, this, std::placeholders::_1));

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        rclcpp::Time now = this->get_clock()->now();

        // 1. map → odom
        geometry_msgs::msg::TransformStamped tf_map_to_odom;
        tf_map_to_odom.header.stamp = now;
        tf_map_to_odom.header.frame_id = "map";
        tf_map_to_odom.child_frame_id = "odom";
        tf_map_to_odom.transform.translation.x = 0.0;
        tf_map_to_odom.transform.translation.y = 0.0;
        tf_map_to_odom.transform.translation.z = 0.0;
        tf_map_to_odom.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(tf_map_to_odom);

        // 2. odom → base_link (실제 UWB 값)
        geometry_msgs::msg::TransformStamped tf_odom_to_base;
        tf_odom_to_base.header.stamp = now;
        tf_odom_to_base.header.frame_id = "odom";
        tf_odom_to_base.child_frame_id = "base_link";
        tf_odom_to_base.transform.translation.x = msg->pose.position.x;
        tf_odom_to_base.transform.translation.y = msg->pose.position.y;
        tf_odom_to_base.transform.translation.z = msg->pose.position.z;
        tf_odom_to_base.transform.rotation = msg->pose.orientation;

        tf_broadcaster_->sendTransform(tf_odom_to_base);

        RCLCPP_INFO(this->get_logger(), "TF (map→odom) + (odom→base_link) 발행됨: x=%.2f y=%.2f",
                    msg->pose.position.x, msg->pose.position.y);
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UwbTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}

