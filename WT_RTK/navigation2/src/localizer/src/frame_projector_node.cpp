#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// 条件编译：兼容所有 ROS 2 版本（Humble 推荐）
#ifdef TF2_CPP_HEADERS
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#endif

// tf2 内部头文件
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class FrameProjectorNode : public rclcpp::Node {
public:
    FrameProjectorNode() : Node("frame_projector_node") {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),  // 20Hz
            std::bind(&FrameProjectorNode::timerCallback, this)
        );
        RCLCPP_INFO(this->get_logger(), "Frame Projector Node 启动，广播 map → base_link");
    }

private:
    void timerCallback() {
        geometry_msgs::msg::TransformStamped t;
        try {
            t = tf_buffer_->lookupTransform("map", "body", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "等待 map→body TF: %s", ex.what());
            return;
        }

        // 提取 yaw
        tf2::Quaternion q;
        tf2::fromMsg(t.transform.rotation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        // 构造水平姿态（仅保留 yaw）
        tf2::Quaternion q_level;
        q_level.setRPY(0.0, 0.0, yaw);

        // 构造 base_link 变换
        geometry_msgs::msg::TransformStamped t_projected;
        t_projected.header.stamp = this->now();
        t_projected.header.frame_id = "map";
        t_projected.child_frame_id = "base_link";
        t_projected.transform.translation.x = t.transform.translation.x;
        t_projected.transform.translation.y = t.transform.translation.y;
        t_projected.transform.translation.z = 0.0;  // 固定地面高度
        t_projected.transform.rotation = tf2::toMsg(q_level);

        tf_broadcaster_->sendTransform(t_projected);
    }

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrameProjectorNode>());
    rclcpp::shutdown();
    return 0;
}