#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <communication/msg/motion_commands.hpp>

#include <memory>
#include <mutex>

using namespace std::chrono_literals;

class CmdVelToMotion : public rclcpp::Node {
public:
    CmdVelToMotion() : Node("cmd_vel_to_motion") {
        // 参数声明
        this->declare_parameter("mode", 1);                    // 默认 walk 模式
        this->declare_parameter("filter_alpha_xy", 0.03);      // x/y 滤波系数
        this->declare_parameter("filter_alpha_yaw", 0.05);     // yaw 滤波系数
        this->declare_parameter("dead_zone", 0.05);            // 死区 0.05 m/s
        this->declare_parameter("max_vel_x", 1.0);
        this->declare_parameter("min_vel_x", -0.5);
        this->declare_parameter("max_vel_y", 0.4);
        this->declare_parameter("min_vel_y", -0.4);
        this->declare_parameter("max_yaw_rate", 0.6);
        this->declare_parameter("min_yaw_rate", -0.6);

        // 读取参数
        mode_ = this->get_parameter("mode").as_int();
        alpha_xy_ = this->get_parameter("filter_alpha_xy").as_double();
        alpha_yaw_ = this->get_parameter("filter_alpha_yaw").as_double();
        dead_zone_ = this->get_parameter("dead_zone").as_double();
        max_vel_x_ = this->get_parameter("max_vel_x").as_double();
        min_vel_x_ = this->get_parameter("min_vel_x").as_double();
        max_vel_y_ = this->get_parameter("max_vel_y").as_double();
        min_vel_y_ = this->get_parameter("min_vel_y").as_double();
        max_yaw_ = this->get_parameter("max_yaw_rate").as_double();
        min_yaw_ = this->get_parameter("min_yaw_rate").as_double();

        // 初始化滤波值
        filtered_x_ = 0.0;
        filtered_y_ = 0.0;
        filtered_yaw_ = 0.0;

        // 订阅与发布
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&CmdVelToMotion::cmdVelCallback, this, std::placeholders::_1));

        motion_pub_ = this->create_publisher<communication::msg::MotionCommands>(
            "motion_commands", 10);

        // 20Hz 发布（与 controller_server 一致）
        timer_ = this->create_wall_timer(
            50ms, std::bind(&CmdVelToMotion::publishFilteredCommand, this));

        RCLCPP_INFO(this->get_logger(), "CmdVelToMotion node started with mode=%d", mode_);
    }

private:
    // 死区处理
    double applyDeadZone(double value, double dead_zone) {
        if (std::abs(value) < dead_zone) {
            return 0.0;
        }
        return value;
    }

    // 速度限幅
    double clamp(double value, double min_val, double max_val) {
        return std::max(min_val, std::min(max_val, value));
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);

        // 读取原始速度
        raw_x_ = msg->linear.x;
        raw_y_ = msg->linear.y;
        raw_yaw_ = msg->angular.z;

        // 死区
        raw_x_ = applyDeadZone(raw_x_, dead_zone_);
        raw_y_ = applyDeadZone(raw_y_, dead_zone_);
        raw_yaw_ = applyDeadZone(raw_yaw_, dead_zone_ * 2);  // yaw 死区稍大

        // 限幅
        raw_x_ = clamp(raw_x_, min_vel_x_, max_vel_x_);
        raw_y_ = clamp(raw_y_, min_vel_y_, max_vel_y_);
        raw_yaw_ = clamp(raw_yaw_, min_yaw_, max_yaw_);
    }

    void publishFilteredCommand() {
        std::lock_guard<std::mutex> lock(mutex_);

        // 一阶低通滤波
        filtered_x_ = raw_x_ * alpha_xy_ + filtered_x_ * (1.0 - alpha_xy_);
        filtered_y_ = raw_y_ * alpha_xy_ + filtered_y_ * (1.0 - alpha_xy_);
        filtered_yaw_ = raw_yaw_ * alpha_yaw_ + filtered_yaw_ * (1.0 - alpha_yaw_);

        auto motion_msg = communication::msg::MotionCommands();
        motion_msg.vel_des.x = filtered_x_;
        motion_msg.vel_des.y = filtered_y_;
        motion_msg.yawdot_des = filtered_yaw_;
        motion_msg.mode = mode_;

        // 其他字段默认 0（可扩展）
        motion_msg.btn_6 = 0;
        motion_msg.btn_7 = 0;
        // ... 其他 btn

        motion_pub_->publish(motion_msg);
    }

    // ROS2 接口
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<communication::msg::MotionCommands>::SharedPtr motion_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 参数
    int mode_;
    double alpha_xy_, alpha_yaw_;
    double dead_zone_;
    double max_vel_x_, min_vel_x_;
    double max_vel_y_, min_vel_y_;
    double max_yaw_, min_yaw_;

    // 状态变量（线程安全）
    std::mutex mutex_;
    double raw_x_ = 0.0, raw_y_ = 0.0, raw_yaw_ = 0.0;
    double filtered_x_ = 0.0, filtered_y_ = 0.0, filtered_yaw_ = 0.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelToMotion>());
    rclcpp::shutdown();
    return 0;
}
