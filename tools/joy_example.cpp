#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <thread>
#include <atomic>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class MotionCtrl : public rclcpp::Node
{
public:
    MotionCtrl() : Node("motion_ctrl")
    {
        // 1. 初始化手柄
        const char* device = "/dev/input/js0";
        joy_fd_ = open(device, O_RDONLY | O_NONBLOCK);
        if (joy_fd_ == -1) {
            RCLCPP_WARN(this->get_logger(), "无法打开手柄 %s，仅话题控制模式生效", device);
            use_joy_mode_ = false; // 打开失败则强制进入话题模式
        } else {
            RCLCPP_INFO(this->get_logger(), "成功连接手柄: %s", device);
            read_thread_ = std::thread(&MotionCtrl::read_joy_loop, this);
        }

        // 2. Pub & Sub (统一使用 TwistStamped)
        publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel_car", 10);
        
        subscription_stamped_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
            "/cmd_vel", 10, std::bind(&MotionCtrl::topic_callback, this, _1));

        // 3. 定y时器发布 (20Hz)
        timer_ = this->create_wall_timer(50ms, std::bind(&MotionCtrl::timer_callback, this));
    }

    ~MotionCtrl() {
        if (read_thread_.joinable()) read_thread_.join();
        if (joy_fd_ != -1) close(joy_fd_);
    }

private:
    void topic_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
    {
        // 存储话题速度
        v_x_topic_ = msg->twist.linear.x;
        v_w_topic_ = msg->twist.angular.z;
    }

    void timer_callback()
    {
        auto out_msg = geometry_msgs::msg::TwistStamped();
        out_msg.header.stamp = this->now();
        out_msg.header.frame_id = "";

        if (use_joy_mode_.load()) {
            out_msg.twist.linear.x = linear_vel_.load();
            out_msg.twist.angular.z = angular_vel_.load();
        } else {
            out_msg.twist.linear.x = v_x_topic_;
            out_msg.twist.angular.z = v_w_topic_;
        }

        // 统一死区过滤 (0.07 稍微有点大，可以根据手柄质量调整)
        if (std::abs(out_msg.twist.linear.x) < 0.05) out_msg.twist.linear.x = 0.0;
        if (std::abs(out_msg.twist.angular.z) < 0.05) out_msg.twist.angular.z = 0.0;

        publisher_->publish(out_msg);
    }

    void read_joy_loop()
    {
        struct js_event event;
        while (rclcpp::ok() && joy_fd_ != -1) {
            if (read(joy_fd_, &event, sizeof(event)) > 0) {
                if (event.type & JS_EVENT_INIT) continue;

                // 按键逻辑
                if (event.type == JS_EVENT_BUTTON && event.value == 1) {
                    if (event.number == 1) { // 切换模式键
                        use_joy_mode_ = !use_joy_mode_;
                        RCLCPP_INFO(this->get_logger(), "模式切换 -> %s", 
                                   use_joy_mode_ ? "【手柄控制】" : "【话题控制】");
                    }
                    if (event.number == 0) { // 急停
                        linear_vel_.store(0.0f);
                        angular_vel_.store(0.0f);
                    }
                }

                // 摇杆逻辑
                if (event.type == JS_EVENT_AXIS) {
                    float val = -(float)event.value / 32767.0f;
                    if (event.number == 4) linear_vel_.store(val * 0.5f);
                    else if (event.number == 0) angular_vel_.store(val * 1.0f);
                }
            }
            std::this_thread::sleep_for(1ms);
        }
    }

    // 基础变量
    int joy_fd_ = -1;
    std::thread read_thread_;
    std::atomic<bool> use_joy_mode_{true}; 
    
    // 速度存储
    std::atomic<float> linear_vel_{0.0f};
    std::atomic<float> angular_vel_{0.0f};
    float v_x_topic_ = 0.0f;
    float v_w_topic_ = 0.0f;

    // ROS 对象
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr subscription_stamped_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotionCtrl>());
    rclcpp::shutdown();
    return 0;
}