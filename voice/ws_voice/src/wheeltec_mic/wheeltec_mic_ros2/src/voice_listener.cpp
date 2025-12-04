/****************************************************************/
/* 语音识别结果监听节点                                            */
/* 功能：订阅voice_words话题，打印和处理识别到的语音内容            */
/****************************************************************/
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/int8.hpp>
#include <iostream>

class VoiceListener : public rclcpp::Node
{
public:
    VoiceListener() : Node("voice_listener")
    {
        // 订阅语音识别结果话题
        voice_words_sub_ = this->create_subscription<std_msgs::msg::String>(
            "voice_words", 10,
            std::bind(&VoiceListener::voiceWordsCallback, this, std::placeholders::_1));
        
        // 订阅唤醒角度话题
        awake_angle_sub_ = this->create_subscription<std_msgs::msg::UInt32>(
            "awake_angle", 10,
            std::bind(&VoiceListener::awakeAngleCallback, this, std::placeholders::_1));
        
        // 订阅唤醒标志话题
        awake_flag_sub_ = this->create_subscription<std_msgs::msg::Int8>(
            "awake_flag", 10,
            std::bind(&VoiceListener::awakeFlagCallback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "语音监听节点已启动，等待语音识别结果...");
        RCLCPP_INFO(this->get_logger(), "==========================================");
    }

private:
    void voiceWordsCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        // 打印时间戳
        auto now = this->get_clock()->now();
        RCLCPP_INFO(this->get_logger(), 
                   "\n==========================================");
        RCLCPP_INFO(this->get_logger(), 
                   "[时间: %f]", now.seconds());
        RCLCPP_INFO(this->get_logger(), 
                   ">>> 识别到的语音内容: %s", msg->data.c_str());
        RCLCPP_INFO(this->get_logger(), 
                   "==========================================\n");
        
        // 在这里你可以添加自己的处理逻辑
        processVoiceCommand(msg->data);
    }
    
    void awakeAngleCallback(const std_msgs::msg::UInt32::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
                   ">>> 声音来源角度: %u°", msg->data);
    }
    
    void awakeFlagCallback(const std_msgs::msg::Int8::SharedPtr msg)
    {
        if (msg->data == 1) {
            RCLCPP_INFO(this->get_logger(), 
                       ">>> 麦克风已唤醒！正在监听...");
        }
    }
    
    void processVoiceCommand(const std::string& voice_text)
    {
        // 在这里添加你的语音命令处理逻辑
        // 例如：
        if (voice_text.find("前进") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "    → 检测到前进命令");
        }
        else if (voice_text.find("后退") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "    → 检测到后退命令");
        }
        else if (voice_text.find("停") != std::string::npos) {
            RCLCPP_INFO(this->get_logger(), "    → 检测到停止命令");
        }
        // 可以继续添加更多命令处理
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr voice_words_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr awake_angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr awake_flag_sub_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VoiceListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
