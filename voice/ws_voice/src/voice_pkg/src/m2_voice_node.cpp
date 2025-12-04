#include <iostream>
#include <string>
#include <atomic>
#include <thread>
#include <memory>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"

extern "C" {
    // user_interface.h 已经包含了 asr_offline_record_sample.h
    // 所以只需要包含 user_interface.h 即可
    #include "user_interface.h"
    #include "record.h"
}

#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define RED "\033[31m"
#define RESET "\033[0m"

using namespace std;
using namespace rclcpp;

// 全局标志位
static atomic_bool voice_detected(false);      // 语音检测标志
static atomic_bool recognition_complete(false); // 识别完成标志
static atomic_bool wake_up_detected(false);     // 唤醒检测标志
static string current_result = "";              // 当前识别结果
static int current_confidence = 0;              // 当前置信度
static int voice_angle = 0;                     // 声源角度

// 外部变量声明（来自M2_SDK）
extern int angle_int;
extern int if_awake;
extern int init_rec;
extern int init_success;
extern int record_finish;
extern char* whole_result;

/**
 * M2语音节点类
 * 实现语音唤醒、识别，并通过ROS2话题发布结果和标志位
 */
class M2VoiceNode : public rclcpp::Node {
public:
    M2VoiceNode() : Node("m2_voice_node") {
        // 创建发布者
        result_publisher_ = this->create_publisher<std_msgs::msg::String>("voice_result", 10);
        confidence_publisher_ = this->create_publisher<std_msgs::msg::Int32>("voice_confidence", 10);
        angle_publisher_ = this->create_publisher<std_msgs::msg::Int32>("voice_angle", 10);
        
        // 状态标志位发布者
        voice_detected_publisher_ = this->create_publisher<std_msgs::msg::Bool>("voice_detected", 10);
        wake_up_publisher_ = this->create_publisher<std_msgs::msg::Bool>("wake_up_status", 10);
        recognition_complete_publisher_ = this->create_publisher<std_msgs::msg::Bool>("recognition_complete", 10);
        
        // 初始化M2 SDK
        if (!init_m2_sdk()) {
            RCLCPP_ERROR(this->get_logger(), "M2 SDK初始化失败!");
            return;
        }
        
        RCLCPP_INFO(this->get_logger(), "M2 Voice Node 已启动");
        
        // 启动语音监听线程
        voice_thread_ = std::thread(&M2VoiceNode::voice_listening_loop, this);
        
        // 创建定时器，定期发布状态
        status_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&M2VoiceNode::publish_status, this));
    }
    
    ~M2VoiceNode() {
        running_ = false;
        if (voice_thread_.joinable()) {
            voice_thread_.join();
        }
        cleanup_m2_sdk();
    }

private:
    /**
     * 初始化M2 SDK
     */
    bool init_m2_sdk() {
        RCLCPP_INFO(this->get_logger(), "初始化M2 SDK...");
        
        // 初始化ASR参数
        Recognise_Result result = initial_asr_paramers(
            ASR_RES_PATH, 
            GRM_BUILD_PATH, 
            GRM_FILE, 
            LEX_NAME
        );
        
        if (!result.whether_recognised && strlen(result.fail_reason) > 0) {
            RCLCPP_ERROR(this->get_logger(), "ASR初始化失败: %s", result.fail_reason);
            return false;
        }
        
        RCLCPP_INFO(this->get_logger(), GREEN "M2 SDK初始化成功" RESET);
        return true;
    }
    
    /**
     * 清理M2 SDK资源
     */
    void cleanup_m2_sdk() {
        delete_asr_engine();
        RCLCPP_INFO(this->get_logger(), "M2 SDK资源已清理");
    }
    
    /**
     * 语音监听主循环
     */
    void voice_listening_loop() {
        while (running_ && rclcpp::ok()) {
            // 等待唤醒
            if (!if_awake) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                    "等待唤醒...");
                wake_up_detected = false;
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            
            // 检测到唤醒
            if (!wake_up_detected) {
                wake_up_detected = true;
                voice_angle = angle_int;
                RCLCPP_INFO(this->get_logger(), GREEN "唤醒成功! 声源角度: %d度" RESET, voice_angle);
            }
            
            // 进行语音识别
            recognition_complete = false;
            voice_detected = false;
            
            if (process_voice_recognition()) {
                voice_detected = true;
                recognition_complete = true;
                
                // 发布识别结果
                publish_recognition_result();
            } else {
                recognition_complete = true;
                RCLCPP_WARN(this->get_logger(), "未检测到有效语音");
            }
            
            // 重置唤醒状态
            if_awake = 0;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    /**
     * 处理语音识别
     * @return true 识别成功, false 识别失败
     */
    bool process_voice_recognition() {
        RCLCPP_INFO(this->get_logger(), "开始语音识别...");
        
        // 创建ASR引擎
        int ret = create_asr_engine(&asr_data_);
        if (ret != MSP_SUCCESS) {
            RCLCPP_ERROR(this->get_logger(), "创建ASR引擎失败: %d", ret);
            return false;
        }
        
        // 开始录音并识别
        record_finish = 0;
        const char* audio_file = DENOISE_SOUND_PATH;
        
        // 获取录音
        get_the_record_sound(audio_file);
        
        // 等待识别完成
        int timeout = 0;
        while (!record_finish && timeout < 100) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            timeout++;
        }
        
        // 解析识别结果
        bool success = false;
        if (whole_result != nullptr && strlen(whole_result) > 0) {
            Effective_Result effective = parse_result(whole_result);
            
            if (effective.effective_confidence >= confidence) {
                current_result = string(effective.effective_word);
                current_confidence = effective.effective_confidence;
                success = true;
                
                RCLCPP_INFO(this->get_logger(), 
                    YELLOW "识别结果: [%s], 置信度: %d" RESET,
                    current_result.c_str(), current_confidence);
            } else {
                RCLCPP_WARN(this->get_logger(), 
                    "置信度过低: %d (阈值: %d)", 
                    effective.effective_confidence, confidence);
            }
        }
        
        // 清理引擎
        delete_asr_engine();
        whole_result = (char*)" ";
        
        return success;
    }
    
    /**
     * 解析识别结果
     */
    Effective_Result parse_result(const char* result_str) {
        Effective_Result result;
        result.effective_confidence = 0;
        strcpy(result.effective_word, "");
        result.effective_id = 0;
        
        if (strlen(result_str) < 250) {
            return result;
        }
        
        // 解析置信度
        char* p_conf_start = strstr((char*)result_str, "<confidence>");
        char* p_conf_end = strstr((char*)result_str, "</confidence>");
        
        if (p_conf_start && p_conf_end) {
            char confidence_str[8] = {0};
            int conf_len = p_conf_end - p_conf_start - strlen("<confidence>");
            strncpy(confidence_str, p_conf_start + strlen("<confidence>"), conf_len);
            result.effective_confidence = atoi(confidence_str);
        }
        
        // 解析文本
        if (result.effective_confidence >= confidence) {
            char* p_text_start = strstr((char*)result_str, "<rawtext>");
            char* p_text_end = strstr((char*)result_str, "</rawtext>");
            
            if (p_text_start && p_text_end) {
                int text_len = p_text_end - p_text_start - strlen("<rawtext>");
                strncpy(result.effective_word, 
                    p_text_start + strlen("<rawtext>"), 
                    min(text_len, 31));
                result.effective_word[min(text_len, 31)] = '\0';
            }
            
            // 解析ID
            char* p_id = strstr(p_conf_start, "id=");
            if (p_id) {
                char* p_id_end = strstr(p_id, ">");
                char id_str[8] = {0};
                strncpy(id_str, p_id + 4, p_id_end - p_id - 5);
                result.effective_id = atoi(id_str);
            }
        }
        
        return result;
    }
    
    /**
     * 发布识别结果
     */
    void publish_recognition_result() {
        // 发布识别文本
        auto result_msg = std_msgs::msg::String();
        result_msg.data = current_result;
        result_publisher_->publish(result_msg);
        
        // 发布置信度
        auto conf_msg = std_msgs::msg::Int32();
        conf_msg.data = current_confidence;
        confidence_publisher_->publish(conf_msg);
        
        // 发布声源角度
        auto angle_msg = std_msgs::msg::Int32();
        angle_msg.data = voice_angle;
        angle_publisher_->publish(angle_msg);
        
        RCLCPP_INFO(this->get_logger(), 
            GREEN "发布识别结果: %s (置信度:%d, 角度:%d度)" RESET,
            current_result.c_str(), current_confidence, voice_angle);
    }
    
    /**
     * 定期发布状态标志位
     */
    void publish_status() {
        auto detected_msg = std_msgs::msg::Bool();
        detected_msg.data = voice_detected.load();
        voice_detected_publisher_->publish(detected_msg);
        
        auto wakeup_msg = std_msgs::msg::Bool();
        wakeup_msg.data = wake_up_detected.load();
        wake_up_publisher_->publish(wakeup_msg);
        
        auto complete_msg = std_msgs::msg::Bool();
        complete_msg.data = recognition_complete.load();
        recognition_complete_publisher_->publish(complete_msg);
    }
    
    // ROS发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr confidence_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr angle_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr voice_detected_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wake_up_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr recognition_complete_publisher_;
    
    // 定时器和线程
    rclcpp::TimerBase::SharedPtr status_timer_;
    std::thread voice_thread_;
    atomic_bool running_{true};
    
    // M2 SDK数据
    UserData asr_data_;
};

int main(int argc, char** argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建并运行节点
    auto node = std::make_shared<M2VoiceNode>();
    rclcpp::spin(node);
    
    // 清理
    rclcpp::shutdown();
    return 0;
}
