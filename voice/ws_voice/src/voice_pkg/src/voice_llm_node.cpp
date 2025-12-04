#include <iostream>
#include <string>
#include <atomic>
#include <fstream>
#include <vector>
#include <unistd.h>
#include <regex>
#include <stdio.h>
#include <stdlib.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "sparkchain.h"
#include "sc_asr.h"
#include "sc_llm.h"

#define GREEN "\033[32m"
#define YELLOW "\033[33m"
#define RED "\033[31m"
#define RESET "\033[0m"

using namespace SparkChain;
using namespace std;
using namespace rclcpp;

// 全局变量
static atomic_bool asr_finish(false);
static atomic_bool llm_finish(false);
static string asr_result = "";
static string llm_result = "";

/*************************************SDK初始化参数**********************************************************/
char * APPID     = "ad184cbf";                        // 用户的APPID
char * APIKEY    = "301e469bbd67f75138209b58436a9e1a"; // 用户的APIKey
char * APISECRET = "ZDFkY2JkNjIyZDU3NzZjNjhmYmMzZTA5"; // 用户的APISecret
char * WORKDIR   = "./";                              // SDK工作目录，要求有读写权限
int logLevel     = 100;                               // 日志等级
/*************************************SDK初始化参数**********************************************************/

// ASR识别监听回调
class ASRCallbacksImpl : public ASRCallbacks {
    void onResult(ASRResult * result, void * usrTag) override {
        string asrText = result->bestMatchText(); // 解析识别结果
        int status = result->status();            // 解析结果返回状态
        
        printf(GREEN "识别结果: %s status = %d\n" RESET, asrText.c_str(), status);
        
        if (status == 2) {
            asr_result = asrText;
            asr_finish = true;
        }
    }
    
    void onError(ASRError * error, void * usrTag) override {
        int errCode = error->code();     // 错误码
        string errMsg = error->errMsg(); // 错误信息
        
        printf(RED "语音识别出错,错误码: %d,错误信息:%s\n" RESET, errCode, errMsg.c_str());
        asr_finish = true; 
    }
};

// LLM大模型交互回调
class LLMCallbacksImpl : public LLMCallbacks {
    void onLLMResult(LLMResult *result, void *usrContext) override {
        if(result->getContentType() == LLMResult::TEXT){
            const char* content = result->getContent(); // 获取返回结果
            int status = result->getStatus();           // 返回结果状态
            
            printf(YELLOW "%s" RESET, content);
            llm_result += string(content);
            
            if (status == 2) {
                printf(YELLOW "\n" RESET);
                llm_finish = true;
            } 
        }   
    }

    void onLLMError(LLMError *error, void *usrContext) override {
        int errCode = error->getErrCode();     // 获取错误码
        const char* errMsg = error->getErrMsg(); // 获取错误信息
        
        printf(RED "大模型请求出错,错误码: %d,错误信息:%s\n" RESET, errCode, errMsg);
        llm_finish = true;  
    }
    
    void onLLMEvent(LLMEvent *event, void *usrContext) override {
        // 事件处理，可根据需要实现
    }
};

class VoiceLLMNode : public rclcpp::Node {
public:
    VoiceLLMNode() : Node("voice_llm_node") {
        // 创建发布者
        asr_result_publisher_ = this->create_publisher<std_msgs::msg::String>("asr_result", 10);
        llm_result_publisher_ = this->create_publisher<std_msgs::msg::String>("llm_result", 10);
        
        // 创建定时器，定期处理语音和LLM交互
        timer_ = this->create_wall_timer(
            std::chrono::seconds(10),
            std::bind(&VoiceLLMNode::process_voice_llm, this));
            
        RCLCPP_INFO(this->get_logger(), "Voice LLM Node 已启动");
    }

private:
    void process_voice_llm() {
        RCLCPP_INFO(this->get_logger(), "开始语音处理流程");
        
        // 1. 语音识别
        string recognized_text = run_speech_recognition();
        
        if (!recognized_text.empty()) {
            // 发布ASR结果
            auto asr_msg = std_msgs::msg::String();
            asr_msg.data = recognized_text;
            asr_result_publisher_->publish(asr_msg);
            RCLCPP_INFO(this->get_logger(), "语音识别结果: %s", recognized_text.c_str());
            
            // 2. 大模型交互
            string llm_response = run_llm_interaction(recognized_text);
            
            if (!llm_response.empty()) {
                // 发布LLM结果
                auto llm_msg = std_msgs::msg::String();
                llm_msg.data = llm_response;
                llm_result_publisher_->publish(llm_msg);
                RCLCPP_INFO(this->get_logger(), "大模型回复: %s", llm_response.c_str());
            }
        }
    }
    
    string run_speech_recognition() {
        asr_finish = false;
        asr_result = "";
        
        // 构建ASR实例 - 中文识别
        ASR asr("zh_cn", "iat", "mandarin");
        asr.vinfo(true); // 启用句子级别帧对齐
        
        ASRCallbacksImpl *asr_callbacks = new ASRCallbacksImpl();
        asr.registerCallbacks(asr_callbacks); // 注册监听回调
        
        // 设置音频属性
        SparkChain::AudioAttributes attr;
        attr.setSampleRate(16000);  // 输入音频采样率: 16K
        attr.setEncoding("raw");    // 输入音频的编码格式: raw (pcm音频)
        attr.setChannels(1);        // 输入音频的声道: 单声道
        
        // 开始识别
        asr.start(attr);
        
        // 读取音频文件并发送数据
        const char* audio_path = "/home/tang/voice/SparkChain_Linux_SDK_2.0.0_rc1/SparkChain_Linux_SDK_2.0.0_rc1/test_file/cn.pcm";
        FILE *file = fopen(audio_path, "rb");
        if (!file) {
            RCLCPP_ERROR(this->get_logger(), "无法打开音频文件: %s", audio_path);
            delete asr_callbacks;
            return "";
        }
        
        fseek(file, 0, SEEK_END);
        size_t file_len = ftell(file);
        fseek(file, 0, SEEK_SET);
        
        char *data = new char[file_len];
        fread(data, 1, file_len, file);
        
        const int per_frame_size = 1280 * 8;
        size_t read_len = 0;
        
        while (read_len < file_len) {
            size_t cur_size = per_frame_size;
            if (read_len + per_frame_size > file_len) {
                cur_size = file_len - read_len;
            }
            
            asr.write(data + read_len, cur_size);
            read_len += cur_size;
            usleep(40 * 1000); // 模拟实时音频流
        }
        
        asr.stop();
        fclose(file);
        delete[] data;
        
        // 等待识别完成
        int times = 0;
        while (!asr_finish && times < 10) {
            sleep(1);
            times++;
        }
        
        delete asr_callbacks;
        return asr_result;
    }
    
    string run_llm_interaction(const string& input_text) {
        llm_finish = false;
        llm_result = "";
        
        // 配置大模型参数
        LLMConfig *llm_config = LLMConfig::builder();
        llm_config->domain("4.0Ultra"); // 使用4.0Ultra版本
        
        // 设置历史上下文（保留最近5轮对话）
        Memory* window_memory = Memory::WindowMemory(5);
        LLM *llm = LLMFactory::textGeneration(llm_config, window_memory);
        
        if (llm == nullptr) {
            RCLCPP_ERROR(this->get_logger(), "LLM实例创建失败");
            return "";
        }
        
        LLMCallbacksImpl *llm_callbacks = new LLMCallbacksImpl();
        llm->registerLLMCallbacks(llm_callbacks); // 注册结果监听回调
        
        // 异步请求
        int ret = llm->arun(input_text.c_str());
        if (ret != 0) {
            RCLCPP_ERROR(this->get_logger(), "异步请求失败,错误码: %d", ret);
            LLM::destroy(llm);
            delete llm_callbacks;
            return "";
        }
        
        // 等待结果返回
        int times = 0;
        while (!llm_finish && times < 20) {
            sleep(1);
            times++;
        }
        
        // 清理资源
        LLM::destroy(llm);
        delete llm_callbacks;
        
        return llm_result;
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr asr_result_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr llm_result_publisher_;
};

/***
 * SDK初始化
 ***/
int initSDK() {
    SparkChainConfig *config = SparkChainConfig::builder();
    config->appID(APPID)        // 你的appid
        ->apiKey(APIKEY)        // 你的apikey
        ->apiSecret(APISECRET)  // 你的apisecret
        ->workDir(WORKDIR)
        ->logLevel(logLevel); 
    int ret = SparkChain::init(config);
    return ret;
}

void uninitSDK() {
    // SDK逆初始化
    SparkChain::unInit();
}

int main(int argc, char ** argv) {
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 初始化科大讯飞SDK
    int ret = initSDK();
    if (ret != 0) {
        printf(RED "SDK初始化失败!错误码:%d\n" RESET, ret);
        return -1;
    }
    
    printf(GREEN "科大讯飞SDK初始化成功\n" RESET);
    
    // 创建并运行ROS节点
    auto node = std::make_shared<VoiceLLMNode>();
    rclcpp::spin(node);
    
    // 清理资源
    uninitSDK();
    rclcpp::shutdown();
    
    return 0;
}
