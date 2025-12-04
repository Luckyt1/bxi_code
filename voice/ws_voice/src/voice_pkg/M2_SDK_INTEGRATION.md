# M2_SDK 语音识别集成指南

## 概述

本文档说明如何使用 M2_SDK 实现语音读取并返回标志位的功能。提供了两个实现版本：
1. **ROS2集成版本** (`m2_voice_node.cpp`) - 完整的ROS2节点实现
2. **简化C版本** (`m2_voice_example.c`) - 核心功能演示

---

## 核心功能说明

### 1. 状态标志位结构

```c
typedef struct {
    bool is_awake;              // 唤醒标志
    bool voice_detected;        // 语音检测标志
    bool recognition_complete;  // 识别完成标志
    int voice_angle;            // 声源方向角度
    int confidence;             // 识别置信度
    char result_text[128];      // 识别结果文本
    int result_id;              // 识别结果ID
} VoiceStatus;
```

### 2. 关键API函数

#### 初始化SDK
```c
Recognise_Result initial_asr_paramers(
    char *jet_path,      // ASR资源路径
    char *grammer_path,  // 语法构建路径
    char *bnf_path,      // 语法文件路径
    char *lex_na         // 词典名称
);
```

#### 创建ASR引擎
```c
int create_asr_engine(UserData *udata);
```

#### 开始录音和识别
```c
void get_the_record_sound(const char *file);
```

#### 删除引擎
```c
void delete_asr_engine();
```

#### 检查唤醒状态
```c
extern int if_awake;    // 1=已唤醒, 0=未唤醒
extern int angle_int;   // 声源角度
```

#### 获取识别结果
```c
extern char* whole_result;  // 识别结果XML字符串
extern int record_finish;   // 录音完成标志
```

---

## 使用流程

### 基本流程

```
1. 初始化SDK
   ↓
2. 等待唤醒 (if_awake == 1)
   ↓
3. 获取声源角度 (angle_int)
   ↓
4. 创建ASR引擎
   ↓
5. 开始录音识别
   ↓
6. 等待识别完成 (record_finish == 1)
   ↓
7. 解析结果 (whole_result)
   ↓
8. 清理引擎
   ↓
9. 重置状态，返回步骤2
```

### 示例代码（简化版）

```c
// 1. 初始化
Recognise_Result init_result = initial_asr_paramers(
    ASR_RES_PATH, GRM_BUILD_PATH, GRM_FILE, LEX_NAME
);

while (1) {
    // 2. 等待唤醒
    if (!if_awake) {
        sleep(1);
        continue;
    }
    
    // 3. 获取角度
    int angle = angle_int;
    printf("声源角度: %d\n", angle);
    
    // 4. 创建引擎
    UserData asr_data;
    create_asr_engine(&asr_data);
    
    // 5. 录音识别
    record_finish = 0;
    get_the_record_sound(DENOISE_SOUND_PATH);
    
    // 6. 等待完成
    while (!record_finish) {
        usleep(100000);
    }
    
    // 7. 处理结果
    if (whole_result != NULL) {
        printf("识别结果: %s\n", whole_result);
        // 解析XML结果...
    }
    
    // 8. 清理
    delete_asr_engine();
    if_awake = 0;
}
```

---

## 结果解析

识别结果是XML格式字符串，需要解析以下字段：

### XML结构示例
```xml
<confidence>75</confidence>
<rawtext>向前走</rawtext>
<id=123>
```

### 解析代码
```c
// 解析置信度
char* p_conf = strstr(result, "<confidence>");
char* p_conf_end = strstr(result, "</confidence>");
// 提取数字...

// 解析文本
char* p_text = strstr(result, "<rawtext>");
char* p_text_end = strstr(result, "</rawtext>");
// 提取文本...

// 解析ID
char* p_id = strstr(result, "id=");
// 提取ID...
```

---

## ROS2集成版本使用

### 编译
```bash
cd /home/tang/voice/ws_voice
colcon build --packages-select voice_pkg
source install/setup.bash
```

### 运行
```bash
ros2 run voice_pkg m2_voice_node
```

### 话题订阅

| 话题名称 | 消息类型 | 说明 |
|---------|---------|------|
| `/voice_result` | `std_msgs/String` | 识别结果文本 |
| `/voice_confidence` | `std_msgs/Int32` | 置信度 |
| `/voice_angle` | `std_msgs/Int32` | 声源角度 |
| `/voice_detected` | `std_msgs/Bool` | 语音检测标志 |
| `/wake_up_status` | `std_msgs/Bool` | 唤醒状态 |
| `/recognition_complete` | `std_msgs/Bool` | 识别完成标志 |

### 订阅示例
```bash
# 查看识别结果
ros2 topic echo /voice_result

# 查看唤醒状态
ros2 topic echo /wake_up_status

# 查看所有话题
ros2 topic list
```

---

## 简化C版本使用

### 编译
```bash
cd /home/tang/voice/ws_voice/M2_SDK/offline_mic_vad/samples
gcc -o m2_example \
    ../../src/m2_voice_example.c \
    -I../../include \
    -L../../lib \
    -lmsc -lasound -lpthread
```

### 运行
```bash
./m2_example
```

### 输出示例
```
>>>>>> M2_SDK 语音识别示例启动
>>>>>> 初始化M2 SDK...
>>>>>> M2 SDK初始化成功

>>>>>> 等待唤醒...
>>>>>> 唤醒成功! 声源角度: 45度
>>>>>> 开始语音识别...
>>>>>> 识别成功!
       文本: [向前走]
       置信度: 85
       ID: 1

========== 语音状态 ==========
唤醒状态: 已唤醒
语音检测: 检测到
识别完成: 完成
声源角度: 45度
置信度: 85
识别结果: 向前走
结果ID: 1
==============================
```

---

## 标志位使用场景

### 场景1：等待唤醒
```c
VoiceStatus status;
get_voice_status(&status);

if (status.is_awake) {
    // 开始处理语音
}
```

### 场景2：检测语音
```c
if (status.voice_detected && status.recognition_complete) {
    printf("检测到语音: %s\n", status.result_text);
}
```

### 场景3：根据ID执行命令
```c
switch (status.result_id) {
    case 1:
        move_forward();
        break;
    case 2:
        turn_left();
        break;
    case 3:
        turn_right();
        break;
}
```

### 场景4：置信度判断
```c
if (status.confidence > 80) {
    // 高置信度，直接执行
    execute_command(status.result_id);
} else if (status.confidence > 50) {
    // 中等置信度，请求确认
    ask_confirmation(status.result_text);
} else {
    // 低置信度，忽略
    printf("识别不清楚，请重新说\n");
}
```

---

## 配置参数

在 `user_interface.h` 中配置：

```c
// 置信度阈值（建议40-80）
int confidence = 40;

// 最大识别时长（秒）
int max_asr_time = 10;

// 一次唤醒可对话次数
int awake_count = 5;

// APPID（需替换为自己的）
char *APPID = "your_appid";

// 资源路径（需修改为实际路径）
char *ASR_RES_PATH = "/path/to/common.jet";
char *GRM_BUILD_PATH = "/path/to/GrmBuilld";
char *GRM_FILE = "/path/to/call.bnf";
```

---

## 常见问题

### Q1: 如何修改唤醒词？
A: 唤醒词由硬件麦克风阵列固定，通过串口 `/dev/wheeltec_mic` 接收唤醒信号。

### Q2: 如何自定义识别命令词？
A: 编辑 `call.bnf` 语法文件，添加自定义命令词和ID。

### Q3: 置信度设置多少合适？
A: 
- 安静环境：40-50
- 一般环境：50-60
- 嘈杂环境：60-80

### Q4: 如何获取实时音频流？
A: 参考 `record.h` 中的 `business_data()` 回调函数。

### Q5: 多线程安全吗？
A: 示例代码使用 `pthread_mutex` 保护状态变量，确保线程安全。

---

## 完整项目结构

```
ws_voice/
├── voice_pkg/
│   └── src/
│       ├── voice_llm_node.cpp       # 原SparkChain版本
│       ├── m2_voice_node.cpp        # M2 ROS2集成版本
│       └── m2_voice_example.c       # M2 简化示例
└── M2_SDK/
    └── offline_mic_vad/
        ├── include/
        │   ├── user_interface.h
        │   ├── record.h
        │   └── asr_offline_record_sample.h
        ├── lib/
        └── samples/
```

---

## 参考资料

- M2_SDK官方文档
- `samples/offline_command_sample/main.c` - 官方示例
- `readme.md` - SDK说明文档

---

## 技术支持

如有问题，请检查：
1. APPID/APIKey是否正确
2. 资源路径是否存在
3. 麦克风设备是否连接
4. 权限是否足够（串口/音频设备）
