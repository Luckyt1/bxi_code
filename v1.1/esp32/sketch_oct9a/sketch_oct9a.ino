#include <SCServo.h>
#include <Arduino.h>
#include <HardwareSerial.h>
#include <WiFi.h>
#include <WiFiUdp.h>

HLSCL hlscl;
unsigned long previousMillis = 0;
unsigned long serialPrintMillis = 0;
unsigned long wifiSendMillis = 0;
const long readInterval = 100;     // 数据读取间隔
const long printInterval = 1000;   // 串口打印间隔
const long wifiSendInterval = 500; // WiFi发送间隔

#define MIN_SERVO_ID 0
#define MAX_SERVO_ID 7
#define SERVO_COUNT (MAX_SERVO_ID - MIN_SERVO_ID + 1)
#define TARGET_ANGLE 180
#define MOVE_SPEED 500
#define PACKET_HEADER 0xAA
#define PACKET_FOOTER 0x55
#define PACKET_SIZE 34

const char *ssid = "ikuai-hupa2.4G";
const char *password = "Hupa@2018";
const char *server_ip = "192.168.88.92";
const int server_port = 8080;
WiFiUDP udp;
bool wifi_connected = false;

struct Sensor_Data
{
    float angle_deg = 0;        // 当前角度（-180到+180）
    float multi_turn_angle = 0; // 多圈累计角度
    float current_mA = 0;
    float current_pos = 0;
    float current_speed = 0;
    int zero_offset = 0;          // 零点偏移值
    bool zero_calibrated = false; // 是否已校正零点
    int last_raw_pos = 0;         // 上次原始位置值
    bool first_read = true;       // 是否第一次读取
};
Sensor_Data sensor[9];

// 初始化零点校正函数
void initializeZeroCalibration()
{
    Serial.println("========== 初始化零点校正 ==========");
    Serial.println("正在记录当前位置作为零点参考...");

    // 等待舵机稳定
    delay(1000);

    bool allSuccess = true;

    for (int id = MIN_SERVO_ID; id <= MAX_SERVO_ID; id++)
    {
        // 多次读取确保数据稳定
        int posSum = 0;
        int validReads = 0;

        for (int i = 0; i < 5; i++)
        {
            int currentPos = hlscl.ReadPos(id);
            if (!hlscl.getLastError())
            {
                posSum += currentPos;
                validReads++;
            }
            delay(20);
        }

        if (validReads >= 3) // 至少3次成功读取
        {
            sensor[id].zero_offset = posSum / validReads; // 取平均值
            sensor[id].zero_calibrated = true;
            // 重置多圈计数
            sensor[id].multi_turn_angle = 0;
            sensor[id].first_read = true;
            Serial.printf("舵机%d初始零点位置: %d (平均值，基于%d次读取)\n",
                          id, sensor[id].zero_offset, validReads);
        }
        else
        {
            Serial.printf("舵机%d初始化失败 - 读取错误\n", id);
            sensor[id].zero_calibrated = false;
            allSuccess = false;
        }
    }

    if (allSuccess)
    {
        Serial.println("✓ 所有舵机初始化零点校正完成！");
    }
    else
    {
        Serial.println("⚠ 部分舵机初始化失败，请检查连接");
    }

    Serial.println("=====================================\n");
}

// 手动零点校正函数
void calibrateZeroPoint()
{
    Serial.println("开始零点校正...");
    Serial.println("请将所有舵机手动调整到零点位置，然后按回车键确认");

    // 等待用户输入
    while (!Serial.available())
    {
        delay(100);
    }

    // 清空串口缓冲区
    while (Serial.available())
    {
        Serial.read();
    }

    Serial.println("正在记录零点位置...");

    for (int id = MIN_SERVO_ID; id <= MAX_SERVO_ID; id++)
    {
        int currentPos = hlscl.ReadPos(id);
        if (!hlscl.getLastError())
        {
            sensor[id].zero_offset = currentPos;
            sensor[id].zero_calibrated = true;
            // 重置多圈计数
            sensor[id].multi_turn_angle = 0;
            sensor[id].first_read = true;
            Serial.printf("舵机%d零点位置: %d\n", id, currentPos);
        }
        else
        {
            Serial.printf("舵机%d读取失败\n", id);
            sensor[id].zero_calibrated = false;
        }
        delay(50);
    }

    Serial.println("零点校正完成！");
}

// 简单累加多圈角度计算函数
void updateMultiTurnAngleSimple(int id, int current_raw_pos)
{
    if (sensor[id].first_read)
    {
        // 第一次读取，直接设置
        sensor[id].last_raw_pos = current_raw_pos;
        sensor[id].first_read = false;
        return;
    }

    // 计算原始位置差值
    int pos_diff = current_raw_pos - sensor[id].last_raw_pos;

    // 处理4095边界跨越
    if (pos_diff > 2000)
    {
        pos_diff -= 4095; // 逆时针跨越
    }
    else if (pos_diff < -2000)
    {
        pos_diff += 4095; // 顺时针跨越
    }

    // 直接累加角度变化
    float angle_change = pos_diff / 4095.0 * 360;
    sensor[id].multi_turn_angle += angle_change;

    // 更新上次位置
    sensor[id].last_raw_pos = current_raw_pos;
}

// 数据读取函数
void readSensorData()
{
    
    for (int id = MIN_SERVO_ID; id <= MAX_SERVO_ID; id++)
    {
        sensor[id].current_pos = hlscl.ReadPos(id);
        sensor[id].current_speed = hlscl.ReadSpeed(id);
        sensor[id].current_mA = hlscl.ReadCurrent(id);

        // 转换为角度值（带零点校正）
        if (!hlscl.getLastError())
        {
            int calibrated_pos = sensor[id].current_pos;
            int raw_pos_for_multi = sensor[id].current_pos;

            // 如果已校正零点，则应用偏移
            if (sensor[id].zero_calibrated)
            {
                calibrated_pos = sensor[id].current_pos - sensor[id].zero_offset;
                raw_pos_for_multi = sensor[id].current_pos - sensor[id].zero_offset;

                // 处理角度环绕（-180°到+180°）
                while (calibrated_pos > 2047)
                    calibrated_pos -= 4095;
                while (calibrated_pos < -2048)
                    calibrated_pos += 4095;
            }

            sensor[id].angle_deg = calibrated_pos / 4095.0 * 360;
            
            // 限制角度范围到-180°到+180°
            if (sensor[id].angle_deg > 179)
                sensor[id].angle_deg -= 360;
            if (sensor[id].angle_deg < -179)
                sensor[id].angle_deg += 360;

            // 更新多圈角度（使用原始位置数据）
            updateMultiTurnAngleSimple(id, raw_pos_for_multi);
        }
    }
}

// 串口打印函数
void printSensorData()
{
    for (int id = MIN_SERVO_ID; id <= MAX_SERVO_ID; id++)
    {
        Serial.print(id);
        Serial.print("号舵机: ");
        Serial.print(sensor[id].angle_deg, 1);
        Serial.print("° (多圈: ");
        Serial.print(sensor[id].multi_turn_angle, 1);
        Serial.print("°, 电流: ");
        Serial.print(sensor[id].current_mA, 1);
        Serial.print("mA)");

        if (!sensor[id].zero_calibrated)
        {
            Serial.print(" [未校正]");
        }

        if (id < MAX_SERVO_ID)
        {
            Serial.print(" | ");
        }
    }
    Serial.println();
}

// WiFi发送函数
void sendWifiData()
{
    if (wifi_connected)
    {
        String jsonData = "{\"sensors\":[";

        for (int i = 0; i < 8; i++)
        {
            if (i > 0)
                jsonData += ",";
            jsonData += "{\"id\":" + String(i) +
                        ",\"angle\":" + String(sensor[i].angle_deg) +
                        ",\"multi_turn_angle\":" + String(sensor[i].multi_turn_angle) +
                        ",\"calibrated\":" + String(sensor[i].zero_calibrated ? "true" : "false") + "}";
        }

        jsonData += "],\"timestamp\":" + String(millis()) + "}";

        // 发送数据
        udp.beginPacket(server_ip, server_port);
        udp.print(jsonData);
        int result = udp.endPacket();

        if (!result)
        {
            Serial.println("UDP发送失败");
        }
    }
}

// 处理串口命令

void wifi_init()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.println("Connected to WiFi");

    wifi_connected = true;

    udp.begin(8081);
    Serial.println("UDP initialized");
    Serial.printf("UDP发送目标: %s:%d\n", server_ip, server_port);
}

void setup()
{
    Serial1.begin(1000000, SERIAL_8N1, 44, 43);
    Serial.begin(115200);
    hlscl.pSerial = &Serial1;

    delay(1000);

    int targetPulse = TARGET_ANGLE / 360.0 * 4095;

    Serial.println("等待所有舵机到达目标位置...");
    delay(5000);

    for (int id = MIN_SERVO_ID; id <= MAX_SERVO_ID; id++)
    {
        int currentPos = hlscl.ReadPos(id);
        hlscl.WritePosEx(id, currentPos, 0, 0, 0);
    }

    // 执行初始化零点校正
    initializeZeroCalibration();

    wifi_init();

    Serial.println("\n0-7号舵机已进入被动模式（无扭矩输出），可通过外部力量转动");
    Serial.println("多舵机角度反馈开始...");
    Serial.println("\n零点校正功能:");
    Serial.println("  输入 'cal' 或 'calibrate' 开始手动零点校正");
    Serial.println("  输入 'init_cal' 重新执行初始化零点校正");
    Serial.println("  输入 'reset' 重置所有零点校正和多圈计数");
    Serial.println("  输入 'reset_turns' 只重置多圈计数");
    Serial.println("  输入 'help' 查看帮助信息\n");

    // 初始化时间戳
    previousMillis = millis();
    serialPrintMillis = millis();
    wifiSendMillis = millis();
}

void loop()
{
    unsigned long currentMillis = millis();

    readSensorData();

    // 串口打印（1000ms间隔）
    printSensorData();
    // WiFi发送（500ms间隔）

    sendWifiData();

    delay(1);
}