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
#define MAX_SERVO_ID 15
#define SERVO_COUNT (MAX_SERVO_ID - MIN_SERVO_ID + 1)
#define TARGET_ANGLE 180
#define MOVE_SPEED 500
#define PACKET_HEADER 0xAA
#define PACKET_FOOTER 0x55
#define PACKET_SIZE 34

const char *ssid = "ikuai-hupa2.4G";
const char *password = "Hupa@2018";
// const char *server_ip = "192.168.88.92";
const char *server_ip = "192.168.88.131";
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
Sensor_Data sensor[20];

// 初始化零点校正函数
void initializeZeroCalibration()
{
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
}

// 手动零点校正函数
void calibrateZeroPoint()
{
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

// 数据读取函数
void readSensorData()
{
    
    for (int id = MIN_SERVO_ID; id <= MAX_SERVO_ID; id++)
    {
        sensor[id].current_pos = hlscl.ReadPos(id);
        printf("id:%f,%f",id, sensor[id].current_pos);
        sensor[id].current_speed = hlscl.ReadSpeed(id);
        sensor[id].current_mA = hlscl.ReadCurrent(id);

        // 转换为角度值（带零点校正）
            int calibrated_pos = sensor[id].current_pos;
            int raw_pos_for_multi = sensor[id].current_pos;

            // 如果已校正零点，则应用偏移
            if (sensor[id].zero_calibrated)
            {
                calibrated_pos = sensor[id].current_pos - sensor[id].zero_offset;
                raw_pos_for_multi = sensor[id].current_pos - sensor[id].zero_offset;
            }
            sensor[id].angle_deg = calibrated_pos / 4095.0 * 360;
    }
}
void sendWifiData()
{
    // Serial.println("test");
    if (wifi_connected)
    {
        String jsonData = "{\"sensors\":[";

        for (int i = 0; i < 16; i++)
        {
            if (i > 0)
                jsonData += ",";
            jsonData += "{\"id\":" + String(i) +
                        ",\"angle\":" + String(sensor[i].angle_deg) +"}";
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
        // else
        // {
        //     Serial.println("UDP success");
        // }
    }
}

// 处理串口命令

void wifi_init()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(100);
        Serial.print(".");
    }
    Serial.println();

    wifi_connected = true;

    udp.begin(8081);
    Serial.printf("sucess");
}

void setup()
{
    Serial1.begin(1000000, SERIAL_8N1, 44, 43);
    Serial.begin(115200);
    wifi_init();
    hlscl.pSerial = &Serial1;
    delay(100);
    int targetPulse = TARGET_ANGLE / 360.0 * 4095;

    for (int id = MIN_SERVO_ID; id <= MAX_SERVO_ID; id++)
    {
        int currentPos = hlscl.ReadPos(id);
      
        hlscl.WritePosEx(id, currentPos, 0, 0, 0);
    }
    // 执行初始化零点校正
    initializeZeroCalibration();
   
}

void loop()
{
    readSensorData();
    sendWifiData();
    delay(1);
}