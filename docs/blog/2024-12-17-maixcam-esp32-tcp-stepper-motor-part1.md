# Maixcam Pro 与 ESP32 通过 socket TCP 通信来控制步进电机（一）

> 原文发布于 CSDN（2024-12-17）：<https://blog.csdn.net/2401_84550508/article/details/144541721>
> 本文由 CSDN 迁移至 GitHub，**已对原文中的 Wi-Fi 密码进行脱敏处理**（占位符 `<YOUR_WIFI_PASSWORD>`）。

## 前言

本文通过 socket TCP 通信，实现 Maixcam 与 ESP32 之间相互发送信息的功能，主要讲述 **ESP32 端** 的主要操作和代码。

可拓展功能：Maixcam 识别物体坐标信息和种类后，传输给 ESP32，ESP32 控制步进电机带动机械臂到物体上方实现抓取功能……

## 一、TCP 介绍

### 1. TCP 协议的定义
TCP（Transmission Control Protocol，传输控制协议）是一种面向连接的、可靠的、基于字节流的传输层通信协议。它负责在源主机和目标主机之间建立可靠的、有序的、错误检查的数据传输通道。TCP 协议通过确认应答、超时重传、滑动窗口等机制来保证数据的可靠传输。

### 2. TCP 协议的工作原理
1. **建立连接**：TCP 在数据传输前，会通过“三次握手”与对方建立连接，确保双方都已准备好进行通信。
2. **数据传输**：连接建立后，TCP 会将数据分成多个数据包进行传输。每个数据包都会加上序列号，以确保接收方能够按照正确的顺序重新组装数据。
3. **确认应答**：接收方在收到数据包后，会向发送方发送确认应答。如果发送方在一段时间内没有收到确认应答，它会认为数据包丢失，并重新发送该数据包。
4. **流量控制**：TCP 会根据接收方的处理能力，动态调整发送方的发送速度，避免发送过快导致接收方处理不过来。
5. **断开连接**：当数据传输完成后，TCP 会通过“四次挥手”与对方断开连接。

### 3. TCP 协议的特性
1. **可靠性**：通过确认应答和重传机制确保数据包的可靠传输。
2. **有序性**：采用序列号对数据包进行排序，确保数据顺序正确。
3. **流量控制**：通过滑动窗口机制实现流量控制。
4. **拥塞控制**：通过滑动窗口、慢启动、拥塞避免等算法来防止网络拥塞。

## 二、器材选用
1. ESP32 Dev Module
2. Maixcam Pro
3. 多路开关电源
4. 杜邦线若干
5. 57 步进闭环驱动电机

## 三、代码及操作

### （一）具体步骤

#### 1. 引入库
```cpp
#include <WiFi.h>
#include <WiFiClient.h>
#include <AccelStepper.h>
```

#### 2. 定义电机引脚和参数
通过脉冲来控制步进电机转动，其中一圈设置为 3200 个脉冲。
```cpp
// 定义电机引脚
#define En_Pin2     17   // 电机1使能引脚
#define Stp_Pin2    5    // 电机1步进脉冲引脚
#define Dir_Pin2    18   // 电机1方向控制引脚
#define En_Pin1     27   // 电机2使能引脚
#define Stp_Pin1    25   // 电机2步进脉冲引脚
#define Dir_Pin1    26   // 电机2方向控制引脚

// 定义电机参数
#define CIRCLE_Puls        3200   // 电机旋转一圈的脉冲数
#define MAX_PIXEL_X        2560   // X轴最大像素点坐标
#define MAX_PIXEL_Y        2560   // Y轴最大像素点坐标
```

#### 3. 创建 AccelStepper 对象
```cpp
AccelStepper stepper1(AccelStepper::DRIVER, Stp_Pin1, Dir_Pin1);  // 电机1
AccelStepper stepper2(AccelStepper::DRIVER, Stp_Pin2, Dir_Pin2);  // 电机2
```

#### 4. 定义其它变量
```cpp
long targetPosition1 = 0;  // 初始化电机1位置
long targetPosition2 = 0;  // 初始化电机2位置
int category = 1;          // 垃圾种类整型变量，赋值为1
String receivedData = "";  // 用于存储接收到的数据
unsigned long lastDataProcessTime = 0;
const unsigned long dataProcessInterval = 10000;  // 数据处理间隔(ms)
```

#### 5. Wi-Fi 设置
WiFi 的主要作用是建立网络连接，实现 ESP32 与 Maixcam 之间的数据通信。
```cpp
// ⚠️ 以下 Wi-Fi 信息已从原文脱敏，请替换为你自己的网络
const char* ssid = "205-03";
const char* password = "<YOUR_WIFI_PASSWORD>";   // 原文为明文密码，已脱敏

const char* targetIP = "192.168.10.5";
const uint16_t targetPort = 8080;

WiFiClient client;  // ESP32 作为 TCP 客户端
```

#### 6. 初始化设置（setup）
连接指定 Wi-Fi，连接 TCP 服务器（Maixcam），配置电机参数（最大速度、初始速度、最大加速度），初始化电机位置，并用 LED 指示连接状态。完整代码见文末。

#### 7. 处理 TCP 数据并控制重连（loop）
循环接收数据、以换行符 `\n` 为完整数据包标志进行处理，断线自动重连。完整代码见文末。

#### 8. processReceivedData 函数
解析像素坐标与类别，驱动两台电机移动到计算出的位置，执行特定动作后归零，并通过串口反馈状态。完整代码见文末。

### （二）注意事项
1. 电机接电前应按一下 ESP32 的 boot 键，且不要在开始通信后才给电机通电，防止电机接收到脉冲信号后转动，带动组件与其它组件发生冲突；
2. Maixcam 在重新连接 Wi-Fi 以传输代码时，会出现 IP 改变的情况，需同步修改目标 IP，否则 ESP32 将无法连接到 Maixcam。

### （三）完整代码
```cpp
#include <WiFi.h>
#include <WiFiClient.h>
#include <AccelStepper.h>

// 定义电机引脚
#define En_Pin2     17   // 电机1使能引脚
#define Stp_Pin2    5    // 电机1步进脉冲引脚
#define Dir_Pin2    18   // 电机1方向控制引脚
#define En_Pin1     27   // 电机2使能引脚
#define Stp_Pin1    25   // 电机2步进脉冲引脚
#define Dir_Pin1    26   // 电机2方向控制引脚

// 定义电机参数
#define CIRCLE_Puls        3200   // 电机旋转一圈的脉冲数
#define MAX_PIXEL_X        2560   // X轴最大像素点坐标
#define MAX_PIXEL_Y        2560   // Y轴最大像素点坐标
#define LED 2    // LED引脚

// 创建AccelStepper对象
AccelStepper stepper1(AccelStepper::DRIVER, Stp_Pin1, Dir_Pin1);  // 电机1
AccelStepper stepper2(AccelStepper::DRIVER, Stp_Pin2, Dir_Pin2);  // 电机2

// 定义变量
long targetPosition1 = 0;
long targetPosition2 = 0;
int category = 1;
String receivedData = "";
unsigned long lastDataProcessTime = 0;
const unsigned long dataProcessInterval = 10000;

// ⚠️ Wi-Fi 密码已从原文脱敏，请替换为你自己的网络
const char* ssid = "205-03";
const char* password = "<YOUR_WIFI_PASSWORD>";
const char* targetIP = "192.168.10.5";
const uint16_t targetPort = 8080;

WiFiClient client;

void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);
  delay(200);
  digitalWrite(LED, HIGH);
  delay(200);
  digitalWrite(LED, LOW);

  Serial.begin(115200);
  delay(10);

  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println(WiFi.localIP());

  pinMode(En_Pin1, OUTPUT);
  pinMode(En_Pin2, OUTPUT);
  digitalWrite(En_Pin1, 1);
  digitalWrite(En_Pin2, 1);

  stepper1.setMaxSpeed(CIRCLE_Puls * 1000);
  stepper1.setSpeed(CIRCLE_Puls * 1.0);
  stepper1.setAcceleration(CIRCLE_Puls * 6);
  stepper2.setMaxSpeed(CIRCLE_Puls * 1000);
  stepper2.setSpeed(CIRCLE_Puls * 1.0);
  stepper2.setAcceleration(CIRCLE_Puls * 6);

  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);

  delay(2000);

  Serial.print("Connecting to ");
  Serial.print(targetIP);
  Serial.print(":");
  Serial.println(targetPort);
  while (!client.connect(targetIP, targetPort)) {
    Serial.println("Connection to host failed");
    digitalWrite(LED, HIGH);
    delay(500);
    digitalWrite(LED, LOW);
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to host");
  digitalWrite(LED, HIGH);
}

void loop() {
  if (client.connected()) {
    while (client.available()) {
      char c = client.read();
      Serial.write(c);
      receivedData += c;
      if (c == '\n') {
        unsigned long currentTime = millis();
        if (currentTime - lastDataProcessTime >= dataProcessInterval) {
          processReceivedData(receivedData);
          receivedData = "";
          lastDataProcessTime = currentTime;
        }
      }
    }
    delay(100);
  } else {
    Serial.println("Disconnected from host, trying to reconnect...");
    while (!client.connect(targetIP, targetPort)) {
      digitalWrite(LED, HIGH);
      delay(2000);
      digitalWrite(LED, LOW);
      delay(2000);
      Serial.print(".");
    }
    Serial.println("Reconnected to host");
    digitalWrite(LED, HIGH);
  }
}

void processReceivedData(String data) {
  String parts[3];
  int index = 0;
  int start = 0;
  for (int i = 0; i < data.length(); i++) {
    if (data.charAt(i) == ',') {
      parts[index] = data.substring(start, i);
      index++;
      start = i + 1;
    }
  }
  parts[index] = data.substring(start);

  int pixelValue_X = parts[0].toInt();
  int pixelValue_Y = parts[1].toInt();
  int receivedCategory = parts[2].toInt();

  if (pixelValue_X >= 0 && pixelValue_X <= MAX_PIXEL_X &&
      pixelValue_Y >= 0 && pixelValue_Y <= MAX_PIXEL_Y &&
      (receivedCategory == 1 || receivedCategory == 2 || receivedCategory == 3 || receivedCategory == 4)) {
    float circles_X = pixelValue_X / (float)MAX_PIXEL_X * 6;
    float circles_Y = pixelValue_Y / (float)MAX_PIXEL_Y * 6;
    long targetPosition1 = (long)(circles_X * CIRCLE_Puls);
    long targetPosition2 = (long)(circles_Y * CIRCLE_Puls);
    int category = receivedCategory;

    stepper2.runToNewPosition(targetPosition1);
    stepper1.runToNewPosition(targetPosition2);
    while (stepper1.distanceToGo() != 0 || stepper2.distanceToGo() != 0);

    Serial.print("1# Motor Current Position (pulses): ");
    Serial.println(stepper1.currentPosition());
    Serial.print("2# Motor Current Position (pulses): ");
    Serial.println(stepper2.currentPosition());

    delay(1000);
    if (stepper1.distanceToGo() == 0 && stepper2.distanceToGo() == 0) {
      if (receivedCategory == 1) {
        targetPosition1 = 0;
        targetPosition2 = 0;
        stepper1.runToNewPosition(targetPosition1);
        stepper2.runToNewPosition(targetPosition2);
        Serial.println("Both Motors moved to 1 position.");
      } else if (receivedCategory == 2) {
        targetPosition1 = 0;
        targetPosition2 = (long)(6 * CIRCLE_Puls);
        stepper1.runToNewPosition(targetPosition1);
        stepper2.runToNewPosition(targetPosition2);
        Serial.println("Both Motors moved to 2 position.");
      } else if (receivedCategory == 3) {
        targetPosition1 = (long)(6 * CIRCLE_Puls);
        targetPosition2 = (long)(6 * CIRCLE_Puls);
        stepper1.runToNewPosition(targetPosition1);
        stepper2.runToNewPosition(targetPosition2);
        Serial.println("Both Motors moved to 3 position.");
      } else if (receivedCategory == 4) {
        targetPosition1 = (long)(3 * CIRCLE_Puls);
        targetPosition2 = (long)(3 * CIRCLE_Puls);
        stepper1.runToNewPosition(targetPosition1);
        stepper2.runToNewPosition(targetPosition2);
        Serial.println("Both Motors moved to 4 position.");
      }
      stepper1.runToNewPosition(0);
      stepper2.runToNewPosition(0);
      Serial.println("Both Motors moved back to zero position.");
    }
    client.println("Received!");
  }
}
```

## 总结
以上就是 Maixcam 和 ESP32 通信之间有关 ESP32 的内容。本文仅仅简单介绍了 TCP 通信和 ESP32 在通信中的功能。如有错误，敬请批评指正！
