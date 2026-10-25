// Copyright (c) 2024, Unitree Robotics
//
// 本文件的报文结构定义派生自 unitree_lidar_sdk2 的
// unitree_lidar_sdk/include/unitree_lidar_protocol.h，该项目以 BSD-3-Clause
// 发布。按该许可证第 1 条，此处保留原始版权声明。完整许可证与派生范围说明
// 见仓库根目录的 NOTICE。
//
// 相对原始头文件的改动：
//   - 改用 C++ 的 struct 写法，并统一按 1 字节打包（#pragma pack(push, 1)），
//     消除不同编译器成员对齐方式导致的偏移漂移；
//   - ranges 与 intensities 的容量由字面量 300 改为命名常量 kMaxPoints3D；
//   - 补充了帧定界与越界校验所需的常量：帧头与帧尾的标识字节、各报文类型、
//     帧头帧尾长度，以及由字段实算出的整包长度 kPointPacketSize / kImuPacketSize；
//   - 增加 crc32() 声明（标准 CRC-32/ISO-HDLC，非厂商私有算法）。
//
// 运行时不链接、不调用、不 include 官方的任何文件。

#pragma once

#include <cstddef>
#include <cstdint>

namespace unilidar_driver {

// 帧头标识与帧尾标识，用于字节流里定位报文边界
constexpr uint8_t kHeaderMark[4] = {0x55, 0xAA, 0x05, 0x0A};
constexpr uint8_t kTailMark[2] = {0x00, 0xFF};

// 报文类型（官方定义值）
constexpr uint32_t kTypeAck = 101;
constexpr uint32_t kTypePointCloud3D = 102;
constexpr uint32_t kTypePointCloud2D = 103;
constexpr uint32_t kTypeImu = 104;
constexpr uint32_t kTypeVersion = 105;
constexpr uint32_t kTypeTimeStamp = 106;

// 单帧 3D 点云包携带的测距点上限
constexpr std::size_t kMaxPoints3D = 300;
constexpr std::size_t kMaxPoints2D = 1800;

// 帧头、帧尾均为 12 字节
constexpr std::size_t kHeaderSize = 12;
constexpr std::size_t kTailSize = 12;

// CRC32 覆盖范围：**只有数据段**，即 [kHeaderSize, packet_size - kTailSize)，
// 既不含 12 字节帧头，也不含 12 字节帧尾。
//
// 这个范围是对着官方"实现"核对出来的，不是照注释抄的：从官方
// unitree_lidar_sdk2 的 libunilidar_sdk2.a 反汇编 LidarPacketParser::parseByte，
// CRC 循环起点是 frame+12、终点是 frame+packet_size-12，长度 = packet_size-24；
// 而官方协议头文件里 `uint32_t crc32; // crc check of head and data` 这句注释
// 与它自己的实现相矛盾。按注释实现会导致真机每一帧都 bad_crc。
constexpr std::size_t kCrcOffset = kHeaderSize;

#pragma pack(push, 1)

// 时间戳，8 字节
struct TimeStamp {
  uint32_t sec;
  uint32_t nsec;
};

// 帧头，12 字节
struct FrameHeader {
  uint8_t header[4];      // kHeaderMark
  uint32_t packet_type;   // 报文类型，见上面的常量
  uint32_t packet_size;   // 整包字节数，含帧头与帧尾
};

// 帧尾，12 字节
struct FrameTail {
  uint32_t crc32;           // CRC32，只覆盖数据段（见 kCrcOffset 的说明）
  uint32_t msg_type_check;  // 雷达侧回执类型
  uint8_t reserve[2];
  uint8_t tail[2];          // kTailMark
};

// 数据段公共信息，16 字节
struct DataInfo {
  uint32_t seq;           // 包序号，连续递增，用于检测丢包
  uint32_t payload_size;  // 数据段字节数
  TimeStamp stamp;        // 雷达硬件时间戳
};

// 雷达内部状态，36 字节
struct LidarInsideState {
  uint32_t sys_rotation_period;  // 上电机转动周期
  uint32_t com_rotation_period;  // 下电机转动周期
  float dirty_index;             // 防护罩脏污程度
  float packet_lost_up;
  float packet_lost_down;
  float apd_temperature;
  float apd_voltage;
  float laser_voltage;
  float imu_temperature;
};

// 标定参数，32 字节
struct LidarCalibParam {
  float a_axis_dist;      // m
  float b_axis_dist;      // m
  float theta_angle_bias; // rad
  float alpha_angle_bias; // rad
  float beta_angle;       // rad
  float xi_angle;         // rad
  float range_bias;       // mm
  float range_scale;      // 无量纲
};

// 3D 点云数据段
struct LidarPointData {
  DataInfo info;
  LidarInsideState state;
  LidarCalibParam param;

  float com_horizontal_angle_start;  // 水平起始角 rad
  float com_horizontal_angle_step;   // 水平角步长 rad
  float scan_period;                 // 单条线扫周期 s
  float range_min;                   // 本帧有效最近距离 m
  float range_max;                   // 本帧有效最远距离 m
  float angle_min;                   // 垂直起始角 rad
  float angle_increment;             // 垂直角步长 rad
  float time_increment;              // 点间时间步长 s
  uint32_t point_num;                // 本帧有效点数，不超过 kMaxPoints3D

  uint16_t ranges[kMaxPoints3D];      // 距离，单位 mm
  uint8_t intensities[kMaxPoints3D];  // 反射强度 0 至 255
};

struct LidarPointDataPacket {
  FrameHeader header;
  LidarPointData data;
  FrameTail tail;
};

// IMU 数据段
struct LidarImuData {
  DataInfo info;
  float quaternion[4];           // 原始四元数，分量顺序与固件有关
  float angular_velocity[3];     // 三轴角速度 rad/s
  float linear_acceleration[3];  // 三轴线加速度 m/s^2
};

struct LidarImuDataPacket {
  FrameHeader header;
  LidarImuData data;
  FrameTail tail;
};

#pragma pack(pop)

// 3D 点云整包字节数。固件若调整了结构应当同步此值；
// 解析器允许用 strict_packet_size 关闭等值校验，按实际长度解析。
constexpr std::size_t kPointPacketSize = sizeof(LidarPointDataPacket);
constexpr std::size_t kImuPacketSize = sizeof(LidarImuDataPacket);

// 标准 CRC-32（IEEE 802.3）：反射多项式 0xEDB88320，初值 0xFFFFFFFF，结果取反。
uint32_t crc32(const uint8_t* buf, std::size_t len);

}  // namespace unilidar_driver
