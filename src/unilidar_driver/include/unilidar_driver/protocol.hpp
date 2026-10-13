// Unitree L1 / L2 雷达报文结构定义。
//
// 字段布局参考 unitree_lidar_sdk2 的 unitree_lidar_protocol.h（BSD-3-Clause），
// 本文件按其公开字段自行重述，运行时不链接官方静态库 libunitree_lidar_sdk.a。
// 所有结构体按 1 字节打包，避免不同编译器的成员对齐方式导致偏移漂移。

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
  uint32_t crc32;           // 覆盖「帧头 + 数据段」，不含本结构
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
