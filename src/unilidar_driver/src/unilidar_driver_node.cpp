// Unitree L1 驱动节点：收报文 -> 解成点云与 IMU -> 发布成标准 ROS 话题。
//
// 传输层二选一（transport 参数）：
//   serial —— 实物默认路径。L1 的原生通讯接口就是 TTL UART，官方 SDK 默认
//             经 USB 转串口模块（CH340 类）以 2,000,000 bps 接入，设备名
//             通常为 /dev/ttyUSB0；
//   udp    —— 备选路径。L1 以太网口 192.168.1.62:6101 发往本机 6201。
// 两条路径喂给 packet_parser 的都是无边界字节流，解析与发布完全共用。
//
// 本文件只做「参数、收包、发布」的接线，协议解析与几何换算都在 unilidar_driver
// 核心库里，不依赖 ROS，可单独离线自检。

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "unilidar_driver/frame_decoder.hpp"
#include "unilidar_driver/packet_parser.hpp"
#include "unilidar_driver/serial_receiver.hpp"
#include "unilidar_driver/udp_receiver.hpp"

using namespace unilidar_driver;

namespace {

enum class Transport { kSerial, kUdp };

double wallNowSec() {
  const auto now = std::chrono::system_clock::now();
  return std::chrono::duration<double>(now.time_since_epoch()).count();
}

rclcpp::Time toRosTime(double sec) {
  const double whole = std::floor(sec);
  const int64_t s = static_cast<int64_t>(whole);
  const uint32_t ns = static_cast<uint32_t>(std::llround((sec - whole) * 1e9));
  return rclcpp::Time(s, ns, RCL_SYSTEM_TIME);
}

}  // namespace

class UnilidarDriverNode : public rclcpp::Node {
public:
  UnilidarDriverNode() : rclcpp::Node("unilidar_driver_node") { init(); }

private:
  void init() {
    declareParams();

    std::string err;
    if (transport_ == Transport::kSerial) {
      if (!serial_rx_.open(serial_device_, serial_baud_, recv_timeout_ms_, err)) {
        RCLCPP_FATAL(get_logger(), "串口打开失败：%s", err.c_str());
        RCLCPP_FATAL(get_logger(),
                     "排查顺序：1) ls %s 确认设备存在（CH340 类模块枚举为 ttyUSB*）；"
                     "2) 把当前用户加入 dialout 组后重新登录：sudo usermod -a -G dialout $USER；"
                     "3) 确认波特率 %d 是官方 SDK 默认值 2000000，换错会一个整帧都解不出；"
                     "4) 确认 L1 的 UART 口接线（TX/RX 交叉、共地）。",
                     serial_device_.c_str(), serial_baud_);
        throw std::runtime_error("串口打开失败");
      }
    } else {
      if (!rx_.open(local_port_, local_ip_, recv_timeout_ms_, recv_buf_bytes_, err)) {
        RCLCPP_FATAL(get_logger(), "UDP 打开失败：%s", err.c_str());
        throw std::runtime_error("UDP 打开失败");
      }
    }

    // 下游（v2 预瞄节点、v3 控制节点）都用 SensorDataQoS 订阅，即 BEST_EFFORT。
    // 这里必须同 QoS 发布，否则 RELIABLE 与 BEST_EFFORT 不兼容，
    // 表现为雷达明明在发包，后面却一个字节都收不到。
    cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic_,
                                                                 rclcpp::SensorDataQoS());
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, rclcpp::SensorDataQoS());

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    publishStaticTf();

    poll_timer_ = create_wall_timer(std::chrono::milliseconds(poll_period_ms_),
                                    [this]() { pollOnce(); });
    diag_timer_ = create_wall_timer(std::chrono::duration<double>(diag_period_s_),
                                    [this]() { logDiagnostics(); });

    if (transport_ == Transport::kSerial) {
      RCLCPP_INFO(get_logger(),
                  "驱动就绪 | 串口 %s @ %d bps（L1 原生 TTL UART 经 CH340 类模块转 USB）| "
                  "点云 %s(%s) 每 %d 条扫描合成一帧 | IMU %s(%s) 四元数顺序 %s",
                  serial_device_.c_str(), serial_baud_, cloud_topic_.c_str(),
                  cloud_frame_.c_str(), cloud_scan_num_, imu_topic_.c_str(), imu_frame_.c_str(),
                  quat_order_str_.c_str());
      if (!source_ip_.empty()) {
        RCLCPP_WARN(get_logger(),
                    "transport=serial 时 source_ip 不生效（串口没有来源地址概念），已忽略。");
      }
    } else {
      RCLCPP_INFO(get_logger(),
                  "驱动就绪 | 监听 %s:%d | 点云 %s(%s) 每 %d 条扫描合成一帧 | IMU %s(%s) "
                  "四元数顺序 %s",
                  local_ip_.empty() ? "0.0.0.0" : local_ip_.c_str(), local_port_,
                  cloud_topic_.c_str(), cloud_frame_.c_str(), cloud_scan_num_,
                  imu_topic_.c_str(), imu_frame_.c_str(), quat_order_str_.c_str());
      RCLCPP_INFO(get_logger(),
                  "收包缓冲 SO_RCVBUF 请求 %d 字节，内核实际生效 %d 字节"
                  "（Linux 会翻倍并受 net.core.rmem_max 裁剪）",
                  recv_buf_bytes_, rx_.recvBufBytes());
    }
  }

  // 参数非法时不是「悄悄改成一个能跑的值」：静默回退会让用户以为配置生效了，
  // 实际行为与配置不符，排障时极难发现。这里一律 FATAL 打印后抛异常，
  // 由 main 捕获并以非 0 退出码结束进程。
  [[noreturn]] void fatalParam(const std::string& msg) {
    RCLCPP_FATAL(get_logger(), "参数非法：%s", msg.c_str());
    throw std::runtime_error("参数非法：" + msg);
  }

  void checkParam(bool ok, const std::string& msg) {
    if (!ok) fatalParam(msg);
  }

  void declareParams() {
    // 传输层：serial 为实物默认路径（L1 原生 TTL UART 经 CH340 类模块转 USB，
    // /dev/ttyUSB0 @ 2 Mbps）；udp 为以太网备选路径。
    const std::string transport_str = declare_parameter<std::string>("transport", "serial");
    if (transport_str == "serial") {
      transport_ = Transport::kSerial;
    } else if (transport_str == "udp") {
      transport_ = Transport::kUdp;
    } else {
      fatalParam("transport=\"" + transport_str + "\"，只接受 serial 或 udp");
    }
    serial_device_ = declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
    serial_baud_ = declare_parameter<int>("serial_baud", 2000000);

    local_ip_ = declare_parameter<std::string>("local_ip", "");
    local_port_ = declare_parameter<int>("local_port", 6201);
    source_ip_ = declare_parameter<std::string>("source_ip", "");

    cloud_frame_ = declare_parameter<std::string>("cloud_frame", "unilidar_lidar");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/unilidar/cloud");
    imu_frame_ = declare_parameter<std::string>("imu_frame", "unilidar_imu");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/unilidar/imu");

    cloud_scan_num_ = declare_parameter<int>("cloud_scan_num", 18);
    // 默认 xyzw：官方 ROS2 节点填 Imu 话题就是这么取的（orientation.x = quaternion[0]）。
    // 官方同一文件广播 TF 时又按 wxyz 取，两处矛盾，所以这个参数必须能改；
    // 台架上用"静止水平时 |w|≈1"确认一次即可。
    quat_order_str_ = declare_parameter<std::string>("quaternion_order", "xyzw");
    use_system_timestamp_ = declare_parameter<bool>("use_system_timestamp", true);
    range_min_ = declare_parameter<double>("range_min", 0.0);
    range_max_ = declare_parameter<double>("range_max", 100.0);
    publish_tf_ = declare_parameter<bool>("publish_tf", true);

    poll_period_ms_ = declare_parameter<int>("poll_period_ms", 2);
    recv_timeout_ms_ = declare_parameter<int>("recv_timeout_ms", 1);
    diag_period_s_ = declare_parameter<double>("diag_period_s", 5.0);
    // 雷达约 180 KB/s，内核默认 rmem（约 200 KB）在调度抖动时会溢出丢包。
    // 0 表示不设置、沿用系统默认；默认请求 8 MiB。
    recv_buf_bytes_ = declare_parameter<int>("recv_buf_bytes", kDefaultRecvBufBytes);

    checkParam(local_port_ >= 1 && local_port_ <= 65535,
               "local_port=" + std::to_string(local_port_) + "，应在 1..65535 之间");
    if (transport_ == Transport::kSerial) {
      checkParam(!serial_device_.empty(), "serial_device 不能为空（transport=serial 时必填）");
      checkParam(serial_baud_ > 0, "serial_baud=" + std::to_string(serial_baud_) + "，应 > 0");
      // 波特率是否可配置取决于平台 termios 常量表，提前问出来，
      // 否则错误只会在 open 里以「不支持」暴露，排障时离现场远。
      checkParam(serialBaudSupported(serial_baud_),
                 "serial_baud=" + std::to_string(serial_baud_) +
                     "，本平台 termios 无对应常量（L1 官方 SDK 默认 2000000）");
    }
    checkParam(quat_order_str_ == "wxyz" || quat_order_str_ == "xyzw",
               "quaternion_order=\"" + quat_order_str_ + "\"，只接受 wxyz 或 xyzw");
    checkParam(cloud_scan_num_ >= 1 && cloud_scan_num_ <= 65535,
               "cloud_scan_num=" + std::to_string(cloud_scan_num_) +
                   "，应在 1..65535 之间（点云 ring 字段为 UINT16，上限 65535）");
    checkParam(poll_period_ms_ >= 1,
               "poll_period_ms=" + std::to_string(poll_period_ms_) + "，应 >= 1");
    checkParam(range_min_ < range_max_,
               "range_min=" + std::to_string(range_min_) + " 应小于 range_max=" +
                   std::to_string(range_max_));
    checkParam(recv_buf_bytes_ >= 0,
               "recv_buf_bytes=" + std::to_string(recv_buf_bytes_) + "，应 >= 0（0 表示用系统默认）");

    params_.quat_order = (quat_order_str_ == "xyzw") ? QuatOrder::kXyzw : QuatOrder::kWxyz;
    params_.use_system_timestamp = use_system_timestamp_;
    params_.range_min = range_min_;
    params_.range_max = range_max_;
  }

  void publishStaticTf() {
    // IMU 原点位于雷达坐标系的 [-0.007698, -0.014655, 0.00667]，
    // 这里广播的是它的逆变换：imu_frame -> cloud_frame
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = now();
    tf.header.frame_id = imu_frame_;
    tf.child_frame_id = cloud_frame_;
    tf.transform.translation.x = 0.007698;
    tf.transform.translation.y = 0.014655;
    tf.transform.translation.z = -0.00667;
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    static_broadcaster_->sendTransform(tf);
  }

  void publishImuTf(const ImuSample& s) {
    if (!publish_tf_) return;
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = toRosTime(s.stamp);
    tf.header.frame_id = imu_frame_ + "_initial";
    tf.child_frame_id = imu_frame_;
    tf.transform.translation.x = 0.0;
    tf.transform.translation.y = 0.0;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.x = s.quat[0];
    tf.transform.rotation.y = s.quat[1];
    tf.transform.rotation.z = s.quat[2];
    tf.transform.rotation.w = s.quat[3];
    tf_broadcaster_->sendTransform(tf);
  }

  void pollOnce() {
    std::uint8_t buf[65536];
    for (int i = 0; i < kMaxRecvPerPoll; ++i) {
      std::string err;
      std::string src;
      int n = 0;
      if (transport_ == Transport::kSerial) {
        // 串口是字节流：不存在「一条报文一次 read」的边界，readSome 返回多少
        // 就喂多少，定界交给 packet_parser。0 = 当前无数据（非阻塞）。
        n = serial_rx_.readSome(buf, sizeof(buf), err);
      } else {
        n = rx_.receive(buf, sizeof(buf), err, &src);
      }
      if (n == 0) break;  // 本轮没有更多数据
      if (n < 0) {
        RCLCPP_ERROR(get_logger(), "%s", err.c_str());
        break;
      }
      // 指定了来源 IP 时只认那台雷达，避免别人的点云混进控制环（仅 UDP 有意义）
      if (transport_ == Transport::kUdp && !source_ip_.empty() && src != source_ip_) {
        filtered_++;
        continue;
      }
      parser_.append(buf, static_cast<std::size_t>(n));
    }
    drainFrames();
  }

  void drainFrames() {
    std::vector<std::uint8_t> frame;
    std::uint32_t type = 0;
    while (parser_.pull(frame, type) == PullStatus::kFrameReady) {
      if (type == kTypePointCloud3D) {
        handleCloud(frame);
      } else if (type == kTypeImu) {
        handleImu(frame);
      }
    }
  }

  void handleCloud(const std::vector<std::uint8_t>& frame) {
    // ring = 这条扫描线在整圈中的序号（1 基，与官方 SDK 的 ring 从 1 起算一致）。
    // 协议里一个点云报文就是一条扫描线，单包内没有"第几条线"字段（官方单包内
    // 每点 ring 写死为 1），只能由拼帧顺序给出：本帧第 k 条线（k 从 0 起）→ k+1。
    std::vector<PointXYZIT> pts;
    double stamp = 0.0;
    const std::uint16_t ring = static_cast<std::uint16_t>(pending_scans_ + 1);
    if (!decodePointCloud(frame, params_, wallNowSec(), pts, stamp, dstats_, ring)) return;

    if (pending_scans_ == 0) pending_stamp_ = stamp;
    // 解码器给的 time 是相对**本条线**起始时刻的偏移；发布的这一帧以第一条线的
    // 起始时刻作 header.stamp，所以要把后续每条线整体平移 (本线 - 首线)，
    // 使全帧的 time 都以 header.stamp 为基准。这样跨线时间仍然连续递增，
    // 且 float 只在 0..0.2 s 的小量级上量化，精度才有意义。
    const double line_shift = stamp - pending_stamp_;
    if (line_shift != 0.0) {
      for (PointXYZIT& p : pts) {
        p.time = static_cast<float>(static_cast<double>(p.time) + line_shift);
      }
    }
    pending_.insert(pending_.end(), pts.begin(), pts.end());
    ++pending_scans_;

    if (pending_scans_ >= static_cast<std::size_t>(cloud_scan_num_)) {
      publishCloud(pending_, pending_stamp_);
      pending_.clear();
      pending_scans_ = 0;
    }
  }

  // PointCloud2 字段布局（point_step = 24）：
  //   偏移 0   x          FLOAT32
  //   偏移 4   y          FLOAT32
  //   偏移 8   z          FLOAT32
  //   偏移 12  intensity  FLOAT32
  //   偏移 16  time       FLOAT32  相对 header.stamp 的偏移秒数（非绝对时间）
  //   偏移 20  ring       UINT16   该扫描线在整圈中的 1 基序号
  //   偏移 22  2 字节 padding
  // padding 不是可选的：环号只占 2 字节，若把 point_step 压成 22，下一个点的 x
  // 就落在 2 mod 4 的地址上，任何按 float 读下一点的下游都会读到未对齐数据。
  // 取 24（4 的倍数）后每点起点仍 4 字节对齐；下游按字段名/偏移读取，会跳过 padding。
  static constexpr std::uint32_t kPointStep = 24;

  void publishCloud(const std::vector<PointXYZIT>& pts, double stamp) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = toRosTime(stamp);
    msg.header.frame_id = cloud_frame_;
    msg.height = 1;
    msg.width = static_cast<std::uint32_t>(pts.size());

    struct FieldSpec {
      const char* name;
      std::uint32_t offset;
      std::uint8_t datatype;
    };
    const FieldSpec specs[] = {
        {"x", 0, sensor_msgs::msg::PointField::FLOAT32},
        {"y", 4, sensor_msgs::msg::PointField::FLOAT32},
        {"z", 8, sensor_msgs::msg::PointField::FLOAT32},
        {"intensity", 12, sensor_msgs::msg::PointField::FLOAT32},
        {"time", 16, sensor_msgs::msg::PointField::FLOAT32},
        {"ring", 20, sensor_msgs::msg::PointField::UINT16},
    };
    constexpr std::size_t kFieldCount = sizeof(specs) / sizeof(specs[0]);
    msg.fields.resize(kFieldCount);
    for (std::size_t i = 0; i < kFieldCount; ++i) {
      msg.fields[i].name = specs[i].name;
      msg.fields[i].offset = specs[i].offset;
      msg.fields[i].datatype = specs[i].datatype;
      msg.fields[i].count = 1;
    }

    msg.point_step = kPointStep;
    msg.row_step = kPointStep * msg.width;
    msg.is_dense = false;  // 已剔除无效点，下标不再连续
    msg.data.assign(static_cast<std::size_t>(msg.row_step), 0);  // padding 字节保持 0

    // 逐字段 memcpy：msg.data 缓冲区本身不保证 4 字节对齐，不能直接按 float* 写。
    std::size_t off = 0;
    for (const PointXYZIT& p : pts) {
      std::memcpy(&msg.data[off + 0], &p.x, sizeof(p.x));
      std::memcpy(&msg.data[off + 4], &p.y, sizeof(p.y));
      std::memcpy(&msg.data[off + 8], &p.z, sizeof(p.z));
      std::memcpy(&msg.data[off + 12], &p.intensity, sizeof(p.intensity));
      std::memcpy(&msg.data[off + 16], &p.time, sizeof(p.time));
      std::memcpy(&msg.data[off + 20], &p.ring, sizeof(p.ring));
      off += kPointStep;
    }

    cloud_pub_->publish(msg);
    clouds_published_++;
  }

  void handleImu(const std::vector<std::uint8_t>& frame) {
    ImuSample s;
    if (!decodeImu(frame, params_, wallNowSec(), s, dstats_)) return;

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = toRosTime(s.stamp);
    msg.header.frame_id = imu_frame_;
    msg.orientation.x = s.quat[0];
    msg.orientation.y = s.quat[1];
    msg.orientation.z = s.quat[2];
    msg.orientation.w = s.quat[3];
    msg.angular_velocity.x = s.angular_velocity[0];
    msg.angular_velocity.y = s.angular_velocity[1];
    msg.angular_velocity.z = s.angular_velocity[2];
    msg.linear_acceleration.x = s.linear_acceleration[0];
    msg.linear_acceleration.y = s.linear_acceleration[1];
    msg.linear_acceleration.z = s.linear_acceleration[2];

    imu_pub_->publish(msg);
    imus_published_++;
    publishImuTf(s);
  }

  void logDiagnostics() {
    const ParseStats& ps = parser_.stats();
    RCLCPP_INFO(get_logger(),
                "收到 %lu 帧 序号缺口 %lu 重复包 %lu 乱序 %lu | 点云已发 %lu（累计 %lu 点）"
                "IMU 已发 %lu | 校验错 CRC %lu 帧尾 %lu 长度 %lu | 重同步 %lu 字节 | "
                "长度不符提示 %lu | 逐点时间步长异常 %lu",
                static_cast<unsigned long>(ps.frames_ok),
                static_cast<unsigned long>(ps.seq_gaps),
                static_cast<unsigned long>(ps.duplicates),
                static_cast<unsigned long>(ps.out_of_order),
                static_cast<unsigned long>(clouds_published_),
                static_cast<unsigned long>(dstats_.points_out),
                static_cast<unsigned long>(imus_published_),
                static_cast<unsigned long>(ps.bad_crc),
                static_cast<unsigned long>(ps.bad_tail),
                static_cast<unsigned long>(ps.bad_size),
                static_cast<unsigned long>(ps.resync_bytes),
                static_cast<unsigned long>(dstats_.size_hint),
                static_cast<unsigned long>(dstats_.time_increment_invalid));
    if (transport_ == Transport::kUdp) {
      RCLCPP_INFO(get_logger(), "UDP 收包缓冲 SO_RCVBUF 实际生效 %d 字节（请求 recv_buf_bytes=%d）",
                  rx_.recvBufBytes(), recv_buf_bytes_);
    }
    if (filtered_ > 0) {
      RCLCPP_INFO(get_logger(), "已丢弃来自非指定来源的数据 %lu 条",
                  static_cast<unsigned long>(filtered_));
    }

    // 累计值每轮都打印，告警只看「新增」缺口，避免刷屏。
    if (ps.seq_gaps > last_seq_gaps_) {
      if (transport_ == Transport::kUdp) {
        RCLCPP_WARN(get_logger(),
                    "本周期新增序号缺口 %lu（累计 %lu）：可能正在丢包。优先确认收包缓冲是否够大——"
                    "当前 SO_RCVBUF 实际生效 %d 字节（请求 recv_buf_bytes=%d），"
                    "可调大到 16 MiB / 32 MiB 后重启；其次检查 poll_period_ms 是否被调度延迟、"
                    "本机 CPU 是否过载。若缺口只在启动瞬间出现一次，通常是进程启动晚于雷达。",
                    static_cast<unsigned long>(ps.seq_gaps - last_seq_gaps_),
                    static_cast<unsigned long>(ps.seq_gaps), rx_.recvBufBytes(), recv_buf_bytes_);
      } else {
        RCLCPP_WARN(get_logger(),
                    "本周期新增序号缺口 %lu（累计 %lu）：可能正在丢包。串口路径优先确认："
                    "1) 波特率是否为 2000000（L1 官方 SDK 默认），错配会持续 CRC 失败与缺口；"
                    "2) USB 线缆/转接模块质量，2 Mbps 下劣质线会丢字节；"
                    "3) poll_period_ms=%d 是否被调度延迟、本机 CPU 是否过载。"
                    "若缺口只在启动瞬间出现一次，通常是进程启动晚于雷达。",
                    static_cast<unsigned long>(ps.seq_gaps - last_seq_gaps_),
                    static_cast<unsigned long>(ps.seq_gaps), poll_period_ms_);
      }
      last_seq_gaps_ = ps.seq_gaps;
    }

    if (ps.frames_ok == 0 && ps.bad_crc + ps.bad_size + ps.bad_tail > 0) {
      if (transport_ == Transport::kUdp) {
        RCLCPP_WARN(get_logger(),
                    "一帧都没解出却不乏错误计数：先看 bad_crc 是否在涨——若持续在涨而 bad_tail "
                    "很少，最常见的是 CRC 覆盖范围与本固件不一致（官方实现只覆盖数据段）；"
                    "否则通常是 IP/端口不对，或对面根本不是宇树雷达。"
                    "可用 udp_probe 工具先看原始包的实际长度与 CRC 情况。");
      } else {
        RCLCPP_WARN(get_logger(),
                    "一帧都没解出却不乏错误计数：先看 bad_crc 是否在涨——串口路径最常见的原因是"
                    "波特率不是 2000000 或线序/共地有问题（错一个字节后续帧全部 CRC 失败）；"
                    "其次确认对面真的是 L1（TX/RX 是否交叉）。可用 serial_receiver_selftest "
                    "先验证本机 PTY 上的定界逻辑。");
      }
    }
    if (dstats_.size_hint > 0) {
      if (transport_ == Transport::kUdp) {
        RCLCPP_WARN(get_logger(),
                    "整包长度与本驱动预期不同（累计 %lu 次）。若点云形状明显不对，"
                    "请把 udp_probe 报告的实际长度反馈回来核对固件版本。",
                    static_cast<unsigned long>(dstats_.size_hint));
      } else {
        RCLCPP_WARN(get_logger(),
                    "整包长度与本驱动预期不同（累计 %lu 次）。若点云形状明显不对，"
                    "请记录串口实际收到的整包长度反馈回来核对固件版本"
                    "（可用 od 抓包或临时加打印统计）。",
                    static_cast<unsigned long>(dstats_.size_hint));
      }
    }
    // time_increment 是逐点时间唯一的来源；它无效时 time 偏移全为 0，
    // 去畸变会算成"整条线同一时刻"，必须让用户看见。
    if (dstats_.time_increment_invalid > 0 && !warned_time_increment_) {
      warned_time_increment_ = true;
      RCLCPP_WARN(get_logger(),
                  "报文里的 time_increment 非正或非有限（累计 %lu 次），逐点 time 会退化为"
                  "帧时间戳本身。要用逐点时间去畸变，先核对固件输出的该字段。",
                  static_cast<unsigned long>(dstats_.time_increment_invalid));
    }
  }

  static constexpr int kMaxRecvPerPoll = 64;

  Transport transport_{Transport::kSerial};
  std::string serial_device_;
  int serial_baud_{2000000};

  std::string local_ip_;
  int local_port_{6201};
  std::string source_ip_;
  std::string cloud_frame_;
  std::string cloud_topic_;
  std::string imu_frame_;
  std::string imu_topic_;
  int cloud_scan_num_{18};
  std::string quat_order_str_;
  bool use_system_timestamp_{true};
  double range_min_{0.0};
  double range_max_{100.0};
  bool publish_tf_{true};
  int poll_period_ms_{2};
  int recv_timeout_ms_{1};
  int recv_buf_bytes_{kDefaultRecvBufBytes};
  double diag_period_s_{5.0};

  DecodeParams params_;
  DecodeStats dstats_;
  PacketParser parser_;
  SerialReceiver serial_rx_;  // transport=serial 时使用
  UdpReceiver rx_;           // transport=udp 时使用
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  std::vector<PointXYZIT> pending_;
  std::size_t pending_scans_{0};
  double pending_stamp_{0.0};
  std::uint64_t clouds_published_{0};
  std::uint64_t imus_published_{0};
  std::uint64_t filtered_{0};
  std::uint64_t last_seq_gaps_{0};  // 上一轮诊断时的 seq_gaps，用于只报新增缺口
  bool warned_time_increment_{false};  // time_increment 无效只告警一次，避免刷屏
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  int rc = 0;
  try {
    rclcpp::spin(std::make_shared<UnilidarDriverNode>());
  } catch (const std::exception& e) {
    // 参数校验失败等致命错误必须让进程以非 0 退出，否则启动脚本/CI
    // 只看退出码时会误以为驱动正常起来了。
    std::fprintf(stderr, "驱动退出：%s\n", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
