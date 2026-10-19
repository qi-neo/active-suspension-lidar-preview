// Unitree L1 驱动节点：收 UDP 报文 -> 解成点云与 IMU -> 发布成标准 ROS 话题。
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
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

#include "unilidar_driver/frame_decoder.hpp"
#include "unilidar_driver/packet_parser.hpp"
#include "unilidar_driver/udp_receiver.hpp"

using namespace unilidar_driver;

namespace {

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
    if (!rx_.open(local_port_, local_ip_, recv_timeout_ms_, err)) {
      RCLCPP_FATAL(get_logger(), "UDP 打开失败：%s", err.c_str());
      throw std::runtime_error("UDP 打开失败");
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

    RCLCPP_INFO(get_logger(),
                "驱动就绪 | 监听 %s:%d | 点云 %s(%s) 每 %d 条扫描合成一帧 | IMU %s(%s) "
                "四元数顺序 %s",
                local_ip_.empty() ? "0.0.0.0" : local_ip_.c_str(), local_port_,
                cloud_topic_.c_str(), cloud_frame_.c_str(), cloud_scan_num_,
                imu_topic_.c_str(), imu_frame_.c_str(), quat_order_str_.c_str());
  }

  void declareParams() {
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

    if (cloud_scan_num_ < 1) {
      RCLCPP_WARN(get_logger(), "cloud_scan_num=%d 不合法，已改为 1", cloud_scan_num_);
      cloud_scan_num_ = 1;
    }
    if (poll_period_ms_ < 1) poll_period_ms_ = 1;

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
      const int n = rx_.receive(buf, sizeof(buf), err, &src);
      if (n == 0) break;  // 本轮没有更多数据
      if (n < 0) {
        RCLCPP_ERROR(get_logger(), "%s", err.c_str());
        break;
      }
      // 指定了来源 IP 时只认那台雷达，避免别人的点云混进控制环
      if (!source_ip_.empty() && src != source_ip_) {
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
    std::vector<PointXYZI> pts;
    double stamp = 0.0;
    if (!decodePointCloud(frame, params_, wallNowSec(), pts, stamp, dstats_)) return;

    if (pending_scans_ == 0) pending_stamp_ = stamp;
    pending_.insert(pending_.end(), pts.begin(), pts.end());
    ++pending_scans_;

    if (pending_scans_ >= static_cast<std::size_t>(cloud_scan_num_)) {
      publishCloud(pending_, pending_stamp_);
      pending_.clear();
      pending_scans_ = 0;
    }
  }

  void publishCloud(const std::vector<PointXYZI>& pts, double stamp) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = toRosTime(stamp);
    msg.header.frame_id = cloud_frame_;
    msg.height = 1;
    msg.width = static_cast<std::uint32_t>(pts.size());
    msg.fields.resize(4);
    const char* names[4] = {"x", "y", "z", "intensity"};
    for (int i = 0; i < 4; ++i) {
      msg.fields[static_cast<std::size_t>(i)].name = names[i];
      msg.fields[static_cast<std::size_t>(i)].offset = static_cast<std::uint32_t>(i) * 4;
      msg.fields[static_cast<std::size_t>(i)].datatype = sensor_msgs::msg::PointField::FLOAT32;
      msg.fields[static_cast<std::size_t>(i)].count = 1;
    }
    msg.point_step = 16;
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = false;  // 已剔除无效点，下标不再连续
    msg.data.resize(msg.row_step);

    std::size_t off = 0;
    for (const PointXYZI& p : pts) {
      const float v[4] = {p.x, p.y, p.z, p.intensity};
      std::memcpy(&msg.data[off], v, sizeof(v));
      off += sizeof(v);
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
                "收到 %lu 帧 序号缺口 %lu | 点云已发 %lu（累计 %lu 点）IMU 已发 %lu | "
                "校验错 CRC %lu 帧尾 %lu 长度 %lu | 重同步 %lu 字节 | 长度不符提示 %lu",
                static_cast<unsigned long>(ps.frames_ok),
                static_cast<unsigned long>(ps.seq_gaps),
                static_cast<unsigned long>(clouds_published_),
                static_cast<unsigned long>(dstats_.points_out),
                static_cast<unsigned long>(imus_published_),
                static_cast<unsigned long>(ps.bad_crc),
                static_cast<unsigned long>(ps.bad_tail),
                static_cast<unsigned long>(ps.bad_size),
                static_cast<unsigned long>(ps.resync_bytes),
                static_cast<unsigned long>(dstats_.size_hint));
    if (filtered_ > 0) {
      RCLCPP_INFO(get_logger(), "已丢弃来自非指定来源的数据 %lu 条",
                  static_cast<unsigned long>(filtered_));
    }

    if (ps.frames_ok == 0 && ps.bad_crc + ps.bad_size + ps.bad_tail > 0) {
      RCLCPP_WARN(get_logger(),
                  "一帧都没解出却不乏错误计数：先看 bad_crc 是否在涨——若持续在涨而 bad_tail "
                  "很少，最常见的是 CRC 覆盖范围与本固件不一致（官方实现只覆盖数据段）；"
                  "否则通常是 IP/端口不对，或对面根本不是宇树雷达。"
                  "可用 udp_probe 工具先看原始包的实际长度与 CRC 情况。");
    }
    if (dstats_.size_hint > 0) {
      RCLCPP_WARN(get_logger(),
                  "整包长度与本驱动预期不同（累计 %lu 次）。若点云形状明显不对，"
                  "请把 udp_probe 报告的实际长度反馈回来核对固件版本。",
                  static_cast<unsigned long>(dstats_.size_hint));
    }
  }

  static constexpr int kMaxRecvPerPoll = 64;

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
  double diag_period_s_{5.0};

  DecodeParams params_;
  DecodeStats dstats_;
  PacketParser parser_;
  UdpReceiver rx_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::TimerBase::SharedPtr poll_timer_;
  rclcpp::TimerBase::SharedPtr diag_timer_;

  std::vector<PointXYZI> pending_;
  std::size_t pending_scans_{0};
  double pending_stamp_{0.0};
  std::uint64_t clouds_published_{0};
  std::uint64_t imus_published_{0};
  std::uint64_t filtered_{0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<UnilidarDriverNode>());
  } catch (const std::exception& e) {
    std::fprintf(stderr, "驱动退出：%s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
