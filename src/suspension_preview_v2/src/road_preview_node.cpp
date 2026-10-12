// ============================================================================
//  road_preview_node.cpp  —  激光雷达路面预瞄节点（v2）
//
//  相比 v1（src/suspension_preview/src/preview_node.cpp）的改动，逐条对应代码审查报告：
//    P1-1  订阅使用 SensorDataQoS（避免与 BEST_EFFORT 发布方 QoS 不兼容）
//    P1-2  串口设置 cfmakeraw()，关闭行规程与硬件流控，二进制协议不会被改写
//    P1-3  TF 失败时丢弃该帧（fail-safe），不再用未变换点云下发数据
//    P1-4  TF 使用点云采集时刻，超时缩短且可配置
//    P1-5  串口阻塞写满 + tcdrain() + 失败计数，不再静默丢数据
//    P2-1  地面高程窗口参数化，并用安装高度做合理性校验
//    P2-2  无效格点输出 NaN + 有效掩码，可选线性插值补洞
//    P2-3  格内取中位数（可切均值），格内高低差超阈值判为障碍物
//    P2-4  格数由 preview_dist / resolution 推导，不再三处硬编码
//    P2-5  全部参数化（declare_parameter + YAML + launch）
//    P2-6  串口载荷改 int16 毫米，带宽减半
//    P2-7  校验改 CRC16-CCITT
//    P2-8  memcpy + 显式小端编码，消除类型双关
//    P2-9  独立发送线程 + 最新帧邮箱（丢旧保新）
//    P2-10 新增 RoadProfile 话题 + rviz Marker
//    P2-11 CMake 显式依赖 PCL
//    P2-12 不再依赖 pcl_ros/transforms.h，用 tf2 + pcl::transformPointCloud
//    P3-1  权限提示改为 dialout 组
//    P3-2  serial_fd_ 初始化为 -1
//    P3-3  周期性诊断日志
// ============================================================================

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/pass_through.h>
#include <pcl/common/transforms.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <Eigen/Geometry>

#include <suspension_preview_v2/msg/road_profile.hpp>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace {

// ---- 串口帧协议（与 v1 不兼容，已升级；见 docs/review/02-v2-upgrade-notes.md）----
constexpr uint8_t kFrameHead0 = 0xAA;
constexpr uint8_t kFrameHead1 = 0x55;
constexpr uint8_t kLaneLeft = 0x01;
constexpr uint8_t kLaneRight = 0x02;
constexpr int16_t kInvalidMark = 0x7FFF;  // int16 最大值，表示该点无效

// 有效掩码语义
constexpr uint8_t kInvalid = 0;   // 无效
constexpr uint8_t kMeasured = 1;  // 实测有效
constexpr uint8_t kInterpolated = 2;  // 插值得到（可用，但精度略低）

// CRC16-CCITT (poly 0x1021, init 0xFFFF)
uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

speed_t to_speed(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
#ifdef B921600
        case 921600: return B921600;
#endif
        default:     return B115200;
    }
}

// 循环写满 + 等待发送完成；返回 false 表示确实写失败
bool write_all(int fd, const uint8_t* data, size_t len) {
    size_t off = 0;
    while (off < len) {
        const ssize_t w = ::write(fd, data + off, len - off);
        if (w < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            return false;
        }
        off += static_cast<size_t>(w);
    }
    ::tcdrain(fd);
    return true;
}

}  // namespace

class RoadPreviewNode : public rclcpp::Node {
public:
    explicit RoadPreviewNode(const rclcpp::NodeOptions& options)
        : rclcpp::Node("road_preview_node", options) {
        declare_parameters();

        // P1-1: 传感器数据使用 SensorDataQoS（BEST_EFFORT），避免与雷达发布方不兼容
        const auto qos = rclcpp::SensorDataQoS();
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, qos,
            std::bind(&RoadPreviewNode::cloud_callback, this, std::placeholders::_1));

        pub_ = create_publisher<suspension_preview_v2::msg::RoadProfile>(publish_topic_, 10);
        marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
            publish_topic_ + "_marker", 10);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        setup_serial();          // P3-2: 内部失败时 serial_fd_ 保持 -1
        start_tx_thread();       // P2-9: 独立发送线程

        const int cells = static_cast<int>(std::lround(preview_dist_ / resolution_));
        RCLCPP_INFO(get_logger(),
                    "悬架预瞄节点已启动 | 点云: %s -> %s | 剖面: %d 格 x %.3f m | 串口: %s",
                    cloud_topic_.c_str(), target_frame_.c_str(), cells, resolution_,
                    enable_serial_ ? serial_port_.c_str() : "(已禁用)");

        diag_timer_ = create_wall_timer(
            std::chrono::duration<double>(diag_period_s_),
            std::bind(&RoadPreviewNode::log_diagnostics, this));
    }

    ~RoadPreviewNode() override {
        stop_tx_thread();
        if (serial_fd_ >= 0) {
            ::close(serial_fd_);
            serial_fd_ = -1;
        }
    }

private:
    // ------------------------------------------------------------------ 参数
    void declare_parameters() {
        cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/unilidar/cloud");
        target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
        publish_topic_ = declare_parameter<std::string>(
            "publish_topic", "/suspension/road_profile");

        serial_port_ = declare_parameter<std::string>("serial_port", "/dev/ttyS0");
        baud_rate_ = declare_parameter<int>("baud_rate", 115200);
        enable_serial_ = declare_parameter<bool>("enable_serial", true);

        wheel_dist_ = declare_parameter<double>("wheel_dist", 0.6);
        lane_half_width_ = declare_parameter<double>("lane_half_width", 0.1);
        preview_dist_ = declare_parameter<double>("preview_dist", 5.0);
        resolution_ = declare_parameter<double>("resolution", 0.05);

        ground_z_min_ = declare_parameter<double>("ground_z_min", -0.40);
        ground_z_max_ = declare_parameter<double>("ground_z_max", 0.10);
        mount_height_ = declare_parameter<double>("mount_height", 0.20);

        min_points_per_cell_ = declare_parameter<int>("min_points_per_cell", 3);
        max_cell_spread_ = declare_parameter<double>("max_cell_spread", 0.15);
        use_median_ = declare_parameter<bool>("use_median", true);
        interpolate_gaps_ = declare_parameter<bool>("interpolate_gaps", false);

        tf_timeout_ms_ = declare_parameter<int>("tf_timeout_ms", 30);
        publish_marker_ = declare_parameter<bool>("publish_marker", true);
        diag_period_s_ = declare_parameter<double>("diag_period_s", 5.0);

        // P2-1: 用安装高度做一次合理性校验并提示
        if (resolution_ <= 0.0 || preview_dist_ <= 0.0) {
            RCLCPP_WARN(get_logger(), "preview_dist / resolution 非法，回退到默认值 5.0 m / 0.05 m");
            preview_dist_ = 5.0;
            resolution_ = 0.05;
        }
        if (std::fabs(static_cast<double>(mount_height_)) > 3.0) {
            RCLCPP_WARN(get_logger(), "mount_height=%.2f m 明显异常，请检查参数", mount_height_);
        }
        if (ground_z_min_ >= ground_z_max_) {
            RCLCPP_WARN(get_logger(), "ground_z_min >= ground_z_max，地面窗口无效，已交换");
            std::swap(ground_z_min_, ground_z_max_);
        }
    }

    // -------------------------------------------------------------- 串口（P1-2/P1-5/P3-1）
    void setup_serial() {
        if (!enable_serial_) {
            RCLCPP_INFO(get_logger(), "enable_serial=false，仅发布 ROS 话题，不打开串口");
            return;
        }
        // 阻塞模式打开；不用 O_NDELAY（v1 的问题 P1-5）
        serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY);
        if (serial_fd_ < 0) {
            RCLCPP_ERROR(get_logger(),
                         "串口打开失败 (%s): %s。请确认设备存在，并将当前用户加入 dialout 组: "
                         "sudo usermod -aG dialout $USER（重新登录后生效）",
                         serial_port_.c_str(), std::strerror(errno));
            serial_fd_ = -1;
            return;
        }

        struct termios tty {};
        if (::tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(get_logger(), "tcgetattr 失败: %s", std::strerror(errno));
            ::close(serial_fd_);
            serial_fd_ = -1;
            return;
        }

        ::cfmakeraw(&tty);                     // P1-2: 关键！关掉所有行规程转换
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CRTSCTS;               // 关闭硬件流控
        tty.c_cflag &= ~(CSTOPB | PARENB);     // 8N1
        tty.c_cflag |= CS8;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 5;                   // 0.5 s 读超时（本节点只写不读）

        const speed_t spd = to_speed(baud_rate_);
        ::cfsetispeed(&tty, spd);
        ::cfsetospeed(&tty, spd);

        if (::tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(get_logger(), "tcsetattr 失败: %s", std::strerror(errno));
            ::close(serial_fd_);
            serial_fd_ = -1;
            return;
        }
        ::tcflush(serial_fd_, TCIOFLUSH);
        RCLCPP_INFO(get_logger(), "串口已就绪: %s @ %d bps (raw 模式)", serial_port_.c_str(), baud_rate_);
    }

    // --------------------------------------------------------------- 发送线程
    void start_tx_thread() {
        tx_thread_ = std::thread(&RoadPreviewNode::tx_loop, this);
    }

    void stop_tx_thread() {
        {
            std::lock_guard<std::mutex> lk(tx_mutex_);
            tx_stop_ = true;
        }
        tx_cv_.notify_all();
        if (tx_thread_.joinable()) tx_thread_.join();
    }

    void tx_loop() {
        while (true) {
            std::vector<uint8_t> frame;
            {
                std::unique_lock<std::mutex> lk(tx_mutex_);
                tx_cv_.wait(lk, [this] { return tx_stop_ || mailbox_.has_value(); });
                if (!mailbox_.has_value()) {
                    if (tx_stop_) return;
                    continue;
                }
                frame = std::move(*mailbox_);
                mailbox_.reset();
            }
            if (serial_fd_ < 0) continue;
            if (write_all(serial_fd_, frame.data(), frame.size())) {
                tx_frames_++;
            } else {
                tx_errors_++;
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "串口写入失败: %s", std::strerror(errno));
            }
        }
    }

    // 投递到"最新帧邮箱"：来不及发送的旧帧被丢弃（P2-9），并计数
    void enqueue_frame(std::vector<uint8_t>&& frame) {
        {
            std::lock_guard<std::mutex> lk(tx_mutex_);
            if (mailbox_.has_value()) tx_dropped_++;
            mailbox_ = std::move(frame);
        }
        tx_cv_.notify_one();
    }

    // ------------------------------------------------------------ 点云主回调
    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        frames_++;

        auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

        // P1-3 / P1-4: TF 必须成功，且使用该帧采集时刻；失败则丢弃该帧（fail-safe）
        if (!transform_to_target(msg, cloud)) {
            tf_fail_++;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                 "TF 不可用，已丢弃该帧（累计 %lu 帧）",
                                 static_cast<unsigned long>(tf_fail_));
            return;
        }

        const LaneResult left = process_lane(cloud, -wheel_dist_ / 2.0f);
        const LaneResult right = process_lane(cloud, +wheel_dist_ / 2.0f);

        const auto header = make_header(msg->header.stamp);
        publish_profile(header, left, right);
        if (publish_marker_) publish_markers(header, left, right);

        if (enable_serial_ && serial_fd_ >= 0) {
            auto frame = encode_frame(kLaneLeft, tx_seq_, left);
            frame = concat(std::move(frame), encode_frame(kLaneRight, tx_seq_, right));
            tx_seq_++;
            enqueue_frame(std::move(frame));
        }
    }

    // ---------------------------------------------------------- TF 变换（P1-3/4/12）
    bool transform_to_target(const sensor_msgs::msg::PointCloud2::SharedPtr& msg,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr& out) {
        if (msg->header.frame_id.empty()) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "点云缺少 frame_id");
            return false;
        }

        // 时间戳为 0 表示驱动未填，退化为"最新可用"；否则严格用采集时刻
        rclcpp::Time stamp;
        if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
            stamp = rclcpp::Time(0, 0, get_clock()->get_clock_type());
        } else {
            stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
        }

        geometry_msgs::msg::TransformStamped tf_msg;
        try {
            tf_msg = tf_buffer_->lookupTransform(
                target_frame_, msg->header.frame_id, stamp,
                rclcpp::Duration::from_nanoseconds(
                    static_cast<int64_t>(tf_timeout_ms_) * 1000000LL));
        } catch (const std::exception& ex) {
            RCLCPP_DEBUG(get_logger(), "lookupTransform 失败: %s", ex.what());
            return false;
        }

        auto raw = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *raw);

        // 由 tf2 的变换自行构造 4x4（避免 ROS1 风格头文件，P2-12）
        Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
        const auto& q = tf_msg.transform.rotation;
        const auto& t = tf_msg.transform.translation;
        Eigen::Quaternionf quat(static_cast<float>(q.w), static_cast<float>(q.x),
                                static_cast<float>(q.y), static_cast<float>(q.z));
        quat.normalize();
        m.block<3, 3>(0, 0) = quat.toRotationMatrix();
        m(0, 3) = static_cast<float>(t.x);
        m(1, 3) = static_cast<float>(t.y);
        m(2, 3) = static_cast<float>(t.z);

        out = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::transformPointCloud(*raw, *out, m);
        return true;
    }

    // -------------------------------------------------------- 轮迹高程提取
    struct LaneResult {
        std::vector<float> height;   // 无效点为 NaN
        std::vector<uint8_t> valid;  // 0/1/2
    };

    LaneResult process_lane(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float y_center) {
        const int cells = static_cast<int>(std::lround(preview_dist_ / resolution_));
        LaneResult result;
        result.height.assign(static_cast<size_t>(cells),
                             std::numeric_limits<float>::quiet_NaN());
        result.valid.assign(static_cast<size_t>(cells), kInvalid);

        // 1) ROI：先按 y 取条带，再按 x 取前方距离（P2-4: 格数由参数推导）
        auto roi = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(static_cast<float>(y_center - lane_half_width_),
                             static_cast<float>(y_center + lane_half_width_));
        pass.filter(*roi);

        pass.setInputCloud(roi);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.0f, static_cast<float>(preview_dist_));
        pass.filter(*roi);

        // 2) 分箱（P2-1: 地面窗口参数化）
        std::vector<std::vector<float>> bins(static_cast<size_t>(cells));
        for (const auto& pt : roi->points) {
            if (!std::isfinite(pt.z)) continue;
            if (pt.z < ground_z_min_ || pt.z > ground_z_max_) continue;
            const int idx = static_cast<int>(std::floor(pt.x / resolution_));
            if (idx < 0 || idx >= cells) continue;
            bins[static_cast<size_t>(idx)].push_back(pt.z);
        }

        // 3) 逐格鲁棒估计（P2-2/P2-3）
        int invalid_count = 0;
        for (int i = 0; i < cells; ++i) {
            const auto& b = bins[static_cast<size_t>(i)];
            if (static_cast<int>(b.size()) < min_points_per_cell_) {
                ++invalid_count;
                continue;
            }
            const auto mm = std::minmax_element(b.begin(), b.end());
            if ((*mm.second - *mm.first) > max_cell_spread_) {
                // 格内高低差过大：疑似障碍物 / 跨越路缘，判为无效
                ++invalid_count;
                continue;
            }
            float est = 0.0f;
            if (use_median_) {
                auto tmp = b;  // 避免破坏原始顺序
                const size_t mid = tmp.size() / 2;
                std::nth_element(tmp.begin(), tmp.begin() + static_cast<long>(mid), tmp.end());
                est = tmp[mid];
            } else {
                est = std::accumulate(b.begin(), b.end(), 0.0f) /
                      static_cast<float>(b.size());
            }
            result.height[static_cast<size_t>(i)] = est;
            result.valid[static_cast<size_t>(i)] = kMeasured;
        }

        invalid_count_ += invalid_count;

        // 4) 可选线性插值补洞
        if (interpolate_gaps_) interpolate(result);
        return result;
    }

    void interpolate(LaneResult& r) {
        const int n = static_cast<int>(r.height.size());
        int i = 0;
        while (i < n) {
            if (r.valid[static_cast<size_t>(i)] != kInvalid) {
                ++i;
                continue;
            }
            int j = i;
            while (j < n && r.valid[static_cast<size_t>(j)] == kInvalid) ++j;
            const bool has_left = (i > 0) && (r.valid[static_cast<size_t>(i - 1)] != kInvalid);
            const bool has_right = (j < n) && (r.valid[static_cast<size_t>(j)] != kInvalid);
            if (has_left && has_right) {
                const float h0 = r.height[static_cast<size_t>(i - 1)];
                const float h1 = r.height[static_cast<size_t>(j)];
                const float span = static_cast<float>(j - (i - 1));
                for (int k = i; k < j; ++k) {
                    const float t = static_cast<float>(k - (i - 1)) / span;
                    r.height[static_cast<size_t>(k)] = h0 + (h1 - h0) * t;
                    r.valid[static_cast<size_t>(k)] = kInterpolated;
                }
            }
            i = j;
        }
    }

    // ------------------------------------------------------------- 输出（ROS）
    std_msgs::msg::Header make_header(const builtin_interfaces::msg::Time& stamp) {
        std_msgs::msg::Header h;
        h.stamp = stamp;
        h.frame_id = target_frame_;
        return h;
    }

    void publish_profile(const std_msgs::msg::Header& header,
                         const LaneResult& left, const LaneResult& right) {
        suspension_preview_v2::msg::RoadProfile msg;
        msg.header = header;
        msg.spacing = static_cast<float>(resolution_);
        msg.wheel_dist = static_cast<float>(wheel_dist_);
        msg.preview_dist = static_cast<float>(preview_dist_);
        msg.left_height = left.height;
        msg.right_height = right.height;
        msg.left_valid = left.valid;
        msg.right_valid = right.valid;

        const size_t n = left.valid.size() + right.valid.size();
        size_t ok = 0;
        for (auto v : left.valid) if (v != kInvalid) ++ok;
        for (auto v : right.valid) if (v != kInvalid) ++ok;
        msg.quality = static_cast<uint8_t>(n > 0 ? (100 * ok) / n : 0);

        pub_->publish(msg);
    }

    void publish_markers(const std_msgs::msg::Header& header,
                         const LaneResult& left, const LaneResult& right) {
        publish_one_marker(header, left, -wheel_dist_ / 2.0, 0, "left");
        publish_one_marker(header, right, +wheel_dist_ / 2.0, 1, "right");
    }

    void publish_one_marker(const std_msgs::msg::Header& header, const LaneResult& lane,
                            double y_center, int id, const char* ns) {
        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.ns = ns;
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.01;
        marker.color.r = (id == 0) ? 1.0f : 0.2f;
        marker.color.g = (id == 0) ? 0.4f : 0.8f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0f;

        for (size_t i = 0; i < lane.height.size(); ++i) {
            if (lane.valid[i] == kInvalid || !std::isfinite(lane.height[i])) continue;
            geometry_msgs::msg::Point p;
            p.x = static_cast<double>(i) * resolution_;
            p.y = y_center;
            p.z = lane.height[i];
            marker.points.push_back(p);
        }
        marker_pub_->publish(marker);
    }

    // ----------------------------------------------------------- 串口编码
    std::vector<uint8_t> encode_frame(uint8_t id, uint8_t seq, const LaneResult& lane) {
        std::vector<uint8_t> f;
        f.reserve(8 + lane.height.size() * 2);
        f.push_back(kFrameHead0);
        f.push_back(kFrameHead1);
        f.push_back(id);
        f.push_back(seq);
        const uint16_t n = static_cast<uint16_t>(lane.height.size());
        f.push_back(static_cast<uint8_t>(n & 0xFF));
        f.push_back(static_cast<uint8_t>((n >> 8) & 0xFF));

        for (size_t i = 0; i < lane.height.size(); ++i) {
            int16_t mm = kInvalidMark;
            if (lane.valid[i] != kInvalid && std::isfinite(lane.height[i])) {
                // P2-6: 米 -> 毫米 int16；P2-8: 显式小端编码
                const float v = std::max(-32767.0f, std::min(32767.0f,
                                                             lane.height[i] * 1000.0f));
                mm = static_cast<int16_t>(std::lround(v));
            }
            const uint16_t u = static_cast<uint16_t>(mm);
            f.push_back(static_cast<uint8_t>(u & 0x00FF));
            f.push_back(static_cast<uint8_t>((u >> 8) & 0x00FF));
        }

        const uint16_t crc = crc16_ccitt(f.data(), f.size());  // P2-7
        f.push_back(static_cast<uint8_t>(crc & 0x00FF));
        f.push_back(static_cast<uint8_t>((crc >> 8) & 0x00FF));
        return f;
    }

    static std::vector<uint8_t> concat(std::vector<uint8_t>&& a, std::vector<uint8_t>&& b) {
        a.insert(a.end(), b.begin(), b.end());
        return a;
    }

    // --------------------------------------------------------------- 诊断
    void log_diagnostics() {
        RCLCPP_INFO(get_logger(),
                    "帧:%lu TF失败:%lu 无效格:%lu 质量参考 | TX:%lu 丢帧:%lu 写错:%lu",
                    static_cast<unsigned long>(frames_), static_cast<unsigned long>(tf_fail_),
                    static_cast<unsigned long>(invalid_count_),
                    static_cast<unsigned long>(tx_frames_),
                    static_cast<unsigned long>(tx_dropped_),
                    static_cast<unsigned long>(tx_errors_));
    }

    // --------------------------------------------------------------- 成员
    // 参数
    std::string cloud_topic_;
    std::string target_frame_;
    std::string publish_topic_;
    std::string serial_port_;
    int baud_rate_{115200};
    bool enable_serial_{true};
    double wheel_dist_{0.6};
    double lane_half_width_{0.1};
    double preview_dist_{5.0};
    double resolution_{0.05};
    double ground_z_min_{-0.40};
    double ground_z_max_{0.10};
    double mount_height_{0.20};
    int min_points_per_cell_{3};
    double max_cell_spread_{0.15};
    bool use_median_{true};
    bool interpolate_gaps_{false};
    int tf_timeout_ms_{30};
    bool publish_marker_{true};
    double diag_period_s_{5.0};

    // ROS 接口
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<suspension_preview_v2::msg::RoadProfile>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr diag_timer_;

    // 串口
    int serial_fd_{-1};  // P3-2
    std::thread tx_thread_;
    std::mutex tx_mutex_;
    std::condition_variable tx_cv_;
    std::optional<std::vector<uint8_t>> mailbox_;
    bool tx_stop_{false};
    uint8_t tx_seq_{0};

    // 统计
    size_t frames_{0};
    size_t tf_fail_{0};
    size_t invalid_count_{0};
    size_t tx_frames_{0};
    size_t tx_dropped_{0};
    size_t tx_errors_{0};
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::spin(std::make_shared<RoadPreviewNode>(options));
    rclcpp::shutdown();
    return 0;
}
