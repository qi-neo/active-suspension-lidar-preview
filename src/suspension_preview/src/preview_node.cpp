#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/pass_through.h>
#include <pcl_ros/transforms.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

// --- 用户需修改的参数 ---
#define SERIAL_PORT "/dev/ttyS0"  // 请确认你的串口号
#define WHEEL_DIST 0.6f           // 左右轮间距 (米)
#define PREVIEW_DIST 5.0f         // 预瞄距离 (米)
#define MOUNT_HEIGHT 0.2f         // 雷达离地高度 (米) - 用于粗略过滤
// ----------------------

class SuspensionPreviewNode : public rclcpp::Node {
public:
    SuspensionPreviewNode() : Node("suspension_preview_node") {
        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/unilidar/cloud", 10, std::bind(&SuspensionPreviewNode::cloud_callback, this, std::placeholders::_1));
        
        init_serial();
        
        // TF 监听器
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        RCLCPP_INFO(this->get_logger(), "悬架预瞄系统启动: 等待雷达数据...");
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
 
   std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    int serial_fd_;

    void init_serial() {
        serial_fd_ = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ == -1) {
            RCLCPP_ERROR(this->get_logger(), "串口打开失败! 请检查权限: sudo chmod 777 %s", SERIAL_PORT);
        } else {
            struct termios options;
            tcgetattr(serial_fd_, &options);
            cfsetispeed(&options, B115200);
            cfsetospeed(&options, B115200);
            options.c_cflag |= (CLOCAL | CREAD | CS8);
            options.c_cflag &= ~(PARENB | CSTOPB | CSIZE);
            tcsetattr(serial_fd_, TCSANOW, &options);
            RCLCPP_INFO(this->get_logger(), "串口已连接: %s", SERIAL_PORT);
        }
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 1. 坐标转换：将雷达坐标系转换到 base_link (车身底盘中心)
        sensor_msgs::msg::PointCloud2 cloud_out;
        bool tf_success = false;
        try {
            // 尝试寻找变换关系
            if (tf_buffer_->canTransform("base_link", msg->header.frame_id, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1))) {
                pcl_ros::transformPointCloud("base_link", *msg, cloud_out, *tf_buffer_);
                tf_success = true;
            }
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "等待坐标变换(TF)...");
        }

        // 如果TF失败，直接用原始数据（仅供调试，实际必须要有TF）
        if (!tf_success) cloud_out = *msg;

        // 2. 转为 PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(cloud_out, *cloud);

        // 3. 提取两条轮胎轨迹
        auto left_data = process_lane(cloud, -WHEEL_DIST/2.0);
        auto right_data = process_lane(cloud, WHEEL_DIST/2.0);

        // 4. 发送 (限制频率，防止串口堵塞，这里每收到一帧发一次，雷达约10-20Hz)
        send_protocol(0x01, left_data);
        send_protocol(0x02, right_data);
    }

    std::vector<float> process_lane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float y_center) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        
        // ROI 裁剪 (只看轮胎前方)
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(y_center - 0.1, y_center + 0.1); // 左右宽20cm
        pass.filter(*cloud_filtered);
        
        pass.setInputCloud(cloud_filtered);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(0.0, PREVIEW_DIST); // 前方5米
        pass.filter(*cloud_filtered);

        // 地面高度提取 (简单网格法)
        // 100个点，对应5米，每个点代表5cm
        std::vector<float> profile(100, 0.0f);
        std::vector<int> counts(100, 0);

        for (const auto& pt : cloud_filtered->points) {
            // 简单滤除悬空噪点(假设地面在 z=0 附近)
            if (pt.z < -0.5 || pt.z > 0.5) continue;

            int idx = (int)(pt.x / 0.05); // 5cm分辨率
            if (idx >= 0 && idx < 100) {
                profile[idx] += pt.z;
                counts[idx]++;
            }
        }
        
        // 取平均
        for(int i=0; i<100; i++) {
            if(counts[i] > 0) profile[i] /= counts[i];
        }
        return profile;
    }

    void send_protocol(uint8_t id, const std::vector<float>& data) {
        if(serial_fd_ == -1) return;
        std::vector<uint8_t> buf;
        buf.push_back(0xAA); buf.push_back(0x55); // 头
        buf.push_back(id); // 0x01左, 0x02右
        buf.push_back((uint8_t)data.size()); // 长度 100
        
        for(float val : data) {
            uint8_t* p = (uint8_t*)&val;
            for(int k=0; k<4; k++) buf.push_back(p[k]);
        }
        
        uint8_t sum = 0;
        for(auto b : buf) sum += b;
        buf.push_back(sum); // 校验

        write(serial_fd_, buf.data(), buf.size());
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SuspensionPreviewNode>());
    rclcpp::shutdown();
    return 0;
}
