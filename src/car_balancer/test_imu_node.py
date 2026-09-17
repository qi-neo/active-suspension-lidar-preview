import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class LidarImuTester(Node):
    def __init__(self):
        super().__init__('lidar_imu_tester')
        
        # 默认的IMU话题名称为 "unilidar/imu" [cite: 164, 223]
        self.imu_topic = 'unilidar/imu'
        
        self.subscription = self.create_subscription(
            Imu,
            self.imu_topic,
            self.imu_callback,
            10
        )
        self.subscription  # 防止被垃圾回收
        
        self.msg_count = 0
        self.get_logger().info(f'IMU测试节点已启动！正在监听话题: {self.imu_topic}')

    def imu_callback(self, msg):
        self.msg_count += 1               
        
        # 为了防止打印过快，每接收到50条数据打印一次日志
        if self.msg_count % 50 == 0:
            # 获取四元数向量
            q = msg.orientation
            # 获取角速度
            w = msg.angular_velocity
            # 获取线加速度
            a = msg.linear_acceleration
            
            self.get_logger().info(
                f'\n成功接收IMU数据 (第{self.msg_count}条):\n'
                f'  四元数 [x, y, z, w]: [{q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f}]\n'
                f'  角速度 [x, y, z]:    [{w.x:.4f}, {w.y:.4f}, {w.z:.4f}]\n'
                f'  线加速度 [x, y, z]:  [{a.x:.4f}, {a.y:.4f}, {a.z:.4f}]'
            )

def main(args=None):
    rclpy.init(args=args)
    tester_node = LidarImuTester()
    
    try:
        rclpy.spin(tester_node)
    except KeyboardInterrupt:
        tester_node.get_logger().info('测试被用户中断。')
    finally:
        tester_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
