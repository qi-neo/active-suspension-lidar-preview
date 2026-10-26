// 串口接收封装（POSIX termios）。L1 的原生通讯接口就是 TTL UART，官方 SDK
// 默认经 USB 转串口模块（CH340 类）接入，设备名通常为 /dev/ttyUSB0，
// 波特率 2,000,000 bps。本类只封装「打开设备 + 按波特率读字节」，
// 不依赖 ROS；报文定界与解析在 packet_parser，与本类完全解耦——
// 串口是字节流，packet_parser 本就按无边界字节流设计（可在任意位置开始接收）。

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace unilidar_driver {

// 该波特率是否能配置。termios 的标准速率常量只到 38400，其余（57600 起）
// 是各平台扩展：Linux 有 B57600..B4000000，macOS 只到 B230400。
// 2 Mbps 这类非常规速率必须先问清楚再 open，否则错误只在真机上才暴露。
bool serialBaudSupported(int baud);

class SerialReceiver {
public:
  SerialReceiver() = default;
  ~SerialReceiver();

  SerialReceiver(const SerialReceiver&) = delete;
  SerialReceiver& operator=(const SerialReceiver&) = delete;

  // device 如 /dev/ttyUSB0；baud 为波特率（L1 官方 SDK 默认 2000000）；
  // read_timeout_ms 保留参数（读为非阻塞，超时由调用方轮询节奏决定），
  // 传 0 即可。打开失败时 err 带原因（设备不存在 / 无权限 / 波特率不支持）。
  bool open(const std::string& device, int baud, int read_timeout_ms, std::string& err);
  void close();
  bool isOpen() const { return fd_ >= 0; }

  // 底层文件描述符（未打开时为 -1）。供调用方自行 poll/select，或测试
  // 观察句柄状态；句柄的所有权仍归本类，调用方不得关闭它。
  int fd() const { return fd_; }

  const std::string& device() const { return device_; }
  int baud() const { return baud_; }

  // USB 转串口低延迟模式是否已开启。CH340 类芯片默认把收到的字节攒一批
  // 再上报，缓冲延迟约 16 ms；对预览系统这等于凭空多 16 ms 帧龄。
  // 开启后改为逐包上报，实测延迟降到 ~1 ms。
  // 非 Linux、非 usb-serial 设备、或内核无该属性时恒为 false——
  // 这只表示"少了一项优化"，不影响收发功能。
  bool lowLatencyEnabled() const { return low_latency_enabled_; }
  const std::string& lowLatencyNote() const { return low_latency_note_; }

  // 返回读到的字节数；0 表示当前无数据（非阻塞）；-1 表示出错，err 有原因。
  int readSome(std::uint8_t* buf, std::size_t cap, std::string& err);

private:
  int fd_{-1};
  std::string device_;
  int baud_{0};
  bool low_latency_enabled_{false};
  std::string low_latency_note_;
};

}  // namespace unilidar_driver
