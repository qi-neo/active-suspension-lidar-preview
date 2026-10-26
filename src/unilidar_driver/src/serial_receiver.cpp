#include "unilidar_driver/serial_receiver.hpp"

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <cstring>

namespace unilidar_driver {
namespace {

// Linux termios 的扩展速率常量表。POSIX 只保证到 38400；带 #ifdef 逐项守卫，
// 缺项的平台上直接落到「不支持」分支，由 serialBaudSupported() 提前问出来。
struct BaudEntry {
  int baud;
  speed_t speed;
};

const BaudEntry kBaudTable[] = {
    {9600, B9600},
    {19200, B19200},
    {38400, B38400},
#ifdef B57600
    {57600, B57600},
#endif
    {115200, B115200},
#ifdef B230400
    {230400, B230400},
#endif
#ifdef B460800
    {460800, B460800},
#endif
#ifdef B500000
    {500000, B500000},
#endif
#ifdef B576000
    {576000, B576000},
#endif
#ifdef B921600
    {921600, B921600},
#endif
#ifdef B1000000
    {1000000, B1000000},
#endif
#ifdef B1152000
    {1152000, B1152000},
#endif
#ifdef B1500000
    {1500000, B1500000},
#endif
#ifdef B2000000
    {2000000, B2000000},
#endif
#ifdef B2500000
    {2500000, B2500000},
#endif
#ifdef B3000000
    {3000000, B3000000},
#endif
#ifdef B3500000
    {3500000, B3500000},
#endif
#ifdef B4000000
    {4000000, B4000000},
#endif
};

// USB 转串口芯片默认"攒一批再上报"，ftdi/ch341 的缓冲延迟都在毫秒量级
// （CH340 实测约 16 ms）。L1 的预瞄数据对帧龄敏感，这 16 ms 是纯粹白送的。
// ch341 驱动自较新内核起提供 low_latency 属性，写 1 即改为逐包上报。
//
// 不能用 TIOCSSERIAL + ASYNC_LOW_LATENCY：那是 8250 平台串口的标志，
// usb-serial 驱动不读它，设了不生效。必须走 sysfs。
void enableLowLatency(const std::string& device, bool* enabled, std::string* note) {
  *enabled = false;
  note->clear();
#if defined(__linux__)
  // /dev/ttyUSB0 -> ttyUSB0，拼出 usb-serial 设备的 sysfs 目录
  const std::size_t slash = device.find_last_of('/');
  const std::string name = (slash == std::string::npos) ? device : device.substr(slash + 1);
  if (name.empty()) return;

  const std::string path = "/sys/bus/usb-serial/devices/" + name + "/low_latency";
  const int fd = ::open(path.c_str(), O_WRONLY | O_CLOEXEC);
  if (fd < 0) {
    // 属性不存在是常态：板载 UART、老内核、或经 /dev/serial/by-id 传入的
    // 长设备名都会走到这里。不是错误，只是少了这项优化。
    return;
  }
  const char one = '1';
  const ssize_t n = ::write(fd, &one, 1);
  ::close(fd);
  if (n == 1) {
    *enabled = true;
    *note = "low_latency=1（" + path + "）";
  } else {
    *note = "low_latency 写入失败：" + path;
  }
#else
  (void)device;
#endif
}

}  // namespace

bool serialBaudSupported(int baud) {
  for (const BaudEntry& e : kBaudTable) {
    if (e.baud == baud) return true;
  }
  return false;
}

SerialReceiver::~SerialReceiver() { close(); }

bool SerialReceiver::open(const std::string& device, int baud, int read_timeout_ms,
                          std::string& err) {
  (void)read_timeout_ms;  // 读为非阻塞，超时由调用方轮询节奏决定
  close();

  if (device.empty()) {
    err = "串口设备名为空";
    return false;
  }
  if (!serialBaudSupported(baud)) {
    err = "不支持的波特率 " + std::to_string(baud) +
          "（本平台 termios 无对应常量；L1 官方 SDK 默认 2000000）";
    return false;
  }

  // O_NONBLOCK：open 本身不因对端（雷达）未就绪而挂住；随后的 read 也立即
  // 返回，收多少算多少，与 UdpReceiver 的「0 = 当前无数据」契约一致。
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    err = "打开 " + device + " 失败: " + std::strerror(errno) +
          (errno == EACCES ? "（把当前用户加入 dialout 组后重新登录）" : "");
    fd_ = -1;
    return false;
  }

  // 打开后先问一句是不是终端设备：把普通文件/目录当串口打开会在 tcgetattr
  // 才失败，错误信息离真正原因远。
  if (::isatty(fd_) != 1) {
    err = device + " 不是终端设备（检查是否是 USB 转串口模块枚举出的口）";
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  termios tio{};
  if (::tcgetattr(fd_, &tio) != 0) {
    err = std::string("读取 " + device + " 串口属性失败: ") + std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // cfmakeraw：关回显、行规程、信号字符、流量控制外的所有加工——
  // 串口是二进制字节流，任何一行处理都会破坏帧定界。
  ::cfmakeraw(&tio);
  tio.c_cflag |= CLOCAL;  // 忽略调制解调器控制线，不因 DTR/DSR 未连接而阻塞
  tio.c_cflag &= ~CRTSCTS;  // 无硬件流控（CH340 默认不接）

  speed_t speed = B9600;
  for (const BaudEntry& e : kBaudTable) {
    if (e.baud == baud) {
      speed = e.speed;
      break;
    }
  }
  ::cfsetispeed(&tio, speed);
  ::cfsetospeed(&tio, speed);

  // VMIN=0, VTIME=0：read 立即返回（配合 O_NONBLOCK），不等字节攒满。
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;

  if (::tcsetattr(fd_, TCSANOW, &tio) != 0) {
    err = std::string("设置 " + device + " 串口参数失败: ") + std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  // 丢掉打开前积在驱动缓冲里的旧字节：重启驱动后最先读到的往往是上轮的
  // 残帧，直接进 parser 只会制造一批必然 CRC 失败的坏帧。
  ::tcflush(fd_, TCIFLUSH);

  device_ = device;
  baud_ = baud;
  // 放在最后：设备已确认可用、termios 已生效，这时改上报策略才有意义。
  // 失败不影响收发，只是保留了默认的 16 ms 攒批延迟。
  enableLowLatency(device, &low_latency_enabled_, &low_latency_note_);
  return true;
}

void SerialReceiver::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
  device_.clear();
  baud_ = 0;
  low_latency_enabled_ = false;
  low_latency_note_.clear();
}

int SerialReceiver::readSome(std::uint8_t* buf, std::size_t cap, std::string& err) {
  if (fd_ < 0) {
    err = "串口未打开";
    return -1;
  }
  const ssize_t n = ::read(fd_, buf, cap);
  if (n >= 0) return static_cast<int>(n);
  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;  // 当前无数据
  err = std::string("读 ") + device_ + " 失败: " + std::strerror(errno);
  return -1;
}

}  // namespace unilidar_driver
