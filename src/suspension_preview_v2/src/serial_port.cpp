#include "suspension_preview_v2/serial_port.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace suspension_preview_v2 {
namespace {

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

}  // namespace

SerialPort::SerialPort(int write_timeout_ms)
    : write_timeout_ms_(write_timeout_ms > 0 ? write_timeout_ms : 100) {}

SerialPort::~SerialPort() { close(); }

bool SerialPort::is_baud_supported(int baud_rate) {
    switch (baud_rate) {
        case 9600:
        case 19200:
        case 38400:
        case 57600:
        case 115200:
#ifdef B230400
        case 230400:
#endif
#ifdef B460800
        case 460800:
#endif
#ifdef B921600
        case 921600:
#endif
            return true;
        default:
            return false;
    }
}

bool SerialPort::open(const std::string& device, int baud_rate) {
    close();
    device_ = device;
    last_error_.clear();

    // 非阻塞打开 + poll 等待可写：保留"写满"语义，但每次写都有界。
    // 旧实现用阻塞 fd + 无超时 tcdrain，线缆异常或对端不读时 worker 会永久卡住。
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        last_error_ = std::string("open 失败: ") + std::strerror(errno) +
                      "（请确认设备存在，并将用户加入 dialout 组）";
        fd_ = -1;
        return false;
    }

    struct termios tty {};
    if (::tcgetattr(fd_, &tty) != 0) {
        last_error_ = std::string("tcgetattr 失败: ") + std::strerror(errno);
        close();
        return false;
    }

    ::cfmakeraw(&tty);  // 关键：关闭所有行规程转换，二进制协议才不会被改写
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;            // 关硬件流控
    tty.c_cflag &= ~(CSTOPB | PARENB);  // 8N1
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;

    const speed_t spd = to_speed(baud_rate);
    ::cfsetispeed(&tty, spd);
    ::cfsetospeed(&tty, spd);

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
        last_error_ = std::string("tcsetattr 失败: ") + std::strerror(errno);
        close();
        return false;
    }
    ::tcflush(fd_, TCIOFLUSH);
    return true;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::write_all(const uint8_t* data, std::size_t len) {
    if (fd_ < 0) {
        last_error_ = "串口未打开";
        return false;
    }
    if (data == nullptr || len == 0) return true;

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(write_timeout_ms_);
    std::size_t off = 0;
    while (off < len) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            last_error_ = "写超时（" + std::to_string(write_timeout_ms_) +
                          " ms），已写出 " + std::to_string(off) + "/" +
                          std::to_string(len) + " 字节";
            return false;
        }
        const int remaining_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());

        struct pollfd pfd {};
        pfd.fd = fd_;
        pfd.events = POLLOUT;
        const int pr = ::poll(&pfd, 1, remaining_ms);
        if (pr < 0) {
            if (errno == EINTR) continue;
            last_error_ = std::string("poll 失败: ") + std::strerror(errno);
            return false;
        }
        if (pr == 0) {
            last_error_ = "写超时（" + std::to_string(write_timeout_ms_) +
                          " ms），已写出 " + std::to_string(off) + "/" +
                          std::to_string(len) + " 字节";
            return false;
        }

        const ssize_t w = ::write(fd_, data + off, len - off);
        if (w < 0) {
            if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) continue;
            last_error_ = std::string("write 失败: ") + std::strerror(errno);
            return false;
        }
        if (w == 0) continue;  // 理论不会发生；交给 deadline 兜底
        off += static_cast<std::size_t>(w);
    }
    // 刻意不调用 tcdrain()：它在异常链路上没有超时，会把"有界写"重新变成挂死。
    // 内核已接收全部字节；需要确认对端处理时应由协议层 ACK 负责。
    return true;
}

}  // namespace suspension_preview_v2
