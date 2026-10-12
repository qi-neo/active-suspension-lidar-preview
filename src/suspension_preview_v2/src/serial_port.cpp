#include "suspension_preview_v2/serial_port.hpp"

#include <cerrno>
#include <chrono>
#include <cstring>
#include <thread>

#include <fcntl.h>
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

    // 阻塞模式打开（不用 O_NDELAY，避免部分写静默丢失）
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
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
    std::size_t off = 0;
    while (off < len) {
        const ssize_t w = ::write(fd_, data + off, len - off);
        if (w < 0) {
            if (errno == EINTR) continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
                continue;
            }
            last_error_ = std::string("write 失败: ") + std::strerror(errno);
            return false;
        }
        off += static_cast<std::size_t>(w);
    }
    ::tcdrain(fd_);
    return true;
}

}  // namespace suspension_preview_v2
