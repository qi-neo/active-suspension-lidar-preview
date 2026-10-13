#include "unilidar_driver/udp_receiver.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <cstring>

namespace unilidar_driver {

UdpReceiver::~UdpReceiver() { close(); }

bool UdpReceiver::open(unsigned short local_port, const std::string& local_ip,
                       int recv_timeout_ms, std::string& err) {
  close();

  fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd_ < 0) {
    err = "socket 创建失败: " + std::string(std::strerror(errno));
    fd_ = -1;
    return false;
  }

  const int reuse = 1;
  ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(local_port);
  if (local_ip.empty()) {
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
  } else {
    if (::inet_pton(AF_INET, local_ip.c_str(), &addr.sin_addr) != 1) {
      err = "本机 IP 格式不合法: " + local_ip;
      ::close(fd_);
      fd_ = -1;
      return false;
    }
  }

  if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    err = "绑定 " + (local_ip.empty() ? std::string("0.0.0.0") : local_ip) + ":" +
          std::to_string(local_port) + " 失败: " + std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  if (recv_timeout_ms > 0) {
    timeval tv{};
    tv.tv_sec = recv_timeout_ms / 1000;
    tv.tv_usec = (recv_timeout_ms % 1000) * 1000;
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
  }

  return true;
}

void UdpReceiver::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

int UdpReceiver::receive(std::uint8_t* buf, std::size_t cap, std::string& err,
                         std::string* src_ip) {
  if (fd_ < 0) {
    err = "socket 未打开";
    return -1;
  }
  sockaddr_in from{};
  socklen_t from_len = sizeof(from);
  const ssize_t n = ::recvfrom(fd_, buf, cap, 0, reinterpret_cast<sockaddr*>(&from), &from_len);
  if (n < 0) {
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return 0;  // 超时
    err = std::string("recvfrom 失败: ") + std::strerror(errno);
    return -1;
  }
  if (src_ip != nullptr) {
    char host[INET_ADDRSTRLEN] = {0};
    if (::inet_ntop(AF_INET, &from.sin_addr, host, sizeof(host)) != nullptr) {
      *src_ip = host;
    }
  }
  return static_cast<int>(n);
}

}  // namespace unilidar_driver
