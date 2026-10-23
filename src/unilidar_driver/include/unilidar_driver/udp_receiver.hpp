// UDP 收包封装。只用到 POSIX socket 接口，方便在没有雷达的机器上替换成
// 回放文件或自检数据，也让单元测试不必真的开网络。

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace unilidar_driver {

// SO_RCVBUF 的默认请求值：8 MiB。
//
// 雷达以约 180 Hz 发点云包、每包约 1 KB（约 180 KB/s），Linux 内核默认的
// rmem 只有约 200 KB，轮询周期稍有抖动（调度延迟、系统忙、下游阻塞）就会
// 在 recvfrom 之前溢出丢包。8 MiB 约等于几十毫秒的余量，代价只是内存，
// 对嵌入式 Linux 也能接受。
constexpr int kDefaultRecvBufBytes = 8 * 1024 * 1024;

class UdpReceiver {
public:
  UdpReceiver() = default;
  ~UdpReceiver();

  UdpReceiver(const UdpReceiver&) = delete;
  UdpReceiver& operator=(const UdpReceiver&) = delete;

  // local_ip 为空字符串时绑定所有网卡，便于快速确认雷达是否真的在发包。
  // recv_buf_bytes 为请求的 SO_RCVBUF 字节数，0 表示不设置、沿用系统默认；
  // 内核会按 net.core.rmem_max 裁剪并把请求值翻倍，所以请求值不等于生效值，
  // 实际生效值用 recvBufBytes() 读取。
  bool open(unsigned short local_port, const std::string& local_ip, int recv_timeout_ms,
            int recv_buf_bytes, std::string& err);
  void close();
  bool isOpen() const { return fd_ >= 0; }

  // setsockopt 之后用 getsockopt 读回的实际生效值（Linux 通常为请求值的
  // 2 倍，或被 rmem_max 裁剪）；未打开或读取失败时为 0。
  int recvBufBytes() const { return recv_buf_bytes_; }

  // 返回收到的字节数；0 表示超时，-1 表示出错，错误信息写进 err。
  // src_ip 非空时回填发送方地址，用于只接受指定雷达的数据。
  int receive(std::uint8_t* buf, std::size_t cap, std::string& err, std::string* src_ip = nullptr);

private:
  int fd_{-1};
  int recv_buf_bytes_{0};
};

}  // namespace unilidar_driver
