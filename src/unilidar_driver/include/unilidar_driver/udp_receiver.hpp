// UDP 收包封装。只用到 POSIX socket 接口，方便在没有雷达的机器上替换成
// 回放文件或自检数据，也让单元测试不必真的开网络。

#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace unilidar_driver {

class UdpReceiver {
public:
  UdpReceiver() = default;
  ~UdpReceiver();

  UdpReceiver(const UdpReceiver&) = delete;
  UdpReceiver& operator=(const UdpReceiver&) = delete;

  // local_ip 为空字符串时绑定所有网卡，便于快速确认雷达是否真的在发包
  bool open(unsigned short local_port, const std::string& local_ip, int recv_timeout_ms,
            std::string& err);
  void close();
  bool isOpen() const { return fd_ >= 0; }

  // 返回收到的字节数；0 表示超时，-1 表示出错，错误信息写进 err。
  // src_ip 非空时回填发送方地址，用于只接受指定雷达的数据。
  int receive(std::uint8_t* buf, std::size_t cap, std::string& err, std::string* src_ip = nullptr);

private:
  int fd_{-1};
};

}  // namespace unilidar_driver
