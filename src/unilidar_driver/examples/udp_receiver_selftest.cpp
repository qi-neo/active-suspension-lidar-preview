// UDP 收包的离线自检。不需要雷达、也不需要外部网络——
// 收发两端都建在本机回环上：接收端是被测的 UdpReceiver，发送端是一个
// 普通的 UDP socket。真机上 recvfrom 之前的每一类失败（IP 配错、
// 端口被占、socket 没开就收）在这里都能复现并断言。
//
// 覆盖的场景：
//   1. 打开契约：空 IP 绑所有网卡（INADDR_ANY）、显式 IP、
//      SO_RCVBUF 请求 0（不设置、沿用系统默认）、超时 0（不设置），
//      四种组合都能正常打开，recvBufBytes() 读回实际生效值；
//   2. 打开失败路径：IP 格式不合法（值不合法/段数不足）、
//      同端口二次 bind（EADDRINUSE）——真机上对应"雷达在跑、
//      又启动了一个驱动"；
//   3. 未打开就收：返回 -1 并说明原因，而不是崩或假装超时；
//   4. 收发契约：自发自收字节数一致、数据逐字节相同、src_ip
//      回显发送方地址；无数据 + SO_RCVTIMEO：返回 0 且 err 为空
//      （超时不是错误）；
//   5. close 契约：幂等；close 之后再收返回 -1 并点名"未打开"。
//
// 与 tests/udp_loopback_test.sh 的分工：那个脚本是集成测试，验证
// "probe + parser 在真实收发下能解出报文、摘要数字对得上"；本文件是
// 单元测试，专门覆盖 UdpReceiver 自身的每条分支（包括失败路径）——
// 集成测试只走 happy path，失败路径只有单元测试能构造。
//
// 端口默认 46311，可用环境变量 UDP_SELFTEST_PORT 覆盖（CI 与并行
// 运行时避免撞车）。
//
// 写注释时不要把命令行以反斜杠结尾：C++ 翻译阶段会先把「反斜杠 + 换行」
// 拼接掉，// 注释会被续到下一行，GCC 的 -Wcomment 会报 multi-line comment，
// 叠上 -Werror 直接编译失败（clang 不报，所以只在双编译器 CI 里暴露）。

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "selftest_fixture.hpp"
#include "unilidar_driver/udp_receiver.hpp"

using namespace unilidar_driver;
using namespace unilidar_selftest;

namespace {

// 回环发送端：把 payload 发到 127.0.0.1:port。返回是否发送成功。
bool sendLoopback(std::uint16_t port, const std::uint8_t* data, std::size_t len) {
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return false;
  sockaddr_in dst{};
  dst.sin_family = AF_INET;
  dst.sin_port = htons(port);
  if (::inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr) != 1) {
    ::close(fd);
    return false;
  }
  const ssize_t n =
      ::sendto(fd, data, len, 0, reinterpret_cast<const sockaddr*>(&dst), sizeof(dst));
  ::close(fd);
  return n == static_cast<ssize_t>(len);
}

std::uint16_t testPort() {
  const char* env = std::getenv("UDP_SELFTEST_PORT");
  if (env != nullptr && *env != '\0') {
    const int p = std::atoi(env);
    if (p > 0 && p < 65536) return static_cast<std::uint16_t>(p);
  }
  return 46311;
}

}  // namespace

// ---------------------------------------------------------------------------
// 1. 打开契约：四种参数组合都应打开成功
// ---------------------------------------------------------------------------
static void test_open_contract(SelftestLog& log) {
  std::printf("[1] 打开契约\n");
  const std::uint16_t port = testPort();
  std::string err;

  // 空 IP：绑所有网卡。探针缺省就是这个行为，必须保住。
  {
    UdpReceiver rx;
    log.check(rx.open(port, "", 200, kDefaultRecvBufBytes, err), "空 IP 应打开成功（INADDR_ANY）");
    log.check(rx.isOpen(), "打开后 isOpen() 应为 true");
    log.check(rx.recvBufBytes() > 0, "请求 8 MiB 后应读回实际生效值（getsockopt 路径）");
    std::printf("      空 IP: SO_RCVBUF 实际生效 %d 字节\n", rx.recvBufBytes());
  }

  // 显式 IP：回环地址，真机上对应"只听雷达那张网卡"。
  {
    UdpReceiver rx;
    log.check(rx.open(port, "127.0.0.1", 200, kDefaultRecvBufBytes, err),
              "显式回环 IP 应打开成功");
    log.check(rx.isOpen(), "显式 IP 打开后 isOpen() 应为 true");
  }

  // SO_RCVBUF 请求 0：不设置、沿用系统默认。setsockopt 被跳过，
  // 但 getsockopt 照样读回真实值——日志里不能显示 0。
  {
    UdpReceiver rx;
    log.check(rx.open(port, "", 200, 0, err), "SO_RCVBUF 请求 0 应打开成功（沿用系统默认）");
    log.check(rx.recvBufBytes() > 0, "请求 0 时仍应读回系统默认缓冲值");
    std::printf("      请求 0: SO_RCVBUF 实际生效 %d 字节（系统默认）\n", rx.recvBufBytes());
  }

  // 超时 0：不设置 SO_RCVTIMEO。只能验打开与关闭，不能验 receive
  // （没有超时的 socket 会永久阻塞，自检会挂住）。
  {
    UdpReceiver rx;
    log.check(rx.open(port, "", 0, kDefaultRecvBufBytes, err), "超时 0 应打开成功（不设 SO_RCVTIMEO）");
    log.check(rx.isOpen(), "超时 0 打开后 isOpen() 应为 true");
    rx.close();
    log.check(!rx.isOpen(), "超时 0 打开后 close 应正常");
  }
}

// ---------------------------------------------------------------------------
// 2. 打开失败路径：IP 不合法、端口被占
// ---------------------------------------------------------------------------
static void test_open_errors(SelftestLog& log) {
  std::printf("[2] 打开失败路径\n");
  const std::uint16_t port = testPort();
  std::string err;

  // IP 值不合法：inet_pton 拒绝。真机上对应手滑把雷达 IP 填成本机 IP。
  {
    UdpReceiver rx;
    err.clear();
    log.check(!rx.open(port, "300.1.2.3", 200, kDefaultRecvBufBytes, err),
              "段值越界的 IP 应打开失败");
    log.check(!err.empty() && err.find("IP") != std::string::npos,
              "非法 IP 错误应点明「IP 格式不合法」");
    log.check(!rx.isOpen(), "非法 IP 打开失败后不应持有 socket");
    std::printf("      段值越界: %s\n", err.c_str());
  }

  // IP 段数不足：同样必须在 inet_pton 被拒绝。
  {
    UdpReceiver rx;
    err.clear();
    log.check(!rx.open(port, "1.2.3", 200, kDefaultRecvBufBytes, err),
              "段数不足的 IP 应打开失败");
    log.check(!err.empty() && err.find("IP") != std::string::npos,
              "段数不足错误同样应点明「IP 格式不合法」");
    std::printf("      段数不足: %s\n", err.c_str());
  }

  // 同端口二次 bind：SO_REUSEADDR 不允许两个活跃 UDP 监听同端口
  // （那是 SO_REUSEPORT 的语义）。真机上对应驱动起了两个进程。
  {
    UdpReceiver first;
    err.clear();
    log.check(first.open(port, "", 200, kDefaultRecvBufBytes, err), "第一个实例应打开成功");
    UdpReceiver second;
    err.clear();
    log.check(!second.open(port, "", 200, kDefaultRecvBufBytes, err),
              "同端口二次 bind 应失败（EADDRINUSE）");
    log.check(!err.empty() && err.find("绑定") != std::string::npos,
              "bind 失败错误应点明「绑定」与端口");
    log.check(!second.isOpen(), "bind 失败后不应持有 socket");
    std::printf("      端口被占: %s\n", err.c_str());
  }
}

// ---------------------------------------------------------------------------
// 3. 未打开就收
// ---------------------------------------------------------------------------
static void test_receive_without_open(SelftestLog& log) {
  std::printf("[3] 未打开就收\n");

  UdpReceiver rx;
  std::vector<std::uint8_t> buf(64);
  std::string err;
  log.check(rx.receive(buf.data(), buf.size(), err) == -1, "未打开时 receive 应返回 -1");
  log.check(!err.empty() && err.find("未打开") != std::string::npos,
            "未打开错误应点明「socket 未打开」");
  std::printf("      未打开即收: %s\n", err.c_str());

  // 没打开过就 close：幂等，不得崩。
  rx.close();
  rx.close();
  log.check(!rx.isOpen(), "从未打开时 close 后 isOpen() 应为 false");
}

// ---------------------------------------------------------------------------
// 4. 收发契约：自发自收 + src_ip 回显 + 超时语义
// ---------------------------------------------------------------------------
static void test_send_receive(SelftestLog& log) {
  std::printf("[4] 收发契约（回环）\n");
  const std::uint16_t port = testPort();
  std::string err;

  UdpReceiver rx;
  if (!rx.open(port, "", 200, kDefaultRecvBufBytes, err)) {
    log.check(false, "打开接收端应成功");
    std::printf("      %s\n", err.c_str());
    return;
  }

  // 4.1 自发自收：字节数、内容、src_ip 三方核对。
  const std::vector<std::uint8_t> payload = {0x55, 0xAA, 0x05, 0x0A, 1, 2, 3, 4, 0xDE, 0xAD};
  log.check(sendLoopback(port, payload.data(), payload.size()), "回环发送应成功");

  std::vector<std::uint8_t> buf(2048);
  std::string src;
  err.clear();
  const int n = rx.receive(buf.data(), buf.size(), err, &src);
  log.check(n == static_cast<int>(payload.size()), "receive 应返回全部发送字节");
  log.check(!err.empty() == false, "收到数据时 err 应为空");
  log.check(src == "127.0.0.1", "src_ip 应回显发送方地址（127.0.0.1）");
  log.check(std::memcmp(buf.data(), payload.data(), payload.size()) == 0,
            "收到的数据应与发送逐字节一致");
  std::printf("      src_ip = %s，%d 字节\n", src.c_str(), n);

  // 4.2 不传 src_ip（探针的用法）：同样能收，且不写调用方的字符串。
  const std::vector<std::uint8_t> payload2 = {0x10, 0x20, 0x30};
  log.check(sendLoopback(port, payload2.data(), payload2.size()), "第二次回环发送应成功");
  err.clear();
  const int n2 = rx.receive(buf.data(), buf.size(), err);
  log.check(n2 == static_cast<int>(payload2.size()), "不传 src_ip 时同样应收到数据");
  log.check(err.empty(), "不传 src_ip 收到数据时 err 应为空");

  // 4.3 无数据 + SO_RCVTIMEO：返回 0 且 err 为空——超时是正常轮询
  // 节奏的一部分，不是错误。真机上这就是"这一轮没有新包"。
  const auto t0 = std::chrono::steady_clock::now();
  err.clear();
  const int n3 = rx.receive(buf.data(), buf.size(), err);
  const auto waited = std::chrono::steady_clock::now() - t0;
  log.check(n3 == 0, "无数据时应返回 0（超时）");
  log.check(err.empty(), "超时不是错误，err 应为空");
  log.check(std::chrono::duration_cast<std::chrono::milliseconds>(waited).count() >= 150,
            "应至少等满一个超时周期（200 ms 量级）");
  log.check(rx.isOpen(), "超时后 socket 应保持打开");
  std::printf("      无数据超时: n=0，等待 %lld ms\n",
              static_cast<long long>(
                  std::chrono::duration_cast<std::chrono::milliseconds>(waited).count()));
}

// ---------------------------------------------------------------------------
// 5. close 契约：幂等，之后再收返回 -1
// ---------------------------------------------------------------------------
static void test_close_contract(SelftestLog& log) {
  std::printf("[5] close 契约\n");
  const std::uint16_t port = testPort();

  UdpReceiver rx;
  std::string err;
  if (!rx.open(port, "", 200, kDefaultRecvBufBytes, err)) {
    log.check(false, "打开接收端应成功");
    return;
  }
  rx.close();
  rx.close();  // 幂等：重复 close 不得崩、不得关别人的 fd
  log.check(!rx.isOpen(), "close 后 isOpen() 应为 false");
  log.check(rx.recvBufBytes() == 0, "close 后 recvBufBytes() 应归零");

  std::vector<std::uint8_t> buf(64);
  err.clear();
  log.check(rx.receive(buf.data(), buf.size(), err) == -1, "close 后再收应返回 -1");
  log.check(!err.empty() && err.find("未打开") != std::string::npos,
            "close 后再收错误应点明「socket 未打开」");
  std::printf("      close 后再收: %s\n", err.c_str());
}

int main() {
  std::printf("unilidar_driver UDP 接收离线自检（本机回环）\n\n");
  SelftestLog log;
  test_open_contract(log);
  test_open_errors(log);
  test_receive_without_open(log);
  test_send_receive(log);
  test_close_contract(log);

  if (log.report() != 0) return 1;
  std::printf("结论：UdpReceiver 的打开/接收/关闭行为（含 IP 配错、端口被占、"
              "未打开即收、超时契约）\n      在回环上全部符合预期；实物接入只需把"
              "监听 IP 与端口换成雷达实际所发。\n");
  return 0;
}
