# 更新日志

本文件记录各版本的重要变更。格式参考 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本号遵循[语义化版本](https://semver.org/lang/zh-CN/)。

仓库里有五个包，各自独立演进：`suspension_preview`（v1，历史版本）、
`suspension_preview_v2`、`suspension_control_v3`、`unilidar_driver`、`car_balancer`。
本文件按仓库层面的版本记录。

## [未发布]

对应 `suspension_control_v3` 的 3.3.x。主线是三件事：预瞄前馈从"单点比例"升级为
时间域 FIR、把"测什么"从凭感觉补用例改成按覆盖率指路、以及把 CI 补齐为
构建 + 内存检查 + 覆盖率三道门禁。

### 驱动：串口接收路径落地（`unilidar_driver`）

L1 的原生通讯接口是 TTL UART，官方 SDK 默认经 CH340 类模块以 2,000,000 bps
转 USB 接入（`/dev/ttyUSB0`）——此前仓库内驱动副本只有 UDP 接收，与实物
链路不符。本轮把独立仓库 `qi-neo/unilidar-driver` 的实现同步进来：

- 新增 `serial_receiver`（POSIX termios：`O_NONBLOCK` + `cfmakeraw` +
  `CLOCAL` + 关硬件流控 + `VMIN=0/VTIME=0`；打开前 `isatty`、打开后
  `tcflush`；读契约 0 = 无数据 / -1 = 出错，与 `udp_receiver` 一致）。
  报文定界复用 `packet_parser`（串口是字节流，解析器本就按无边界字节流
  设计），协议层零改动
- 节点新增 `transport` 参数（`serial` 默认 / `udp` 备选）与
  `serial_device`、`serial_baud`；诊断日志按传输层分别给出排障建议
- 新增 `serial_receiver_selftest`：PTY 主从对虚拟串口，64 项断言（Linux 平台；
  macOS 本机 63 项——`B2000000` 是 Linux 扩展常量，没有它的平台跳过 2 Mbps
  断言），覆盖
  整帧直达、跨写拆包、垃圾与伪帧头重同步、无数据契约、一次多帧、IMU
  帧与各类打开失败路径。本机（macOS）实测全过
- 离线自检总数 843 → **907** 项（驱动 135 + 64；串口 64 为 Linux 平台数，
  macOS 本机 63），README 中英同步；
  UDP 从"唯一实现"改为"备选路径"，CH340 串口成为文档中的实物默认链路

### 驱动：2 Mbps 压力自检与 `packet_parser` 坏帧修复（`unilidar_driver`）

继续同步独立仓库 `qi-neo/unilidar-driver` 的本轮改动：

- 新增 `serial_stress_selftest`（14 项断言）：PTY 主端按 2 Mbps 灌入合成
  报文，同时掺三种干扰——整秒边界注入 16 B 垃圾 + 12 B 伪帧头、每 2 秒
  一次 100 ms 读暂停（瞬态背压）、帧必然跨写。断言灌入帧数 == 解出帧数、
  `bad_crc`/`bad_tail`/`seq_gaps`/`duplicates` 全 0、缓冲与 RSS 有界、
  实际速率不低于目标 80%。CI 以 `--seconds 3` 运行；本机 60 秒满量实测
  灌入 126,636 帧全部解出（点云 113,972 / IMU 12,664），缓冲峰值
  2,064 B，RSS 峰值 1.7 MB。至此「CH340 需实测」缩小到只剩物理层误码率
- **修复 `packet_parser` 坏帧后丢整帧**：`pull()` 的扫描游标 `pos` 原在
  for 循环外声明，`bad_size`/`bad_tail`/`bad_crc` 分支 `erase(pos + 1)`
  后 `continue` 时未重置 `pos`，下一轮从旧位置扫描而内容已前移，跳过真
  帧头——每次坏帧后再丢一整帧。**该 bug 由压力自检最先发现**：此前
  135 项报文自检与 64 项串口自检全绿，但它们的坏帧用例后面都不接好帧，
  走不到恢复路径。修复为每轮从 0 开扫；回归 135 + 64 + 14 项全绿
- 测试基建抽取为 `selftest_fixture.hpp`（PTY 主从对、帧构造、边写边读），
  与 `serial_receiver_selftest` 共用；重构后断言逐条核对一条不差
- 离线自检总数 907 → **921** 项（新增驱动串口压力 14 项），README 中英、
  包级 README（与独立仓库同源同步）更新；`ros-ci.yml` 与 `sanitizer.yml`
  各新增「驱动串口压力自检」步骤

### 驱动：主循环抽出 `DriverLoop`，节点实时性与背压可离线实测

继续同步独立仓库 `qi-neo/unilidar-driver` 的 6b089e4：

- 节点里唯一"有行为"的只有「收包 → 定界 → 解码 → 拼帧」这一段，原先埋在
  ROS 节点里，吞吐 / 单轮耗时 / 缓冲增长只能靠真机或 ROS 容器观测。现在抽成
  不依赖 ROS 的 `DriverLoop`（发布作为回调注入），节点退化成薄壳：只提供
  "从哪读"与"怎么发布"。节点与自检跑的是同一份循环代码，不存在两套逻辑
  各自漂移
- 新增 `node_loop_selftest`（28 项断言），三个场景：
  - 正常节奏（2 Mbps + 2 ms 节拍）：零丢失、缓冲有界、没有一轮读满上限、
    拼帧完整、单轮均值/p99 耗时度量。10 秒实测 2,639 帧灌入全部解出，
    均值 0.026 ms / p99 0.065 ms / 最大 0.289 ms，缓冲峰值 1,022 B；
  - 慢节拍（8 ms，模拟调度不及时）：仍零丢失、缓冲有界。**需说清的事实**：
    PTY 从端队列（macOS 约 512 B / Linux 约 4 KB）才是唯一积压点且自带上限，
    单线程生产者写不进去只能等下一拍，所以"消费慢于生产"不会让定界缓冲
    无限增长；真机上这条队列是内核串口缓冲 + USB 缓冲，同样有界只是更大。
    所以断言的是"节拍慢 4 倍仍不静默丢帧"，不去制造不会发生的溢出；
  - 超载与恢复：单批 64 KB 灌进 16 KiB 上限，`append()` 整批丢弃并计入
    `buffer_overflow`（丢帧可见、可诊断），随后逐帧续收 60 帧全部解出
- 节点诊断新增实时性与背压一行：吞吐 Mbps、帧数、轮询次数、单轮均值/p99/
  最大耗时、定界缓冲当前值与峰值、整批丢弃次数、读满单轮上限次数、读错误
- 离线自检总数 965 → **1016** 项（驱动报文解析 151 → 202，对着 gcov 报告补掉
  `packet_parser` / `frame_decoder` 共 17 行盲区：`reset()` 整个方法、
  `append()` 累积超限、半截帧头前的重同步、点数组越出报文、帧内距离门限、
  IMU 短帧；六个变异体全部被新断言抓住）；`ros-ci.yml`
  与 `sanitizer.yml` 各新增对应步骤；包级 README 与源码按独立仓库同源覆盖
- A2 追加验证（继续同步独立仓库，节点实时性专项）：同一套 gcov 流程用在
  主循环上，`driver_loop.cpp` 查出的 21 行未覆盖全是异常路径——读错误恢复、
  单轮读满上限、空云线 pending 复位、`dropPendingScans()` 统计复位语义、
  重复包端到端交付。新增 `[4] 追加验证` 段落（37 项断言，节点主循环
  28 → **65**），六个场景全部内存源、毫秒级：读错误注入与恢复（已入缓冲的
  20 帧照常解出、恢复后不重复计数）、深积压读满（4 KiB chunk × 130 帧，
  一轮正好 131072 B、`read_capped_polls==1`、130 帧零丢失；PTY 从端队列
  自限流构造不出深积压，改用内存源模拟内核串口缓冲深/录包回放）、
  18 条空云线不发布空帧（`dropped_frame_range==5400`、只发 1 帧 5400 点，
  坐实 pending 复位）、半圈丢弃（stamp 取新不取陈废、ring 1..18 各 300、
  幂等）、统计复位（只清循环统计、解析器状态不动——语义锁定）、重复线
  端到端（36 帧解出、2 帧各 5400 点）。补完未覆盖只剩 1 行（`pollOnce()`
  收尾大括号，非可执行代码）；六个变异体全部被新断言抓住且只抓目标断言；
  60 s 长时程实测 15,830 帧零丢失、缓冲峰值 1022 B、p99 0.078 ms 与 2 s
  短跑一致、RSS 3.6 MB
- 离线自检总数 1016 → **1053** 项（节点主循环 28 → 65）；`ros-ci.yml` 与
  `sanitizer.yml` 步骤名同步；README.en.md 分组计数与总数同步（含上轮
  漏网的报文解析 151 → 202）；包级 README 与源码按独立仓库同源覆盖

### 驱动：传输层覆盖率补齐（C2，`unilidar_driver`）

继续同步独立仓库 `qi-neo/unilidar-driver`。C1 补完解析层、A2 补完主循环后，
`gcov` 查下来驱动核心里只剩两个传输层文件没过覆盖率分析：`serial_receiver.cpp`
81.69%、`udp_receiver.cpp` 67.14%。未覆盖行逐条归因后分三类：用户态构造不出的
失败（`tcgetattr`/`tcsetattr` 失败分支）、要耗尽 fd 才够得着的硬错误
（`socket()`/`recvfrom()`）、平台语义——驱动自设 `VMIN=0/VTIME=0`，PTY
无数据时 read 返回 0 而非 EAGAIN（`readSome` 的 EAGAIN 分支是防御代码）。
设备拔出的 PTY 语义被首轮 CI 实测推翻，见下。

- **串口接收自检 64 → 81**（macOS 63 → 80）。新增 `[9] 打开契约与设备拔出`
  17 项：空设备名直接失败且 err 提示「空」；EACCES（`chmod 0` 构造）提示
  「Permission denied（把当前用户加入 dialout 组后重新登录）」，root 下自动
  跳过（权限位被绕过）；设备拔出断言 EOF 语义；fd 失效后 `readSome` 走错误
  分支（返回 -1、err 点名设备与「失败」，覆盖 176-177 行）——错误分支靠
  「从外部关掉驱动持有的 fd」模拟句柄变野，为此 `SerialReceiver` 新增
  `fd()` 访问器（select/poll 型调用方同样用得上，非测试后门）；拔出后
  `isOpen` 保持 true、`device()` 不变，close 后句柄归零、再读返回 -1。
  行覆盖 81.69% → **88.73%**（macOS 本机），剩余未覆盖行的不可构造原因
  逐条写在自检头注释
- **UDP 接收自检新建 38 项**（第五套离线自检，`udp_receiver_selftest`）。
  此前 `udp_receiver.cpp` 只有真实回环脚本覆盖 happy path；新建自检补上全部分支：
  打开契约与 `SO_RCVBUF`/`SO_RCVTIMEO` 回读（打印 `getsockopt` 生效值不是
  请求值）、非法 IP（`300.1.2.3`/`1.2.3`）早退、bind 非本机地址
  （`198.51.100.7`，EADDRNOTAVAIL）报「绑定」失败且点明是哪个 IP、
  未打开即收报「未打开」、超时返回 0 且不污染 err、close 幂等。
  行覆盖 67.14% → **92.86%**，剩余未覆盖行（`socket()`/`recvfrom()` 硬错误）
  需耗尽 fd 才构造得出。与 `tests/udp_loopback_test.sh` 的分工写进头注释：
  脚本=集成测试只走 happy path，自检=单元测试覆盖每条分支含失败路径
- **两轮 CI 翻车与修正（平台语义教训，本轮最重要经验）**：
  1. 设备拔出的 PTY 语义原按「Linux 主端关闭后从端 read 返回 -1/EIO」
     断言，首轮 CI 在 Ubuntu 24.04 runner（内核 6.8）上两平台实测都是
     **EOF（返回 0、err 为空）**——4 个核心库组合与 ROS job 当场挂掉。
     修正：删掉 `#ifdef __APPLE__` 分平台断言，统一按 EOF 断言（readSome
     无法区分 EOF 与暂时无数据，都是 0，POSIX read 在 `VMIN=0` 下的
     固有限定；真机 USB 串口拔出两平台都是错误返回，EOF 是 PTY 独有的
     仿真语义），错误分支改由 fd 失效场景覆盖。
  2. 同端口二次 bind 原按「SO_REUSEADDR 下报 EADDRINUSE」断言，二轮 CI 在
     Linux runner 上挂掉——**Linux 上两个都设了 SO_REUSEADDR 的 UDP socket
     允许绑同一端口**（多进程负载均衡语义），macOS 才报 EADDRINUSE。修正：
     换成 bind 非本机地址（EADDRNOTAVAIL，两平台一致；且「local_ip 填了
     不在这台机器上的 IP」真机上比端口被占更常见）。
  **平台语义不能凭文献/记忆断言，要以目标 CI 环境的实测为准；实测与预期
  不符时，改的是断言和文档，不是事实。**
- **变异验证**：十一个变异体（串口 4 + UDP 7：空设备名校验删除、EACCES 提示删除、
  「未打开」err 删除、readSome 错误分支 err 填充删除、非法 IP 早退删除、
  bind 失败返 true、「绑定」提示改名、bind 失败 err 退化成无 IP 通用文案、
  src_ip 回填删除、超时返 -1、close 不清 recv_buf_bytes_）全部被新断言抓住，
  且只抓目标断言、不误伤
- **60 s 长时程复跑**（2 Mbps 灌入 120 MB）：126,637 帧全部解出零丢失、
  `readSome` 调用 136,026 次、缓冲峰值 2,064 B、RSS 峰值 1.7 MB、
  `bad_size 59`（= 注入的伪帧头数，全部按预期拒斥）；双编译器 × 普通/ASan+UBSan
  四组合全过（UDP 自检连跑 5 次零抖动，默认端口 46311 与回环脚本的 46201 不
  冲突，可用 `UDP_SELFTEST_PORT` 覆盖）
- 离线自检总数 1053 → **1108** 项（串口 64 → 81、新增 UDP 38）；
  `ros-ci.yml` 与 `sanitizer.yml` 新增「驱动 UDP 接收自检（38 项）」步骤、
  串口步骤名 64 → 81；README.en.md 分组计数与总数同步（seven → eight 组）；
  vendored 副本与独立仓同源逐字节一致

### 驱动：四个 P0 级缺陷修复同步（`unilidar_driver`）

继续同步独立仓库 `qi-neo/unilidar-driver` 的 13b8320。四个缺陷的共同点是
原有 135 项自检全绿却一个都没抓到——用例只覆盖「典型长度、典型数量级」，
缺陷恰好藏在边界上：

- **逐点 `time` 用 float32 存绝对 Unix 时间，运动去畸变完全失效（静默）。**
  `PointXYZIT::time` 是 FLOAT32 却填绝对 Unix 秒，float32 在 1.7e9 量级的
  量化步长是 **128 秒**（代码注释里写的「约 256 ns」其实是 double 的步长，
  两者被搞混了）。实测：一条扫描线 300 个点（跨度约 2.3 ms）全部塌成同一个
  值、只有 1 个可区分值，下游去畸变会算成「整条线同一时刻」，且不报错不告警。
  改为**相对帧起始的偏移秒数**（0..0.2 s，量化步长约 15 ns，对 7.7 µs 点间
  步长有约 500 倍余量），与 PCL/Velodyne/Ouster/Livox 的约定一致；绝对时刻
  仍等于 `header.stamp + time`。节点侧按「本线时间戳 − 首线时间戳」平移，
  保证跨线连续递增
- **`packet_parser` 数据段短于 `DataInfo` 时越界读（内存安全，外部输入可触发）。**
  `packet_size` 合法下界 24（数据段长 0），而读序号的 `DataInfo` 有 16 字节；
  `pull()` 校验通过后无条件 `memcpy`，24..39 字节的包会越过报文末尾读
  4..16 字节。这类短包是协议允许的（数据段为 0 时官方连 CRC 都不校验），
  不能靠 `bad_size` 拒掉。修复：读序号前先确认数据段足够，不足则跳过序号
  跟踪但照常交付该帧
- **`append()` 的缓冲上限在单批过大时形同虚设**：超限 `clear()` 后仍原样
  `insert`，若 `len` 本身超限则清完再插依然超限。改为单批超限直接整批丢弃
  并计入 `buffer_overflow`
- **`PullStatus::kNoFrame` 是从未返回的死枚举**：缓冲耗尽统一返回
  `kNeedMore`，留着会让调用方误以为要处理第三种状态，已删除
- 新增自检段落 `[10] P0 缺陷回归`（16 项断言，报文自检 135 → **151**）。
  越界读用例**不依赖 ASan**：libc++ 的 container-overflow 标注在 `-O1`
  及以上会被优化掉，而 CI 的 sanitizer job 正是 `-O1`，故改用可观测行为判定
- 离线自检总数 921 → **937** 项（报文解析 135 → 151），README 中英与
  `ros-ci.yml` 步骤名同步；包级 README 与六个源码文件按独立仓库同源覆盖

### CI 与文档

- **CI 门禁与文档口径对齐**（核验 10-23 推送后的 CI 日志时发现）：`ros-ci.yml`
  的控制自检步骤名还写着旧数「428 项」，而场景 41 异步通道进来后实跑已是
  443——步骤名改回 443；更要紧的是 README 声称 CI 跑五组自检、含「驱动串口
  接收 63 项」，但 `ros-ci.yml` 里根本没有这一步（vendored 副本的 CMakeLists
  早已注册 `serial_receiver_selftest`，却没有任何步骤调用它——与当年
  「install(TARGETS) 装了却没人跑」是同一类问题）。两处都补上：`ros-ci.yml`
  新增「驱动串口接收自检（64 项）」步骤；`sanitizer.yml` 新增同款
  ASan + UBSan 步骤（termios/PTY 路径同样要过 sanitizer，与独立仓库
  `unilidar-driver` 的 core job 双编译器 × ASan 互为双保险）。本机对
  vendored 副本按 CI 同款命令实测：ASan/UBSan 下 macOS 63 项全过（Linux 为
  64 项，见下条）
- **串口自检数字按 CI 平台计 64 项**：波特率表里的 2 Mbps 断言依赖
  `B2000000`，它是 Linux 扩展常量，macOS 的 termios 没有、跳过该项——同一份
  `serial_receiver_selftest` 在 CI（Linux）实跑 64 项、本机（macOS）63 项。
  文档与 CI 步骤名统一按 CI 平台 64 项计并注明平台差异；驱动总数 906 → **907**
  项，README 中英、包级 README（与独立仓库同源同步）、NOTICE 同步
- **sanitizer 拆成独立 workflow**（`.github/workflows/sanitizer.yml`）：GitHub 的
  workflow 徽章只能对应整个 workflow，没有"单个 job 的徽章"；混在 `ros-ci.yml`
  里时 README 只能放一个总徽章，看不出 ASan 这一项到底绿没绿。拆开后 ASan 徽章
  真正只反映 sanitizer 状态。两个 workflow 都保留"失败自动把日志贴回 PR"，
  查看失败原因不需要登录 Actions
- **README 新增「质量保障」章节**（中英）：四条门禁各跑什么、红了的含义、覆盖边界
  （sanitizer 只覆盖不依赖 ROS/PCL 的纯逻辑层；覆盖率排除 `serial_port.cpp`），
  以及两个真实案例（ASan 抓到堆越界写、覆盖率暴露口径打架）

### 新增

- **预瞄前馈 FIR**（`use_preview_fir`，默认关闭）：前馈从单点比例改为
  `h = Σ_j f[j]·z_r(start + j·Δt·v) / Σ_used f[j]`。权重由
  `lqr_gain_design.py` 第七节按作动器滞后 τ 最优解给出。直流增益守恒、
  远端缺格按最近有效格兜底、零权重拍按实际用上的权重归一化
- **增益设计工具自校验**：每解出一组增益都附带迭代收敛性、DARE 残差与闭环
  谱半径三项检查，缺一即标"不可用"——这些数要抄进 YAML，给了不稳定增益
  必须当场看出来
- **姿态多级抗噪滤波链与语义预瞄**：多级链替掉单级低通（相位滞后只付一次）；
  语义预瞄在障碍物落到前馈取样范围内时抑制该侧前馈
- **多帧位置对齐融合**：按车速把历史帧换算到最新帧坐标系后逐格取中位数，
  并给出跨帧离散度置信度（超阈值判低置信置无效）
- **作动器位置反馈闭环**：从驱动轮询或外部话题获取实际位置，检出堵转/失步后
  降级；反馈过期自动退出判定。新增 `max_position_mm` 挡住"多打一个零"这类
  离谱上报
- **节点消息转换层** `msg_convert.hpp`：四元数转 roll/pitch 与剖面字段映射
  抽成不依赖 ROS 的纯函数，使其能被离线自检覆盖（原先埋在节点里，CI 碰不到）
- **CI 三道门禁**：新增 ASan/UBSan job（内存与未定义行为）与覆盖率 job
  （gcovr，行 80% / 分支 70% / 函数 85% 三重门槛，HTML 明细上传 artifact）。
  v2 三个纯函数自检此前只装成可执行、没有步骤调用，现已补进 CI
- **启动前参数统一校验**（`param_check.hpp`，不依赖 ROS 因此可被自检覆盖）：
  四组参数（控制律 / 回路 / 校验器 / 状态机）共 30 项判据，每条都写明"填错会
  导致什么现象"，而不只是"必须为正数"。致命项拒绝启动，可疑项只提示——
  例：`height_limit_mm = 0` 会让输出恒为 0（作动器一动不动但日志正常）、
  `controller_kind = 2` 会被静默当成 LQI、`min_lookahead_m > max_lookahead_m`
  会让 `std::clamp` 的前提被破坏
- **参数与 YAML 一致性检查**（`tools/check_ros_params.py`，进 CI）：ROS2 的
  YAML 键名是纯字符串匹配，键写错或改名没同步都会让参数**静默失效**。脚本把
  "YAML 有但节点不读"和"节点读但 YAML 没写"两个方向都拦下。首次运行即抓到
  5 个参数（融合窗口三项、降级超时、单拍时长上限）节点一直在读却不在 YAML 里，
  用户无从发现——已补进配置
- **QUICKSTART.md**：面向刚拿到代码的人，一条 `g++` 命令、不需要 ROS 与硬件
  就能确认代码完整可跑
- v2：用重力向量建虚拟水平面，雷达竖直安装时也能取到真实高程

### 修复

- **FIR 的时间域/空间域错配**：权重按控制周期推导（第 j 拍 = j 个周期之后），
  落地时却当成剖面格距使用。两者只在 `车速 = 格距/周期` 时相等，于是车速越低
  提前量越大——论文车速 0.5 m/s 下有效提前 0.43 s，是最优值的 20 倍，而工具
  早先已量化过"提前 0.35 s 比不提前更差"。改为按 `车速 × 控制周期` 换算，
  无车速时退回单点并如实上报 `feedforward_fir=false`
- **谱半径判据在共轭复根下不收敛**：原先用幂迭代取末次范数，主导特征值为
  一对复根时迭代方向来回摆，把稳定的 0.99 报成 1.28、直接颠倒结论。改用每步
  范数增长的几何平均（`‖A^m v‖^(1/m) → ρ`，对复根同样成立）
- **LQI 积分饱和**：修正量上界改为跟随"剩余行程权威"（= 行程上限 − 本拍最大
  前馈）。固定上限能防无限累积，但防不了权威耗尽——长期钉在边界后，前馈再来
  一个凸起就没有余量了。改后凸起到来时高位修正量会被立即拉回，无需等误差反向
- **GCC 下 `std::clamp` 未声明**：`profile_sampling.hpp` 等只含 `<cmath>`，
  libc++ 会间接引入 `<algorithm>` 而 libstdc++ 不会，本地 macOS 编得过、
  ROS Humble 容器编不过
- **`offline_demo` 的退出码只看协议往返**：解析精度算出 `mean_err` 却只打印，
  于是"点云解析回归"根本门不住，CI 里只能靠 `|| true` 当作不阻塞的装饰。
  现在它自己门住有效格占比与均值误差
- **位置上报的"非有限值"检查是死代码**：入参是 `std::array<int>`，int 装不下
  NaN，该分支永远不成立。换成可达的合理性判据
- **LQI 零增益警告写在读取增益之前**，检查的是默认值 0，于是只要
  `controller_kind=1` 就必然误报一次——哪怕增益填得完全正确。判定移入
  参数校验（在所有参数读完后再跑），同一件事只判定一次
- 测试代码里两处堆溢出（用 `999` 当"无凸起"哨兵却直接索引 100 格数组），
  由 ASan 抓到——当时 234 项断言全绿

### 修改

- 离线自检 142 项 -> **395 项**；`suspension_control_v3/src/` 六个源文件
  **行覆盖 100%**
- `ControlParams` 新增 `control_period_s`（由 `control_rate_hz` 推出，不单独
  配置：两处各写一遍迟早不一致，而那等于给整条权重改了时间标度）；
  `ControlLoopParams` 新增 `max_position_mm`
- `ControlStatus.msg` 新增 `feedforward_fir`（本拍前馈实际走的是哪种）与
  `lqi_windup_clamped`（修正量被剩余权威钳住，台架频繁置位就该减小前馈增益
  或增大行程，而不是继续加积分增益）
- `TemporalProfileFuser::kMaxFrames` 由 private 改为 public：调用方需要知道
  "配再大的 `frames` 也只有这么多帧真被用上"
- 中英文 README 同步更新；新增 CI 徽章与覆盖率说明


### 安全修复（P0，来自一轮独立代码审查）

- **姿态断流不再被静默忽略。** 原先 `attitude_age_ms` 只在 `submit_attitude()`
  里更新，IMU 断流后该函数不再被调用，age 永远冻结、`attitude_.valid` 永远是 true，
  控制器带着几秒前的姿态持续输出、状态机零故障位。现在每拍按 `now` 重算 age，
  过期即置无效，并把"姿态是否可信"纳入状态机的健康度判据（原先只看剖面，
  等于"眼睛瞎了但路还在更新"也判健康）。
- **姿态保护改为单调。** 原先 20° 急停、>69° 反而因为被 `max_abs_attitude_rad`
  当"坏数据"整帧拒收而绕过安全判定。现在"超物理上限"单独置 `attitude_implausible`，
  状态机据此锁存姿态超限并进急停；`param_check` 增加"物理上限必须严格大于安全阈值"
  的致命校验。
- **没有可信姿态时不重算输出。** 反馈项唯一来源是 IMU，缺失时改为保持上一拍下发值，
  而不是带着 `has_attitude=false` 继续算（那会让输出慢慢滑向零位）。
- **硬件急停失败要重试。** 原先只在进入 Emergency 的那一拍调用一次，串口写失败一次
  就再也不会急停。现在成功后不再重复、失败按 10 Hz 重试直到送达。
- **补上 `base_link` 静态 TF。** 仓库里没有任何包发布 `base_link`，预瞄节点每次
  `lookupTransform` 都失败、100% 丢帧（README 快速开始也没有这一步，开箱必然不通）。
  节点新增 `publish_static_tf` / `lidar_frame` / `mount_x..yaw` 参数自动发布静态变换，
  安装量默认全 0（单位变换）并在启动日志里明确告警"必须实测"。
- **`estimate_cell` 的 `min_points<=0` 越界。** `size() < 0` 恒假后 `minmax_element`
  会解引用空区间（YAML 填 0 即可触发），改为显式下限 1。
- **`QUICKSTART` 的编译命令漏了 `param_check.cpp`**，按文档"五分钟上手"会链接失败。
- **文档数字同步**：自检 395 -> 410 项；仓库级合计 596 -> 617 项。
- **vendored 驱动同步**：`unilidar_driver` 副本与独立仓库对齐（CRC32 改为只覆盖
  数据段、`quaternion_order` 默认 xyzw），并补上仓库根目录缺失的 `NOTICE`
  （副本源码注释一直引用它，但文件从未在本仓库存在）。

### 验证

- `control_selftest` 410/410 通过（新增 15 项：场景 33 姿态断流降级、34 超物理上限
  急停、35 紧急停车重试、36 参数阈值分层）；
- ASan+UBSan 构建运行 410/410、退出码 0、无报告；
- v2 四个纯函数自检 29/12/28/69 全过；
- 驱动 `decoder_selftest` 69/69、UDP 回环 PASSED（在独立仓库中验证后同步）。
### P1/P2 批：把"看起来正常"的隐患补上

第二批独立审查修复。原则同上：能用测试或 CI 证明的才改，需要台架定案的留到实机。

**安全/正确性**

- **降级恢复去抖失效**：`healthy_elapsed_` 只在离开降级时清零，进入降级前累积的
  健康时长会直接满足 `recover_duration_s`，降级下一拍就弹回 Leveling（在找平超时
  场景会来回横跳）。改为进入任何状态都从零计时。
- **下行链路"发了就算成功"**：`last_command_` 先更新后发送，串口写失败一次就会被
  下一拍的"没变化"去重判据吞掉，作动器永远收不到这条指令。改为发送成功才记账，
  失败下拍自动重试。
- **找平只按时间判完成**：新增 `leveling_max_tilt_deg`（默认 2.9°）与
  `leveling_max_wait_s`（默认 2 s）——姿态没回到零位基准就继续等，等太久转降级。
  就绪判据用"相对零位标定基准的误差"，车停在斜坡上也能正确判断。
- **非法波特率被静默回退**：`is_baud_supported()` 是死代码，非法值会被悄悄换成
  115200。现在启动即 FATAL。
- **控制频率越界**：`control_rate_hz` 超出 [50, 1000] Hz 时拒绝启动；与 200 Hz
  标定值不一致时给非致命提示（离散系数的时间标度会变）。

**参数校验补缺**（原先完全没校验）

- `kp_roll/kp_pitch` 必须为正、`kd_roll/kd_pitch` 非负——符号写反是正反馈；
- 滤波链系数 `attitude_gyro_alpha/attitude_accel_alpha` 落在 (0,1]、死区非负；
- `max_jump_m` 必须小于物理量程的 2 倍，否则帧间跳变检查永远不会触发；
- `leveling_max_wait_s` 必须大于 `leveling_duration_s`；
- `control_period_s` 超出 [0.001, 0.02] s 判致命。

**可观测性**

- `ControlStatus` 新增 `saturated` / `rate_limited`：控制器内部本来就有这两个标志，
  但一直没上报，联调时只能看到"作动器跟不上"、看不到原因。

**工程规范**

- `suspension_control_v3` 注册 CTest，CI 的 `colcon test` 从"空转"变成真的跑自检；
  `colcon test-result` 不再带 `|| true`。
- 自检 410 → 428 项（场景 37~40）。

### 实时性主线：把串口写移出 200 Hz 控制线程

- **新增命令通道层**（`ICommandSink`：`command_sink.hpp/.cpp`）。控制线程
  `submit()` 非阻塞投递"最新目标"，后台线程负责真正写驱动器；写失败按固定
  间隔重试，**急停走优先路径插队**（已送达后不重复）。通道析构前会 `flush`
  最后一帧，避免退出时把作动器停在半路。
- **合并语义**：下行只剩几十 Hz 时，中间指令本来就没有执行意义，直接覆盖；
  `command_coalesced` 上报被覆盖条数——这个数持续增长说明下行带宽不够。
- **同步通道保留**：不注入通道时自动套 `DirectCommandSink`，行为与旧版完全一致
  （既有 428 项断言零改动通过）；节点显式换成异步通道。
- **控制周期超时计数**：原始 `dt` 超过 1.5 倍标称周期即计一次 `dt_overruns`。
  这是"控制线程有没有被阻塞"的直接证据——旧实现把阻塞串口写放在这条线程里，
  而软件上的 200 Hz 并不能保证物理上每 5 ms 算一次。
- `ControlStatus` 新增 `command_coalesced` / `command_retries` /
  `command_last_sent_age_ms` / `command_sink_async` / `dt_overruns`。
- 自检 428 → **443 项**（场景 41 异步通道：不阻塞/合并/重试/急停优先/析构冲刷；
  场景 42 回路接入异步通道后 600 拍不被 20 ms/次的写阻塞）。

**构建规范化（P2）—— 已落地**

这条最初被回退过一轮。根因是 `suspension_core` 是"源码不依赖 PCL、构建上却
链接 PCL"的静态库，导出时把 Boost/VTK/QHULL/FLANN 的导入目标带给了下游。
拆库（见上）之后彻底解决：

- v2：`install(TARGETS suspension_core EXPORT suspension_core_targets ...)` +
  `ament_export_targets(... HAS_LIBRARY_TARGET)`；`suspension_extractor`
  （依赖 PCL）仅内部使用、不导出；
- v3：删掉 `get_filename_component` 上溯与 `find_library` 硬找路径，
  链接 imported target（兼容 `suspension_core` / `<pkg>::suspension_core`
  两种命名，找不到就 configure 阶段 FATAL）；
- 头文件用 `ament_target_dependencies` + 显式
  `${suspension_preview_v2_INCLUDE_DIRS}`（容器里实测前者对"库目标"不透传）。

CI 最终全绿：`colcon build` + `colcon test` + ASan + 覆盖率。

### 构建重构：核心库与 PCL 层拆分

`suspension_core` 原先把点云解析（PCL）与协议/串口/驱动放在同一个静态库里，
导致"核心库不依赖 PCL"这句话只在源码层面成立：静态库链接 PCL 后，其
`INTERFACE_LINK_LIBRARIES` 会带上 QHULL/VTK/Boost/FLANN 等导入目标，下游
（v3）一旦不 `find_package(PCL)`，configure 阶段就会报一长串 target not found。

- 新增 `suspension_extractor`（`road_profile_extractor.cpp` + `pipeline.cpp`），
  依赖 PCL；接口输入本来就是 `pcl::PointCloud`，所以管线也归这一层；
- `suspension_core` 只保留协议 / 串口 / 作动器驱动 / 趋势 / 障碍 / 标定 /
  参数校验 / 输出接口，**源码与头文件都不再出现 `pcl/`**；
- 预览节点与 `offline_demo` 显式链接两个库；`suspension_driver_node` 只链 core。

本地验证：把 9 个 core 源文件在**不带任何 PCL include** 的情况下逐个编译通过，
并 grep 确认核心头文件无 `pcl/` 引用。这是后续把 v3 切到 `ament_export_targets`
（去掉路径 hack）的前提。

### 控制/标定：高度(mm)→PWM(us) 标定与 P0 板接口层（`suspension_preview_v2`，离线可跑）

P0 板只认 PWM 脉宽（舵机），不认高度；高度到 PWM 的映射每台车不同，必须在台架实测
得到标定表。本轮把这段映射收敛到 `suspension_preview_v2`，交付**离线可跑**成果（真机
CH340 误码率与 P0 板流板仍阻塞在硬件，与 B1 同口径）：

- 新增 `height_calibration`（纯函数，不依赖 ROS / PCL / 串口）：`loadCsv` / `setPoints` /
  `pwmForHeight`（分段线性插值 + 区间外钳位）/ `heightForPwm` / `toJson` / `toHeader` /
  `strictlyIncreasing`；空表换算返回 `NaN`，调用方据 `hasCalibration()` 判断，绝不凭默认值硬算
- `loadCsv` 兼容 UTF-8 BOM（Excel 导出的表常带）与首行表头（如 `target_mm,...` 当表头跳过）；
  真正的坏数据（首字段仍是数值但后续字段非数值）照常报错
- 新增 `P0BoardLink`：把「目标高度(mm)」经标定换算成 PWM，再经 0xAA55 协议下发 `0x13`
  命令（payload = 轮号 + PWM）；四轮逐条下发。`setTargetHeightMm` 在未连接 / 未标定 /
  轮号越界 / 换算 `NaN` / 下发失败时返回 `false`，且 `last_error()` 非空
- `protocol` 新增 `kIdSetTargetPwm = 0x13`，与 `0x11`（直接下发 mm）区分，避免协议单位歧义；
  `ISuspensionDriver` 加 `set_target_pwm` 虚函数，`SerialSuspensionDriver` 补实现
- 新增 `calibration_tool`（CLI）：`validate` / `generate`（`--json` 可切换 JSON）/ `sample` /
  `--selftest`；含 `calibration_template.csv` 五步法模板
- 新增三个 CTest 用例并注册进 CI：
  `height_calib_selftest`（31 项：点集校验 / 插值精度 / 钳位 / 空表 NaN / 往返一致 /
  序列化 / CSV 含注释空行）、`calibration_tool --selftest`（复用同一份 31 项）、
  `p0_link_selftest`（18 项，PTY 仿真板端，校验「mm→标定→0xAA55(0x13) 帧」全链路：收 4 帧
  `id=0x13` 且 wheel/pwm 与标定换算一致，未标定下发被拒；无 PTY 环境自动跳过返回 0）
- v2 纯函数断言 69 → **149** 项（新增 80：31 + 31 + 18），仓库离线断言总数 596 → **676** 项；
  README 中英同步，新增 [docs/calibration.md](docs/calibration.md)

## [3.2.2] - 2026-10-25

### 修复：B3 提交引入的构建断裂与标定层三处静默缺陷

**P0-1 构建断裂：新增纯虚函数未同步测试替身（CI 直接红）**

`4ea1c3e` 给 `ISuspensionDriver` 加了纯虚 `set_target_pwm()`，只补了
`SerialSuspensionDriver` 的实现，漏了 `control_selftest.cpp` 里的两个测试替身
（`MockDriver` / `AsyncProbeDriver`）。抽象类无法实例化，`suspension_control_v3`
的 443 项控制自检**编译不过**——CI 的 `control_selftest` 步骤直接失败。
该提交的 CHANGELOG 记的是"断言 596 → 676（新增 80）"，实际是把既有的 443 项
整段打没了：新加 80，静默损失 443。

- 修复：两个测试替身补上 `set_target_pwm()`，与各自的 `set_target_height()`
  共用同一套成败语义（发送失败模拟、调用计数），而不是随手 `return true`
- 教训已写进注释：给被广泛实现的接口加纯虚函数是破坏性变更，必须同步所有实现者

**P1-2 标定表 CSV：坏数据行被当成表头静默丢弃**

`loadCsv()` 用 `try { std::stod(首字段) } catch { 跳过 }` 判定表头，而这条规则
对**每一行**都成立，于是任何首字段解析失败的数据行都被当作表头悄悄跳过；
`std::stod("1e400")` 抛的 `out_of_range` 也被同一个 `catch(...)` 吞掉。
实测：`0,1500` / `1e400,1700` / `40,1900` 三行数据读进来只剩 2 个点，
`loadCsv` 返回成功、无报错，插值结果偏了也看不出来。

- 修复：表头只允许出现在第一个数据行之前；越界数值单独判 `out_of_range` 并报错

**P1-3 反算 `heightForPwm()` 在反装舵机（PWM 随高度递减）时结果完全错误**

端点钳位与二分查找都隐含"pwm 随 mm 递增"的假设，而标定表只保证 mm 严格递增，
pwm 递减是合法的台架结果（`feedforward_sign` 就是为反装准备的）。
实测：递减表 `0→2000 / 10→1500 / 20→1000` 上查 `heightForPwm(1750)`
返回 **0 mm**，正确值是 **5 mm**，且往返一致性检查不成立。

- 修复：先判方向再钳位；二分改为线性扫描（标定表只几十个点，代价可忽略），
  使任意形状的表都给出确定答案；新增 `pwmMonotonic()` 如实报告表是否可靠

**P1-4 标定表缺舵机量程校验，异常 PWM 可直达硬件**

`setPoints()` 只查了 `std::isfinite(pwm_us)`。RDS3115 mg 只认 **500~2500 μs**
（见 `hardware/` 下的 BOM 与接线说明），但 `999999` 这种显然写错单位的值能被
正常载入，`P0BoardLink` 照原样下发——舵机会被顶到机械极限并持续堵转
（单路 1.6 A，四路 ~10 A）。

- 修复：新增 `kMinServoPwmUs` / `kMaxServoPwmUs` 常量，`setPoints()` 拒绝越界点；
  `P0BoardLink::setTargetHeightMm()` 在下发前再判一次（硬件边界的纵深防御）

### 自检

- `height_calib_selftest` / `calibration_tool --selftest` 31 → **46** 项
  （新增 [H] 缺陷回归 15 项：越界行报错、表头位置、递减表反算与往返一致、
  单调性识别、量程上下界拒绝、模板 CSV 不被误伤）
- `control_selftest` 恢复可编译，443 项全绿；`p0_link_selftest` 18 项全绿
- README 计数同步（控制自检 395 → 443，v2 纯函数 149 → 179，仓库总数 676 → 706）

### 保全

改动前已在 `backup/pre-optimization-20261025` 分支与 `baseline/pre-optimization-20261025`
带注释 tag 上保留旧代码（`4d5a9ba`）。

## [3.2.1] - 2026-10-15

对应 `suspension_control_v3` 的 3.2.x。主线是前馈取样从"固定距离单点取样"
改成"车速自适应 + 帧龄补偿 + 前后轴差异化"，把预瞄控制里一直对不上相位的
部分补上。

### 新增

- **前馈取样距离随车速自适应**：距离 = clamp(车速 × `lookahead_time_s`,
  `min_lookahead_m`, `max_lookahead_m`)，响应时域取作动器走满行程耗时的
  2.5 倍余量（默认 0.35 s）。固定距离在低速时取样点落到有效范围外，高速时
  来不及——论文实验车速 0.5 m/s，固定 1 m 相当于提前 2 s 取路面。
  节点新增 `speed_topic`（默认 `/odom`）订阅车速；没有车速信息时自动退回
  固定 `lookahead_m`，与旧行为一致。`assume_speed_mps` 面向没有里程计的
  台架匀速实验
- **前后轴差异化预瞄**：后轴取样距离比前轴短一个轴距（`wheelbase_m`），
  后轮比前轮晚一个"轴距/车速"遇到同一路面特征。前馈输出从左右两路扩为
  四路（左前/右前/左后/右后），`ControlOutput::feedforward_mm` 随之扩为 4 元素
- **帧龄运动补偿**：剖面坐标系锚定在采集时刻的车体原点，车在帧龄内已经前进
  车速×帧龄，取样位置把这个距离加回去。剖面 10~20 Hz、帧龄常年 50~100 ms，
  2 m/s 时不补偿就是 100~200 mm 的系统性偏差。`ControlInput` 新增
  `profile_age_s`（由控制回路按 `now - frame.stamp` 填写）；无车速信息时
  无法补偿，行为与此前一致。实际取样位置经 `ControlStatus.sample_position_m`
  与 `profile_age_ms` 每拍上报
- **取样线性插值**：在目标距离两侧的括号格之间插值，不再直接取最近格，
  避免 5 cm 量化在前馈里形成台阶式行程指令
- **单侧轮迹失效跨侧回退**：某一侧无有效数据时借对侧高程，避免左右不对称的
  前馈（等效于一个伪横滚指令）。该情况由 `ControlStatus.feedforward_fallback`
  置位，可观测
- `ControlStatus.msg` 新增 `lookahead_m` / `speed_mps` / `feedforward_fallback`
  三个字段，"前馈到底按多远取了样"在联调时直接可见
- 控制回路新增 `submit_speed()` 与 `speed_timeout_s`（默认 0.5 s）：
  过期视为无车速并退回固定取样距离
- `tools/lqr_gain_design.py` 新增第 6 节，直接打印与
  `config/control_params.yaml` 一一对应的落地参数

### 修改

- `ControlInput` 新增 `has_speed` / `speed_mps`；`IControlLoop` 接口新增
  `submit_speed()`
- 离线自检 89 项 -> 142 项：新增场景 16（车速自适应预瞄：钳位、无车速回退、
  回路状态上报与过期）、场景 17（前后轴差异化、括号格插值、跨侧回退）与
  场景 18（帧龄运动补偿：取样位置平移、无车速不补偿、负/NaN 帧龄钳位、
  与自适应距离叠加、回路帧龄上报）
- 控制节点 `package.xml` / `CMakeLists.txt` 补 `nav_msgs` 依赖声明
  （接 `/odom` 用），`tools/check_deps.py` 核对通过

### 修复

- 驱动副本 `unilidar_driver` 同步到独立仓库的 v1.0.1（`package.xml` 补
  `geometry_msgs` 依赖后独立仓库版本号随之更新，本仓库副本原停留在 1.0.0）。

## [3.1.0] - 2026-10-14

**首次在 ROS Humble 干净环境下完整构建通过。** v3.0.1 及更早的 tag
拉下来是编不过的——五个包全都有问题，请直接用本版本。

### 新增

- `unilidar_driver`：自研宇树 L1 雷达驱动，发布 `/unilidar/cloud` 与
  `/unilidar/imu` 两个话题，不链接官方静态库。此前这两个话题只能靠官方
  `unitree_lidar_ros2` 节点的预编译 `.a` 提供。
- 英文版 README（`README.en.md`）。
- 持续集成（`.github/workflows/ros-ci.yml`）：ROS Humble 容器 + `colcon build`，
  并在 CI 上真正跑三个离线自检（v3 控制 89 项、驱动解析 63 项、v2 点云链路）。
  这是 v2 / v3 / 驱动三个包首次获得编译验证。
- `tools/check_deps.py`：核对五个包的 CMakeLists 与 package.xml 依赖声明
  是否一致，接进 CI，漏声明即失败。纯文本检查，不需要 ROS。
- `suspension_preview`（v1）补齐包内 README 与最小 launch，并明确标注为
  历史版本、新用途请用 v2。此前该包只有源码与 CMakeLists，无法判断用法。

### 修复

首次接入 CI 后暴露的一批真实缺陷。这些包此前从未在干净环境编译过，
只在开发机上凑着能过：

- `suspension_preview`：只 `find_package(pcl_ros)` 靠传递依赖拿 PCL，
  拿不到头文件路径。补 `find_package(PCL REQUIRED COMPONENTS common filters)`
  与 `${PCL_INCLUDE_DIRS}` / `${PCL_LIBRARIES}`；`package.xml` 补
  `libpcl-all-dev` 声明。
- `suspension_preview`：`pcl/filters/pass_through.h` 文件名写错
  （PCL 只有 `passthrough.h`）；`pcl_ros/transforms.h` 应为 `.hpp`。
- `suspension_preview_v2`：`road_profile_extractor.cpp` 同样的
  `pass_through.h` 错误。
- `suspension_preview_v2`：`ament_export_libraries` 导出了 `suspension_core`
  却没 `install()`，导致 `suspension_control_v3` 在 configure 阶段报
  "exports the library 'suspension_core' which couldn't be found"。
- `suspension_control_v3`：用 `rosidl_get_typesupport_target` 引用跨包消息，
  而该函数只认本包 `rosidl_generate_interfaces` 生成的 target；
  改用 rosidl 导出的 `suspension_preview_v2__rosidl_typesupport_cpp`。
- `suspension_control_v3` 节点：`ctrl_.lookahead_m` 并不存在
  （`lookahead_m` 属于 `ControlLoopParams`，`ControlParams` 里对应的是
  `max_lookahead_m`，两者不是一个东西）；`make_serial_driver` 返回
  `unique_ptr` 而 `set_driver` 收 `shared_ptr`。

现在 CI 上 `colcon build` 五个包全过，并跑通 v3 的 89 项自检、
驱动的 63 项自检、v2 的离线链路示例。

### 修改

- `unilidar_driver` 已拆出为独立仓库 [qi-neo/unilidar-driver](https://github.com/qi-neo/unilidar-driver)。
  本仓库保留一份副本以便单独 clone 时能直接构建；**改动以独立仓库为准，
  再同步回来**。两个仓库的 README 均已互相注明。
- 清理 `car_balancer` 里 `ros2 pkg create` 留下的样板元数据：`version 0.0.0`、
  `maintainer root`、`root@todo.todo`、description 与 license 的 TODO 文本。
- 补上 `car_balancer` 缺失的运行时依赖声明：原先只有 `test_depend`，
  而 `test_imu_node.py` 实际 import 了 `rclpy` 与 `sensor_msgs`。

## [3.0.1] - 2026-10-12

### 修改

- 主 README 补充 v3 控制节点说明。
- 修复 `car_balancer` 的 entry point：原先两条入口之间缺逗号被 Python 隐式拼接，
  且 `test_imu_node.py` 不在模块目录内，两个命令都无法运行。
  指向未提供模块的 `balancer_node` 入口已移除。

## [3.0.0] - 2026-10-12

### 新增

- `suspension_control_v3`：把 v2 提取出的路面剖面真正用来控制作动器输出。
  - `IInputValidator`：剖面校验，拒绝 9 类坏数据。
  - `ISafetyStateMachine`：安全状态机，三分支输出契约（用控制器输出 /
    保持上次值 / 强制回中）。
  - `IController`：预瞄前馈 + 无导数 PD + 运动学解耦 + 限幅与速率限制。
  - `IControlLoop`：把校验、状态机、控制器、驱动器串成一条链。
  - 控制核心不依赖 ROS、不依赖 PCL，可脱离整车单独编译与自检（89 项检查）。
  - `tools/lqr_gain_design.py`：按性能指标反解 LQR / 增量式 LQI 增益。

### 修改

- 发布前加固：零位标定、降级时限、急停锁存、单周期时长封顶、硬件急停。

## [2.0.0] - 2026-10-12

### 新增

- `suspension_preview_v2`：v1 的接口化重构版。
  - 把点云解析、输出、悬架驱动封装为接口（`IRoadProfileExtractor`、
    `IProfileSink`、`ISuspensionDriver`）。
  - 把与 ROS 无关的部分抽成独立的 `suspension_core` 库，可离线复用与测试。
  - 新增 CRC16 / int16 毫米串口协议，剖面同时作为 ROS 话题发布。
- `docs/review/01-code-review-v1.md`：v1 代码审查报告。

### 修改

- 修正 v1 的四类问题：QoS 兼容性、串口 raw 模式、TF 失败兜底、
  无效格静默填 0。
- 参数由宏定义改为 YAML + launch，运行时生效。

---

## 更早

- 2026-09-17 初始提交：`suspension_preview`（v1 节点）+ `car_balancer`，
  以及从 CSDN 迁移的博客（后已移出本仓库）。该阶段未发版本。
