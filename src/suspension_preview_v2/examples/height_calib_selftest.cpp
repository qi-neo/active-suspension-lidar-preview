// 高度标定模块的离线 CTest 自检。
//
// 不依赖 ROS / PCL / 串口硬件，可直接 g++ 编译：
//   g++ -std=c++17 -I include -I examples \
//     examples/height_calib_selftest.cpp src/height_calibration.cpp
//
// 检查逻辑统一放在 calibration_checks.hpp，供本文件与 calibration_tool --selftest
// 共用，避免两份漂移的实现。

#include <cstdio>

#include "calibration_checks.hpp"

int main() {
    const int fails =
        suspension_preview_v2::calibration_checks::run_calibration_checks();
    if (fails == 0) {
        std::printf("height_calib_selftest: 全部通过\n");
        return 0;
    }
    std::printf("height_calib_selftest: 失败 %d 项\n", fails);
    return 1;
}
