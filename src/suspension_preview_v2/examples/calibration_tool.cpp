// 高度(mm) -> PWM(us) 标定 CLI 工具。纯函数、不依赖 ROS 与串口，可离线运行。
//
//   calibration_tool validate <csv>          校验标定表是否可用
//   calibration_tool generate <csv> [--json] 导出为 C++ 表字面量（默认）或 JSON
//   calibration_tool sample   <csv> <mm>     给定目标高度，输出换算 PWM(us)
//   calibration_tool --selftest              运行内置标定自检，退出码 0/1
//
// 编译：
//   g++ -std=c++17 -I include -I examples
//     examples/calibration_tool.cpp src/height_calibration.cpp

#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "suspension_preview_v2/height_calibration.hpp"
#include "calibration_checks.hpp"

using suspension_preview_v2::HeightCalibration;

namespace {

int usage() {
    std::fprintf(stderr,
        "用法:\n"
        "  calibration_tool validate <csv>\n"
        "  calibration_tool generate <csv> [--json]\n"
        "  calibration_tool sample   <csv> <height_mm>\n"
        "  calibration_tool --selftest\n");
    return 2;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    const std::string cmd = argv[1];

    if (cmd == "--selftest") {
        const int fails =
            suspension_preview_v2::calibration_checks::run_calibration_checks();
        if (fails == 0) {
            std::printf("calibration_tool: 自检全部通过\n");
            return 0;
        }
        std::printf("calibration_tool: 自检失败 %d 项\n", fails);
        return 1;
    }

    if (cmd == "validate") {
        if (argc < 3) return usage();
        HeightCalibration c;
        std::string err;
        if (!c.loadCsv(argv[2], err)) {
            std::fprintf(stderr, "校验失败: %s\n", err.c_str());
            return 1;
        }
        std::printf("标定表有效: %zu 点\n", c.size());
        std::printf("  严格递增: %s\n", c.strictlyIncreasing() ? "是" : "否");
        std::printf("  可经 generate 导出，或经 sample 取任意高度的 PWM\n");
        return 0;
    }

    if (cmd == "generate") {
        if (argc < 3) return usage();
        HeightCalibration c;
        std::string err;
        if (!c.loadCsv(argv[2], err)) {
            std::fprintf(stderr, "载入失败: %s\n", err.c_str());
            return 1;
        }
        bool as_json = (argc >= 4 && std::string(argv[3]) == "--json");
        std::printf("%s\n", as_json ? c.toJson().c_str() : c.toHeader().c_str());
        return 0;
    }

    if (cmd == "sample") {
        if (argc < 4) return usage();
        HeightCalibration c;
        std::string err;
        if (!c.loadCsv(argv[2], err)) {
            std::fprintf(stderr, "载入失败: %s\n", err.c_str());
            return 1;
        }
        const double mm = std::atof(argv[3]);
        const double pwm = c.pwmForHeight(mm);
        if (!std::isfinite(pwm)) {
            std::fprintf(stderr, "换算失败（未标定或高度越界？）\n");
            return 1;
        }
        std::printf("%.3f mm -> %.1f us\n", mm, pwm);
        return 0;
    }

    return usage();
}
