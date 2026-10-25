// 高度标定模块的统一离线自检。
//
// 被 height_calib_selftest 与 calibration_tool --selftest 共用，避免两份漂移的检查
// 逻辑。头文件内联实现，不依赖 ROS / PCL / 串口，可纯离线编译与单测。
//
// 覆盖：点集有效性校验（≥2 点 / 严格递增 / 有限值 / 乱序自动排序）、分段线性插值
// 精度、超出标定区间的钳位、空表返回 NaN、height<->pwm 往返一致性、序列化往返、
// 以及含注释与空行的 CSV 载入。

#pragma once

#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <string>
#include <vector>

#include "suspension_preview_v2/height_calibration.hpp"

namespace suspension_preview_v2 {
namespace calibration_checks {

inline int run_calibration_checks() {
    int checks = 0, fails = 0;
    auto check = [&](bool ok, const char* what) {
        ++checks;
        if (!ok) {
            ++fails;
            std::printf("  FAIL  %s\n", what);
        }
    };
    auto near = [](double a, double b, double tol) {
        return std::fabs(a - b) <= tol;
    };

    std::printf("高度标定自检\n");

    // ---- A. 点集有效性校验 ----
    std::printf("\n=== A. 点集有效性校验 ===\n");
    {
        HeightCalibration c;
        std::string err;
        check(c.setPoints({{0.0, 1000.0}, {100.0, 2000.0}}, err),
              "两个有效点应通过");
        check(c.hasCalibration(), "载入后 hasCalibration 应为真");
        check(c.size() == 2, "size 应为 2");
        check(c.strictlyIncreasing(), "严格递增应成立");
    }
    {
        // 单点应被拒绝（至少 2 点）
        HeightCalibration c;
        std::string err;
        check(!c.setPoints({{0.0, 1000.0}}, err), "单点应拒绝（至少 2 点）");
    }
    {
        // 乱序输入应被按 mm 排序后接受，且上下界正确
        HeightCalibration c;
        std::string err;
        check(c.setPoints({{100.0, 2000.0}, {0.0, 1000.0}}, err),
              "乱序输入应被排序接受");
        check(near(c.pwmForHeight(0.0), 1000.0, 1e-9), "乱序排序后下界正确");
        check(near(c.pwmForHeight(100.0), 2000.0, 1e-9), "乱序排序后上界正确");
    }
    {
        // 重复 mm 应被拒绝（严格递增不变式）
        HeightCalibration c;
        std::string err;
        check(!c.setPoints({{0.0, 1000.0}, {0.0, 1100.0}}, err),
              "重复 mm 应拒绝");
    }
    {
        // 非有限值（NAN / INF）应被拒绝
        HeightCalibration c;
        std::string err;
        check(!c.setPoints({{0.0, 1000.0}, {NAN, 1100.0}}, err),
              "非有限 mm 应拒绝");
        check(!c.setPoints({{0.0, NAN}, {100.0, 1100.0}}, err),
              "非有限 pwm 应拒绝");
    }

    // ---- B. 分段线性插值精度 ----
    std::printf("\n=== B. 分段线性插值 ===\n");
    HeightCalibration lin;
    {
        std::string e;
        lin.setPoints({{0.0, 1000.0}, {50.0, 1300.0}, {100.0, 1600.0}}, e);
    }
    check(near(lin.pwmForHeight(0.0), 1000.0, 1e-9), "端点 0mm -> 1000us");
    check(near(lin.pwmForHeight(100.0), 1600.0, 1e-9), "端点 100mm -> 1600us");
    check(near(lin.pwmForHeight(50.0), 1300.0, 1e-9), "中点 50mm -> 1300us");
    check(near(lin.pwmForHeight(25.0), 1150.0, 1e-9), "25mm 线性插值 = 1150us");
    check(near(lin.pwmForHeight(75.0), 1450.0, 1e-9), "75mm 线性插值 = 1450us");

    // ---- C. 超出区间钳位 ----
    std::printf("\n=== C. 区间外钳位 ===\n");
    check(near(lin.pwmForHeight(-10.0), 1000.0, 1e-9), "低于下界钳位到 1000us");
    check(near(lin.pwmForHeight(999.0), 1600.0, 1e-9), "高于上界钳位到 1600us");

    // ---- D. 空表返回 NaN ----
    std::printf("\n=== D. 空表保护 ===\n");
    {
        HeightCalibration empty;
        check(!empty.hasCalibration(), "默认构造无标定");
        check(std::isnan(empty.pwmForHeight(50.0)), "空表 pwmForHeight 返回 NaN");
        check(std::isnan(empty.heightForPwm(1500.0)),
              "空表 heightForPwm 返回 NaN");
    }

    // ---- E. height<->pwm 往返一致（pwm 随 mm 单调时）----
    std::printf("\n=== E. 往返一致性 ===\n");
    check(near(lin.heightForPwm(lin.pwmForHeight(30.0)), 30.0, 1e-6),
          "height<->pwm 往返一致");

    // ---- F. 序列化往返 ----
    std::printf("\n=== F. 序列化 ===\n");
    {
        const std::string js = lin.toJson();
        check(!js.empty() && js.front() == '[', "toJson 以 [ 开头");
        check(js.find("\"mm\":50") != std::string::npos, "toJson 含 mm 字段");
        check(js.find("\"pwm_us\":1300") != std::string::npos,
              "toJson 含 pwm_us 字段");
        const std::string hdr = lin.toHeader("kTable");
        check(hdr.find("kTable") != std::string::npos, "toHeader 含变量名");
        check(hdr.find("1300") != std::string::npos, "toHeader 含数据点");
    }

    // ---- G. CSV 含注释/空行载入 + 非数值拒绝 ----
    std::printf("\n=== G. CSV 载入 ===\n");
    {
        const char* tmp = "calib_checks_tmp.csv";
        std::FILE* f = std::fopen(tmp, "w");
        if (f) {
            std::fprintf(f, "# 标定表注释行\n\n0.0, 1000.0\n50.0, 1300.0\n100.0, 1600.0\n");
            std::fclose(f);
        }
        HeightCalibration csv;
        std::string ec;
        check(csv.loadCsv(tmp, ec), "含注释/空行的 CSV 应载入成功");
        check(csv.size() == 3, "CSV 应读出 3 点");
        check(near(csv.pwmForHeight(50.0), 1300.0, 1e-9),
              "CSV 载入后插值正确");
        std::remove(tmp);

        const char* bad = "calib_checks_bad.csv";
        std::FILE* fb = std::fopen(bad, "w");
        if (fb) {
            std::fprintf(fb, "0.0, abc\n");
            std::fclose(fb);
        }
        HeightCalibration badc;
        std::string ebad;
        check(!badc.loadCsv(bad, ebad), "非数值字段应拒绝");
        std::remove(bad);
    }

    // ---- H. 缺陷回归：表头判定 / 反算方向 / 舵机量程 ----
    //
    // 这一段专门盯三类"自检全绿但行为是错的"缺陷。它们的共同点是都不崩溃、
    // 不报错，只是悄悄给出错误结果，所以必须转成可观测的行为断言。
    std::printf("\n=== H. 缺陷回归 ===\n");
    {
        // H1. 越界数值的数据行不能被当成表头静默跳过。
        // std::stod("1e400") 抛 out_of_range，原先被 catch(...) 当成"表头"吞掉，
        // 标定表凭空少一个点而 loadCsv 仍返回成功。
        const char* over = "calib_checks_overflow.csv";
        std::FILE* fo = std::fopen(over, "w");
        if (fo) {
            std::fprintf(fo, "0.0, 1000.0\n1e400, 1700.0\n40.0, 1900.0\n");
            std::fclose(fo);
        }
        HeightCalibration c;
        std::string e;
        check(!c.loadCsv(over, e), "越界数值行必须报错，不能当表头跳过");
        check(e.find("超出") != std::string::npos, "越界行的错误信息应说明原因");
        std::remove(over);
    }
    {
        // H2. 表头只允许出现在第一个数据行之前。
        // 原先"首字段非数值就跳过"对每一行都成立，数据行写坏会被静默丢弃。
        const char* mid = "calib_checks_midheader.csv";
        std::FILE* fm = std::fopen(mid, "w");
        if (fm) {
            std::fprintf(fm, "0.0, 1000.0\nlabel, 1700.0\n40.0, 1900.0\n");
            std::fclose(fm);
        }
        HeightCalibration c;
        std::string e;
        check(!c.loadCsv(mid, e), "数据行之后出现非数值行必须报错");
        std::remove(mid);
    }
    {
        // H3. 反装舵机（PWM 随高度递减）时反算必须成立。
        // 原先端点钳位无条件假设 pwm 递增，1750us 会被钳到 0mm（正确值 5mm）。
        HeightCalibration c;
        std::string e;
        check(c.setPoints({{0.0, 2000.0}, {10.0, 1500.0}, {20.0, 1000.0}}, e),
              "递减 PWM 的标定表应被接受（反装是合法台架结果）");
        check(near(c.heightForPwm(1750.0), 5.0, 1e-6),
              "递减表：heightForPwm(1750) 应为 5mm（原实现返回 0mm）");
        check(near(c.heightForPwm(c.pwmForHeight(5.0)), 5.0, 1e-6),
              "递减表：height<->pwm 往返一致");
        check(c.pwmMonotonic(), "递减表应判为单调（反算可靠）");
    }
    {
        // H4. 递增方向的既有行为不能被改坏
        HeightCalibration c;
        std::string e;
        c.setPoints({{0.0, 1000.0}, {10.0, 1500.0}, {20.0, 2000.0}}, e);
        check(near(c.heightForPwm(1250.0), 5.0, 1e-6), "递增表反算保持正确");
        check(c.pwmMonotonic(), "递增表应判为单调");
    }
    {
        // H5. 非单调表要如实报告，而不是给出看似正常的错值
        HeightCalibration c;
        std::string e;
        c.setPoints({{0.0, 1500.0}, {10.0, 1000.0}, {20.0, 1700.0}}, e);
        check(!c.pwmMonotonic(), "非单调表应被 pwmMonotonic 识别");
    }
    {
        // H6. pwm_us 必须落在舵机量程内。原先只查 finite，999999us 能被载入并
        // 一路下发到硬件（RDS3115 只认 500~2500us）。
        HeightCalibration c;
        std::string e;
        check(!c.setPoints({{0.0, 1500.0}, {40.0, 999999.0}}, e),
              "超出舵机量程的 pwm 必须拒绝载入");
        check(!c.setPoints({{0.0, 1500.0}, {40.0, 100.0}}, e),
              "低于舵机量程的 pwm 必须拒绝载入");
        check(c.setPoints({{0.0, 500.0}, {40.0, 2500.0}}, e),
              "量程边界 500/2500us 应被接受");
    }
    {
        // H7. 量程校验不能误伤模板：calibration_template.csv 仍要能载入。
        // 模板在 examples/ 下，而 ctest 的工作目录不固定（构建目录/源码目录都
        // 有可能），所以按候选路径依次尝试；找不到就跳过而不是判失败——
        // 这条是防"校验过严"的护栏，不是被测对象本身
        const char* candidates[] = {
            "calibration_template.csv",
            "src/suspension_preview_v2/examples/calibration_template.csv",
            "../examples/calibration_template.csv",
            "../../src/suspension_preview_v2/examples/calibration_template.csv",
        };
        for (const char* p : candidates) {
            if (std::FILE* ft = std::fopen(p, "r")) {
                std::fclose(ft);
                HeightCalibration c;
                std::string e;
                check(c.loadCsv(p, e),
                      "标定模板 CSV 应仍能载入（量程校验不可过严）");
                check(c.size() == 5, "模板应读出 5 个标定点");
                break;
            }
        }
    }

    std::printf("\n----------------------------------------\n");
    std::printf("检查项 %d，失败 %d\n", checks, fails);
    return fails;
}

}  // namespace calibration_checks
}  // namespace suspension_preview_v2
