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

    std::printf("\n----------------------------------------\n");
    std::printf("检查项 %d，失败 %d\n", checks, fails);
    return fails;
}

}  // namespace calibration_checks
}  // namespace suspension_preview_v2
