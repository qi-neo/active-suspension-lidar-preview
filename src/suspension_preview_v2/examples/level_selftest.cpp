// 虚拟水平面标定的离线自检。不依赖 ROS、不依赖 PCL，可直接 g++ 编译：
//   g++ -std=c++17 -I include examples/level_selftest.cpp src/level_calibration.cpp
//
// 覆盖三类：纯数学性质（正交、右手、保长）、几何正确性（倾斜与竖直安装能否
// 被旋回水平）、以及退化输入（零重力、车头与重力平行）该失败还是该通过。

#include <cmath>
#include <cstdio>

#include "suspension_preview_v2/level_calibration.hpp"

using suspension_preview_v2::GravityAccumulator;
using suspension_preview_v2::LevelRotation;
using suspension_preview_v2::accel_to_up;
using suspension_preview_v2::apply_level_rotation;
using suspension_preview_v2::build_level_rotation;

namespace {

int g_checks = 0;
int g_fails = 0;

void check(bool ok, const char* what) {
    ++g_checks;
    if (!ok) {
        ++g_fails;
        std::printf("  FAIL  %s\n", what);
    }
}

bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

double det3(const double* m) {
    return m[0] * (m[4] * m[8] - m[5] * m[7]) -
           m[1] * (m[3] * m[8] - m[5] * m[6]) +
           m[2] * (m[3] * m[7] - m[4] * m[6]);
}

// R·Rᵀ 与单位阵的最大偏差
double orthogonality_error(const double* m) {
    double worst = 0.0;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) s += m[i * 3 + k] * m[j * 3 + k];
            const double target = (i == j) ? 1.0 : 0.0;
            const double e = std::fabs(s - target);
            if (e > worst) worst = e;
        }
    }
    return worst;
}

void scenario_1_accel_to_up() {
    std::printf("\n=== 1. 加速度计读数 -> 竖直向上 ===\n");
    double up[3] = {0.0, 0.0, 0.0};

    // 水平静止，常规符号约定（比力指向天）
    const double a_flat[3] = {0.0, 0.0, 9.81};
    check(accel_to_up(a_flat, 60.0, up), "水平静止应通过");
    check(near(up[0], 0.0, 1e-12) && near(up[1], 0.0, 1e-12) && near(up[2], 1.0, 1e-12),
          "水平静止的 up 应为 (0,0,1)");

    // 整轴取反的驱动：靠"与车体 z 同向"这一条纠正过来
    const double a_inv[3] = {0.0, 0.0, -9.81};
    check(accel_to_up(a_inv, 60.0, up), "反号约定也应通过");
    check(near(up[2], 1.0, 1e-12), "反号约定的 up 仍应指向 +z");

    // 绕 x 横滚 20°：R^T·(0,0,1) = (0, sin20, cos20)
    const double roll20 = 20.0 * 3.14159265358979323846 / 180.0;
    const double a_tilt[3] = {0.0, 9.81 * std::sin(roll20), 9.81 * std::cos(roll20)};
    check(accel_to_up(a_tilt, 60.0, up), "倾斜 20 度应在容限内");
    check(near(up[1], std::sin(roll20), 1e-9) && near(up[2], std::cos(roll20), 1e-9),
          "倾斜 20 度的 up 应还原出倾角");

    // 80° 超出 60° 容限：此时左右两支几乎对称，取哪支都可能是反的
    const double roll80 = 80.0 * 3.14159265358979323846 / 180.0;
    const double a_steep[3] = {0.0, 9.81 * std::sin(roll80), 9.81 * std::cos(roll80)};
    check(!accel_to_up(a_steep, 60.0, up), "倾斜 80 度超出 60 度容限应失败");

    // 非法输入
    const double a_nan[3] = {0.0, NAN, 9.81};
    check(!accel_to_up(a_nan, 60.0, up), "NaN 输入应失败");
    const double a_zero[3] = {0.0, 0.0, 0.0};
    check(!accel_to_up(a_zero, 60.0, up), "零向量应失败");
}

void scenario_2_level_rotation() {
    std::printf("\n=== 2. 施密特正交化建水平系 ===\n");
    LevelRotation lr;

    // 已经水平：R 应为单位阵。
    // up 传加速度计给出的竖直向量（模长约 9.81），单位向量也可以——
    // 这里只取方向，量纲由调用方用 GravityAccumulator 管
    const double up0[3] = {0.0, 0.0, 9.81};
    const double fwd0[3] = {1.0, 0.0, 0.0};
    check(build_level_rotation(up0, fwd0, 0.5, 0.1, lr), "水平输入应成功");
    check(lr.valid, "水平输入的 valid 应为真");
    for (int i = 0; i < 9; ++i) {
        const double want = (i % 4 == 0) ? 1.0 : 0.0;
        check(near(lr.r[i], want, 1e-12), "水平输入的 R 应为单位阵");
    }
    check(near(lr.gravity_mps2, 9.81, 1e-9), "重力模长应如实记录");
    check(near(lr.tilt_deg, 0.0, 1e-9), "水平输入倾角应为 0");

    // 车体横滚 20°：R·up 必须把 up 映到 +z
    const double roll20 = 20.0 * 3.14159265358979323846 / 180.0;
    const double up_tilt[3] = {0.0, 9.81 * std::sin(roll20), 9.81 * std::cos(roll20)};
    check(build_level_rotation(up_tilt, fwd0, 0.5, 0.1, lr), "倾斜输入应成功");
    double ox = 0.0, oy = 0.0, oz = 0.0;
    apply_level_rotation(lr, up_tilt[0] / 9.81, up_tilt[1] / 9.81, up_tilt[2] / 9.81,
                         ox, oy, oz);
    check(near(ox, 0.0, 1e-12) && near(oy, 0.0, 1e-12) && near(oz, 1.0, 1e-12),
          "倾斜 20 度后 up 应被旋到 +z");
    check(near(lr.tilt_deg, 20.0, 1e-9), "倾角应被如实记录为 20 度");

    // 车头方向在水平面内的投影应落在 +x
    apply_level_rotation(lr, 1.0, 0.0, 0.0, ox, oy, oz);
    check(ox > 0.0 && near(oy, 0.0, 1e-12), "车头方向投影后应落在 +x");

    // 竖直安装：雷达 x 朝上、车头沿雷达 z。这是本项目真实的安装姿态
    const double up_vert[3] = {9.81, 0.0, 0.0};
    const double fwd_vert[3] = {0.0, 0.0, 1.0};
    check(build_level_rotation(up_vert, fwd_vert, 0.5, 0.1, lr), "竖直安装应成功");
    apply_level_rotation(lr, 1.0, 0.0, 0.0, ox, oy, oz);
    check(near(ox, 0.0, 1e-12) && near(oy, 0.0, 1e-12) && near(oz, 1.0, 1e-12),
          "竖直安装：雷达 x 应被旋到 +z");
    apply_level_rotation(lr, 0.0, 0.0, 1.0, ox, oy, oz);
    check(near(ox, 1.0, 1e-12) && near(oz, 0.0, 1e-12),
          "竖直安装：车头方向应被旋到 +x");
    check(near(lr.tilt_deg, 90.0, 1e-9), "竖直安装的倾角应为 90 度");
}

void scenario_3_properties() {
    std::printf("\n=== 3. 正交、右手、保长 ===\n");
    // 几个非平凡朝向，逐个验矩阵性质。用固定的角度而不是随机数，
    // 免得自检本身变成概率性通过
    const double cases[5][2] = {{10.0, 5.0}, {-30.0, 12.0}, {45.0, -20.0},
                                {3.0, 0.0},  {-8.0, 40.0}};
    for (int c = 0; c < 5; ++c) {
        const double roll = cases[c][0] * 3.14159265358979323846 / 180.0;
        const double pitch = cases[c][1] * 3.14159265358979323846 / 180.0;
        // 用 ZYX 把 +z 转到车体系，得到该姿态下的 up（乘 9.81 才是加速度计量纲）
        const double up[3] = {-9.81 * std::sin(pitch),
                              9.81 * std::sin(roll) * std::cos(pitch),
                              9.81 * std::cos(roll) * std::cos(pitch)};
        const double fwd[3] = {std::cos(pitch), 0.0, std::sin(pitch)};
        LevelRotation lr;
        if (!build_level_rotation(up, fwd, 0.5, 0.1, lr)) {
            check(false, "非平凡姿态应能建出水平系");
            continue;
        }
        check(orthogonality_error(lr.r) < 1e-12, "R 应为正交阵（R·Rt = I）");
        check(near(det3(lr.r), 1.0, 1e-12), "det(R) 应为 +1（右手系，不是镜像）");

        // 保长：任取一点，旋转前后模长不变
        const double px = 1.3, py = -0.7, pz = 0.42;
        double ox = 0.0, oy = 0.0, oz = 0.0;
        apply_level_rotation(lr, px, py, pz, ox, oy, oz);
        const double before = std::sqrt(px * px + py * py + pz * pz);
        const double after = std::sqrt(ox * ox + oy * oy + oz * oz);
        check(near(before, after, 1e-12), "旋转应保持模长");

        // 手性：x_L × y_L 应等于 z_L
        const double cx = lr.r[1] * lr.r[5] - lr.r[2] * lr.r[4];
        const double cy = lr.r[2] * lr.r[3] - lr.r[0] * lr.r[5];
        const double cz = lr.r[0] * lr.r[4] - lr.r[1] * lr.r[3];
        check(near(cx, lr.r[6], 1e-12) && near(cy, lr.r[7], 1e-12) &&
                  near(cz, lr.r[8], 1e-12),
              "row1 × row2 应等于 row3（右手）");
    }
}

void scenario_4_degenerate() {
    std::printf("\n=== 4. 退化输入 ===\n");
    LevelRotation lr;
    const double fwd[3] = {1.0, 0.0, 0.0};

    const double up_zero[3] = {0.0, 0.0, 0.0};
    check(!build_level_rotation(up_zero, fwd, 0.5, 0.1, lr), "零重力应失败");
    check(!lr.valid, "失败时 valid 应为假");

    // 模长趋零：方向已无意义，必须判失败而不是给一个随机水平面
    const double up_small[3] = {0.0, 0.0, 1e-6};
    check(!build_level_rotation(up_small, fwd, 0.5, 0.1, lr), "up 模长趋零应失败");

    // 车头几乎与重力平行：水平投影趋于 0，朝向由噪声决定
    const double up_ok[3] = {0.0, 0.0, 9.81};
    const double fwd_parallel[3] = {0.0, 0.0, 1.0};
    check(!build_level_rotation(up_ok, fwd_parallel, 0.5, 0.1, lr),
          "车头与重力平行应失败");

    const double fwd_zero[3] = {0.0, 0.0, 0.0};
    check(!build_level_rotation(up_ok, fwd_zero, 0.5, 0.1, lr), "车头为零向量应失败");

    // 失败后 r 必须还是单位阵：调用方靠这一点退回原行为
    check(near(lr.r[0], 1.0, 1e-12) && near(lr.r[4], 1.0, 1e-12) &&
              near(lr.r[8], 1.0, 1e-12),
          "失败时 R 应保持单位阵");
}

void scenario_5_accumulator() {
    std::printf("\n=== 5. 重力样本累积与静止判定 ===\n");
    GravityAccumulator acc;

    double n = 0.0;
    check(!acc.mean_norm(n), "空累积器应返回失败");
    check(acc.count() == 0, "空累积器计数应为 0");

    // 200 个带噪声的样本，均值应逼近 9.81，抖动远小于阈值
    double seed = 12345.0;
    auto next = [&seed]() {
        seed = std::fmod(seed * 1103515245.0 + 12345.0, 2147483648.0);
        return seed / 2147483648.0 - 0.5;  // [-0.5, 0.5)
    };
    for (int i = 0; i < 200; ++i) {
        acc.add(0.02 * next(), 0.02 * next(), 9.81 + 0.02 * next());
    }
    check(acc.count() == 200, "有效样本计数应为 200");
    check(acc.mean_norm(n), "有样本后应能取出模长");
    check(near(n, 9.81, 0.01), "均值模长应逼近 9.81");
    check(acc.max_std() < 0.02, "静止样本的三轴抖动应小于 0.02");
    check(acc.stable(100, 0.05), "200 个静止样本应判为稳定");
    check(!acc.stable(500, 0.05), "样本数不足时不应判为稳定");

    // 坏样本：计入 rejected，不污染均值
    const std::size_t before = acc.count();
    check(!acc.add(NAN, 0.0, 9.81), "NaN 样本应被拒收");
    check(acc.count() == before, "拒收的样本不应计入均值");
    check(acc.rejected() == 1, "拒收计数应为 1");

    // 确实在动：抖动放大十倍，stability 应判失败
    GravityAccumulator moving;
    for (int i = 0; i < 200; ++i) {
        moving.add(2.0 * next(), 2.0 * next(), 9.81 + 2.0 * next());
    }
    check(!moving.stable(100, 0.05), "明显抖动时不应判为稳定");

    acc.reset();
    check(acc.count() == 0 && !acc.mean_norm(n), "reset 后应恢复空状态");
}

}  // namespace

int main() {
    std::printf("虚拟水平面标定自检\n");
    scenario_1_accel_to_up();
    scenario_2_level_rotation();
    scenario_3_properties();
    scenario_4_degenerate();
    scenario_5_accumulator();

    std::printf("\n----------------------------------------\n");
    if (g_fails == 0) {
        std::printf("检查项 %d，失败 0 -> 全部通过\n", g_checks);
        return 0;
    }
    std::printf("检查项 %d，失败 %d -> 未通过\n", g_checks, g_fails);
    return 1;
}
