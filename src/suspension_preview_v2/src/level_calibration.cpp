#include "suspension_preview_v2/level_calibration.hpp"

namespace suspension_preview_v2 {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kDegToRad = kPi / 180.0;

double norm3(const double v[3]) {
    return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

double clampd(double v, double lo, double hi) {
    if (!std::isfinite(v)) return 0.0;
    return v < lo ? lo : (v > hi ? hi : v);
}

}  // namespace

bool accel_to_up(const double a[3], double max_tilt_deg, double out[3]) {
    const double n = norm3(a);
    if (!(n > 0.0) || !std::isfinite(n)) return false;

    // 加速度计静止时读的是比力，指向天，但不同驱动的整轴符号约定不一致。
    // 不猜约定，直接取与车体 z 同向的那一支——前提是车没倒过来，这一点由
    // max_tilt_deg 兜住。
    const double s = (a[2] >= 0.0) ? 1.0 : -1.0;
    double u[3] = {s * a[0] / n, s * a[1] / n, s * a[2] / n};

    // up 与车体 z 的夹角。超限说明车没摆正（或标定时被人推了、或加速度计
    // 读数根本不是重力），此时无论取哪一支都可能是反的，只能判失败
    const double cos_tilt = clampd(u[2], -1.0, 1.0);
    if (cos_tilt < std::cos(max_tilt_deg * kDegToRad)) return false;

    out[0] = u[0];
    out[1] = u[1];
    out[2] = u[2];
    return true;
}

bool build_level_rotation(const double up[3], const double forward_hint[3],
                          double min_up_norm, double min_forward, LevelRotation& out) {
    out = LevelRotation{};  // 失败时保持单位阵，调用方据此退回原行为

    const double gn = norm3(up);
    if (!(gn > min_up_norm) || !std::isfinite(gn)) return false;

    const double zL[3] = {up[0] / gn, up[1] / gn, up[2] / gn};

    const double fn = norm3(forward_hint);
    if (!(fn > 0.0) || !std::isfinite(fn)) return false;
    const double f[3] = {forward_hint[0] / fn, forward_hint[1] / fn, forward_hint[2] / fn};

    // 施密特的第一步：把车头方向投到水平面上，减去它在竖直方向上的分量
    const double dot = f[0] * zL[0] + f[1] * zL[1] + f[2] * zL[2];
    double xL[3] = {f[0] - dot * zL[0], f[1] - dot * zL[1], f[2] - dot * zL[2]};
    const double xn = norm3(xL);
    // 车头几乎垂直时这个投影趋于 0，朝向完全由噪声决定。与其给一个随机水平面，
    // 不如明确失败——min_forward 是"投影长度下限"，同时也就是夹角的余弦阈值
    if (!(xn > min_forward) || !std::isfinite(xn)) return false;
    xL[0] /= xn;
    xL[1] /= xn;
    xL[2] /= xn;

    // 右手系由叉乘补齐，不做第二次正交化：zL 与 xL 已正交且均为单位向量，
    // 叉乘结果必然也是单位向量，再正交化只会引入一次多余的归一化
    const double yL[3] = {
        zL[1] * xL[2] - zL[2] * xL[1],
        zL[2] * xL[0] - zL[0] * xL[2],
        zL[0] * xL[1] - zL[1] * xL[0],
    };

    // 三行拼成 R：p_level = R · p_body
    out.r[0] = xL[0];
    out.r[1] = xL[1];
    out.r[2] = xL[2];
    out.r[3] = yL[0];
    out.r[4] = yL[1];
    out.r[5] = yL[2];
    out.r[6] = zL[0];
    out.r[7] = zL[1];
    out.r[8] = zL[2];

    out.valid = true;
    out.gravity_mps2 = gn;
    out.tilt_deg = std::acos(clampd(zL[2], -1.0, 1.0)) / kDegToRad;
    return true;
}

}  // namespace suspension_preview_v2
