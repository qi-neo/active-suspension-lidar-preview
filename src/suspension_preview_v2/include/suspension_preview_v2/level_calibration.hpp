// 虚拟水平面标定：用重力向量建立"真正的水平"坐标系。
//
// 为什么需要它：车体坐标系（base_link）是人为定义的，激光雷达装上去时也不可能
// 恰好水平——本项目的雷达是竖直安装的，车体 z 轴与真实竖直差了几十度。于是
//   · 点云里的 z 不是"离地高度"，而是沿车身某根轴的分量；
//   · 斜坡上更糟：车身被悬架维持水平，路面在车体系里是斜的。
// 不校正的话，地面窗口框到的既不是地面，左右轮迹也不是真实轮迹。
//
// 做法是教科书里的施密特正交化，只借重力一条先验：
//   z_L = normalize(up)                       竖直向上（up 由静止时的加速度计给出）
//   x_L = normalize(x_hint - (x_hint·z_L) z_L) 车头方向投影到水平面，去掉竖直分量
//   y_L = z_L × x_L                            叉乘补齐，保证右手系
// 三行拼成 R，p_level = R · p_body。R 是正交阵（R·Rᵀ = I，det = +1）。
//
// 这里只做纯数学，不碰 ROS：调用的节点负责把加速度计读数变换到目标坐标系、
// 判定静止、以及标定失败时退回单位阵。
#pragma once

#include <cmath>
#include <cstddef>

namespace suspension_preview_v2 {

// 重力样本累积器。静止时连续喂加速度计读数，用均值压噪声、用离散度判静止。
//
// 只做一阶统计（和 / 平方和），O(1) 内存——控制侧没有理由为了求个均值缓存
// 上千个样本。
class GravityAccumulator {
public:
    void reset() {
        for (int i = 0; i < 3; ++i) {
            sum_[i] = 0.0;
            sum_sq_[i] = 0.0;
        }
        n_ = 0;
        rejected_ = 0;
    }

    // 返回 false 表示这次样本非法（NaN/Inf），已计入 rejected_ 而不是静默丢弃：
    // "标定用的样本里有多少是坏点"本身就是诊断信息
    bool add(double gx, double gy, double gz) {
        if (!std::isfinite(gx) || !std::isfinite(gy) || !std::isfinite(gz)) {
            ++rejected_;
            return false;
        }
        const double v[3] = {gx, gy, gz};
        for (int i = 0; i < 3; ++i) {
            sum_[i] += v[i];
            sum_sq_[i] += v[i] * v[i];
        }
        ++n_;
        return true;
    }

    std::size_t count() const { return n_; }
    std::size_t rejected() const { return rejected_; }

    // 三轴均值的模长。样本不足或模长非有限时返回 false
    bool mean_norm(double& out) const {
        if (n_ == 0) return false;
        double s = 0.0;
        for (int i = 0; i < 3; ++i) {
            const double m = sum_[i] / static_cast<double>(n_);
            s += m * m;
        }
        out = std::sqrt(s);
        return std::isfinite(out);
    }

    bool mean(double out[3]) const {
        if (n_ == 0) return false;
        for (int i = 0; i < 3; ++i) out[i] = sum_[i] / static_cast<double>(n_);
        return std::isfinite(out[0]) && std::isfinite(out[1]) && std::isfinite(out[2]);
    }

    // 三轴标准差里的最大值（m/s^2）。用总体方差而非样本方差：这里只关心
    // "抖不抖"，不追求无偏估计
    double max_std() const {
        if (n_ == 0) return 0.0;
        const double inv_n = 1.0 / static_cast<double>(n_);
        double worst = 0.0;
        for (int i = 0; i < 3; ++i) {
            const double m = sum_[i] * inv_n;
            const double var = sum_sq_[i] * inv_n - m * m;
            // 浮点误差可能让方差冒出一点负数，开方前夹到 0
            const double sd = std::sqrt(var > 0.0 ? var : 0.0);
            if (sd > worst) worst = sd;
        }
        return worst;
    }

    // 静止判据：样本够多，且三轴抖动都小于阈值。
    // 均值本身的模长不在这个函数里判断——车停在坡上时模长仍是 1 g，
    // 判不了对错，那是 build_level_rotation 的事
    bool stable(std::size_t min_samples, double max_std_mps2) const {
        return n_ >= min_samples && max_std() <= max_std_mps2;
    }

private:
    double sum_[3]{0.0, 0.0, 0.0};
    double sum_sq_[3]{0.0, 0.0, 0.0};
    std::size_t n_{0};
    std::size_t rejected_{0};
};

// 标定结果。r 为行主序 3x3，按 p_level = R · p_body 使用
struct LevelRotation {
    bool valid{false};
    double r[9]{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    double gravity_mps2{0.0};  // 实测重力模长，写进日志供台架核对
    double tilt_deg{0.0};      // 标定出的车身倾角（up 与车体 z 的夹角），诊断用
};

// 加速度计读数 -> 竖直向上方向。
//
// 静止时加速度计读的是**比力**，方向指向天，模长约 9.81 m/s^2（这是多数 IMU
// 与 sensor_msgs/Imu 的约定；也有整轴取反的驱动）。为了两种约定都能用，这里
// 不看符号，而是取与车体 z 同向的那一支：
//   up = ±a/|a|，符号选择使 up·(0,0,1) > 0
// 前提是车没有倒过来。夹角超过 max_tilt_deg 时判失败——那不是符号问题，是车
// 没摆正或标定时有人在推车，标出来的水平面没有意义。
//
// 返回 false 时 out 不被修改。
bool accel_to_up(const double a[3], double max_tilt_deg, double out[3]);

// 施密特正交化建水平系。
//   up            竖直向上方向。单位向量或加速度计量纲的向量都可以，只取方向
//   forward_hint  车头方向在车体系里的表示；只取它垂直于 up 的分量
//   min_up_norm   |up| 的下限，用来挡零向量与数值退化，取 0.5 即可。
//                 "到底有没有称到重力"不在这里判断——那是调用方拿
//                 GravityAccumulator::mean_norm 看的事，两者是不同的问题
//   min_forward   车头方向在水平面上投影长度的下限。车头几乎垂直向上或向下时
//                 这个投影趋于 0，x_L 的方向由噪声决定，必须判失败而不是给一个
//                 随机朝向的水平面
// 返回 true 时 out.valid = true 且 r 已填好；false 时 out.valid = false，
// r 保持单位阵
bool build_level_rotation(const double up[3], const double forward_hint[3],
                          double min_up_norm, double min_forward, LevelRotation& out);

// 把一个点旋进水平系：p_level = R · p_body。
// 用内联函数而不是 std::array 之类，是为了在点云循环里不引入额外拷贝
inline void apply_level_rotation(const LevelRotation& lr, double x, double y, double z,
                                 double& ox, double& oy, double& oz) {
    const double* r = lr.r;
    ox = r[0] * x + r[1] * y + r[2] * z;
    oy = r[3] * x + r[4] * y + r[5] * z;
    oz = r[6] * x + r[7] * y + r[8] * z;
}

}  // namespace suspension_preview_v2
