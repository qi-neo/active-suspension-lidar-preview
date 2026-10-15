#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""v3 控制器增益离线设计工具（只用标准库，不需要 numpy/scipy）。

把"试凑 kp/kd"换成"按性能指标反解最优增益"：给定允许的最大 roll 偏差、
最大角速度、最大行程，由 DARE 解出相对该组指标最优的反馈增益。

用法：
    python3 lqr_gain_design.py
    python3 lqr_gain_design.py --wn 1.8 --zeta 0.3 --theta-max 0.04

支持三种产出品：
    1) 无积分 LQR   -> kp / kd，直接等价于现有 PD 的写法
    2) 增量式 LQI   -> kx1 / kx2 / ki，能补掉常值横坡的稳态残留
    3) 稳态残留解析 -> theta_ss / z_road = 1 / (1 + 2*kp/轮距)

注意：这是二维近似模型（一个二阶 secretory 模态 + 轮距几何），
wn/zeta 需要台架实测；本脚本给出的是"合理量级"，不是最终标定值。
"""
import argparse
import math
import random

# ---------------------------------------------------------------- 小矩阵运算

def mm(A, B):
    n, m, k = len(A), len(B[0]), len(B)
    return [[sum(A[i][t] * B[t][j] for t in range(k)) for j in range(m)]
            for i in range(n)]

def tr(A):
    return [list(r) for r in zip(*A)]

def sub(A, B):
    return [[A[i][j] - B[i][j] for j in range(len(A[0]))] for i in range(len(A))]

def eye(n):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]

def scal(A, s):
    return [[A[i][j] * s for j in range(len(A[0]))] for i in range(len(A))]

def add(A, B):
    return [[A[i][j] + B[i][j] for j in range(len(A[0]))] for i in range(len(A))]

def expm_taylor(M, order=14):
    R = eye(len(M))
    term = eye(len(M))
    for k in range(1, order + 1):
        term = mm(scal(term, 1.0 / k), M)
        R = add(R, term)
    return R

def discretize(A, B_col, Ts):
    """Ad = expm(A*Ts)；Bd = integral_0^T expm(A*s) ds * B"""
    n = len(A)
    Ad = expm_taylor(scal(A, Ts))
    acc = scal(eye(n), Ts)
    term = scal(eye(n), Ts)
    for k in range(1, 14):
        term = mm(scal(term, Ts / (k + 1)), A)
        acc = add(acc, term)
    return Ad, mm(acc, B_col)

def dare(Ad, Bd, Q, r, iters=300000, tol=1e-13):
    """离散 Riccati 值迭代。返回 (增益 K, 迭代次数)。K 满足 u = -K x。"""
    P = [row[:] for row in Q]
    used = iters
    for it in range(iters):
        AtPAd = mm(mm(tr(Ad), P), Ad)
        AtPBd = mm(mm(tr(Ad), P), Bd)
        BtPBd = mm(mm(tr(Bd), P), Bd)[0][0]
        denom = r + BtPBd
        Pn = add(sub(AtPAd, scal(mm(AtPBd, tr(AtPBd)), 1.0 / denom)), Q)
        delta = max(abs(Pn[i][j] - P[i][j])
                    for i in range(len(P)) for j in range(len(P)))
        P = Pn
        if delta < tol:
            used = it
            break
    K = mm(mm(tr(Bd), P), Ad)
    den = r + mm(mm(tr(Bd), P), Bd)[0][0]
    return [[K[0][j] / den for j in range(len(Ad))]], used

# ---------------------------------------------------------------- modelling

def build_model(wn, zeta, wheelbase_mm):
    """roll 通道二阶模型。

    theta'' = -wn^2*theta - 2*zeta*wn*theta' - (2*wn^2/wb) * u
    u 是单侧行程修正量(mm)：右侧加 u、左侧减 u，故左右行程差为 2u。
    """
    A = [[0.0, 1.0], [-wn * wn, -2.0 * zeta * wn]]
    B = [[0.0], [-2.0 * wn * wn / wheelbase_mm]]
    D = [[0.0], [wn * wn]]          # 路面 roll 扰动进入的通道
    return A, B, D

# ---------------------------------------------------------------- 评估

def simulate(Ad, Bd, Dd, kind, gains, zr_const, noise,
             stroke_max, du_max, steps=2000, seed=11):
    rnd = random.Random(seed)
    x = [0.0, 0.0]
    u_prev = 0.0
    x_prev = [0.0, 0.0]
    hist = []
    for k in range(steps):
        zr = zr_const + (rnd.uniform(-noise, noise) if noise > 0 else 0.0)
        if kind == "lqi":
            kx1, kx2, ki = gains
            u = u_prev + (kx1 * (x[0] - x_prev[0]) +
                          kx2 * (x[1] - x_prev[1]) +
                          ki * x[0])
        else:
            kp, kd = gains
            u = kp * x[0] + kd * x[1]
        u = max(-stroke_max, min(stroke_max, u))
        u = max(u_prev - du_max, min(u_prev + du_max, u))
        xn = [Ad[i][0] * x[0] + Ad[i][1] * x[1] + Bd[i][0] * u + Dd[i][0] * zr
              for i in (0, 1)]
        x_prev = x[:]
        u_prev = u
        x = xn
        if k > steps - 500:
            hist.append(x[0])
    rms = math.sqrt(sum(v * v for v in hist) / max(1, len(hist)))
    return x[0], rms

# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description="v3 控制器增益离线设计")
    ap.add_argument("--wn", type=float, default=1.5, help="悬架模态频率 Hz")
    ap.add_argument("--zeta", type=float, default=0.25, help="阻尼比")
    ap.add_argument("--wheelbase", type=float, default=600.0, help="轮距 mm")
    ap.add_argument("--rate", type=float, default=200.0, help="控制环频率 Hz")
    ap.add_argument("--stroke", type=float, default=40.0, help="行程限幅 mm")
    ap.add_argument("--rate-limit", type=float, default=300.0,
                    help="作动器速率上限 mm/s")
    ap.add_argument("--theta-max", type=float, default=0.05,
                    help="允许的最大 roll 偏差 rad")
    ap.add_argument("--omega-max", type=float, default=0.20,
                    help="允许的最大角速度 rad/s")
    ap.add_argument("--u-max", type=float, default=40.0, help="允许的最大行程 mm")
    a = ap.parse_args()

    Ts = 1.0 / a.rate
    du_max = a.rate_limit * Ts
    wn = 2.0 * math.pi * a.wn

    A, B, D = build_model(wn, a.zeta, a.wheelbase)
    Ad, Bd = discretize(A, B, Ts)
    _, Dd = discretize(A, D, Ts)

    Q = [[1.0 / a.theta_max ** 2, 0.0], [0.0, 1.0 / a.omega_max ** 2]]

    print("=" * 72)
    print("v3 控制器增益离线设计")
    print("=" * 72)
    print("  悬架模型 : {0} Hz (wn={1:.3f} rad/s), zeta={2}".format(
        a.wn, wn, a.zeta))
    print("  轮距     : {0:.0f} mm    控制环: {1:.0f} Hz (Ts={2:.4f} s)".format(
        a.wheelbase, a.rate, Ts))
    print("  约束     : 行程 ±{0:.0f} mm, 速率 {1:.0f} mm/s (每拍 Δu ≤ {2:.2f} mm)".format(
        a.stroke, a.rate_limit, du_max))
    print("  指标     : theta_max={0} rad, omega_max={1} rad/s, u_max={2} mm".format(
        a.theta_max, a.omega_max, a.u_max))

    print()
    print("-" * 72)
    print("1) 无积分 LQR —— 与现有 PD 结构完全同构，只是增益来源不同")
    print("-" * 72)
    K, it = dare(Ad, Bd, Q, 1.0 / a.u_max ** 2)
    kp, kd = -K[0][0], -K[0][1]
    print("  kp = {0:.1f} mm/rad        kd = {1:.1f} mm/(rad/s)".format(kp, kd))
    print("  ({0} 次迭代收敛)".format(it))

    print()
    print("-" * 72)
    print("2) 增量式 LQI —— 在上一步基础上加积分，补掉常值横坡残留")
    print("-" * 72)
    Ae = [[Ad[0][0], Ad[0][1], 0.0],
          [Ad[1][0], Ad[1][1], 0.0],
          [Ad[0][0], Ad[0][1], 1.0]]
    Be = [[Bd[0][0]], [Bd[1][0]], [Bd[0][0]]]
    Qe = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 1.0 / a.theta_max ** 2]]
    Ke, it2 = dare(Ae, Be, Qe, 1.0 / du_max ** 2)
    kx1, kx2, ki = -Ke[0][0], -Ke[0][1], -Ke[0][2]
    print("  kx1 = {0:.1f}   kx2 = {1:.1f}   ki = {2:.2f}".format(kx1, kx2, ki))
    print("  (注意 R 必须按 Δu 的量级取，照搬 u_max 会让 R 小两个数量级而失稳)")

    print()
    print("-" * 72)
    print("3) 常值横坡稳态残留 —— 纯反馈压不完，必须靠积分")
    print("-" * 72)
    print("  解析式: theta_ss / z_road = 1 / (1 + 2*kp/轮距)")
    for name, kpv in [("现有 PD(kp=250)", 250.0),
                      ("无积分 LQR", kp),
                      ("kp 翻倍", kp * 2.0)]:
        ss = 1.0 / (1.0 + 2.0 * kpv / a.wheelbase)
        print("    {0:<20} kp={1:>7.1f}  ->  残留 {2:>6.2f} %".format(
            name, kpv, ss * 100.0))

    print()
    print("-" * 72)
    print("4) 带约束仿真（常值横坡 0.05 rad；随机路面 ±0.04 rad）")
    print("-" * 72)
    print("  {0:<22}{1:>16}{2:>16}".format("方案", "横坡残留", "随机路面 RMS"))
    for lbl, kind, g in [("现有 PD 250/30", "pd", (250.0, 30.0)),
                         ("无积分 LQR", "pd", (kp, kd)),
                         ("增量式 LQI", "lqi", (kx1, kx2, ki))]:
        ss, _ = simulate(Ad, Bd, Dd, kind, g, 0.05, 0.0, a.stroke, du_max)
        _, rms = simulate(Ad, Bd, Dd, kind, g, 0.0, 0.04, a.stroke, du_max)
        print("  {0:<22}{1:>15.2f} %{2:>15.5f} rad".format(
            lbl, ss / 0.05 * 100.0, rms))

    print()
    print("-" * 72)
    print("5) 预瞄时间窗 —— lookahead 必须随车速放大，否则前馈来不及")
    print("-" * 72)
    t_full = a.stroke / a.rate_limit
    v_crit = 1.0 / t_full
    print("  走满行程最短耗时 = {0:.0f} mm / {1:.0f} mm/s = {2:.3f} s".format(
        a.stroke, a.rate_limit, t_full))
    print("  lookahead 固定 1.0 m 时，临界车速 = {0:.1f} m/s ({1:.0f} km/h)".format(
        v_crit, v_crit * 3.6))
    print("  建议 lookahead = clamp(v * {0:.1f}, 0.5, {1:.0f}) m".format(
        t_full * 2.5, 5.0))
    print("  实际取样位置还要加 车速×帧龄：剖面 10~20 Hz、帧龄常年 50~100 ms，")
    print("  不补偿的话 2 m/s 时取样点就落后 100~200 mm（控制器已实现该补偿）")

    print()
    print("-" * 72)
    print("6) 落地参数 —— 与控制节点 config/control_params.yaml 一一对应")
    print("-" * 72)
    print("  use_speed_adaptive_lookahead: true")
    print("  speed_topic: \"/odom\"          # 车速来源；台架无里程计时用 assume_speed_mps")
    print("  speed_timeout_s: 0.5            # 车速过期后退回固定 lookahead_m")
    print("  assume_speed_mps: 0.0           # >0 按此恒定车速前馈（仅限台架）")
    print("  lookahead_time_s: 0.35          # 出厂默认；纯按 2.5× 算是 {0:.2f}，可台架标定".format(
        t_full * 2.5))
    print("  min_lookahead_m: 0.5")
    print("  max_lookahead_m: 5.0")
    print("  wheelbase_m: 0.25               # 轴距需实测；后轴取样 = 前馈距离 - 轴距")
    print()
    print("  说明：第 5 步的临界车速是\"固定 lookahead 还能追上路面\"的上限。")
    print("  论文实验车速只有 0.5 m/s，固定 1.0 m 时前馈提前了约 2 s 取路面，")
    print("  相位完全对不上——这就是 v3.2.0 把取样距离改成车速自适应的原因。")
    print()
    print("  提示：以上增益是该组模型参数下的参考量级，wn/zeta 需台架实测。")
    print("        第 3 步显示 kp 翻倍也只把残留从 55% 压到 25%，这是纯反馈的")
    print("        结构上限——要真正消除只能加积分，而不是继续堆 kp。")


if __name__ == "__main__":
    main()
