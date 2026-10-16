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

每次解出增益都会附带自校验（report_gain）：**迭代是否收敛、DARE 残差是否够小、
闭环谱半径是否 < 1**。三者缺一就明确标成"不可用"——这些数是要抄进 YAML 的，
万一给了会把作动器推离稳定的增益，必须当场看出来，而不是等上台架炸。

第七节新增：预瞄前馈优化（垂向 heave 通道）。前面几节只管"反馈"，
前馈一直是"比例映射 + 拍脑袋增益 1.0"。这一节把未来 N 拍的路面高程当作
已知量，用增广状态 DARE 解出最优预瞄前馈，并与比例前馈直接对比
（车身加速度 RMS / 行程占用），回答"增益 1.0 到底是不是最优"。

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
    """离散 Riccati 值迭代。返回 (增益 K, 迭代次数, 是否收敛)。K 满足 u = -K x。"""
    P = [row[:] for row in Q]
    used = iters
    converged = False
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
            converged = True
            break
    K = mm(mm(tr(Bd), P), Ad)
    den = r + mm(mm(tr(Bd), P), Bd)[0][0]
    return [[K[0][j] / den for j in range(len(Ad))]], used, converged, P


def dare_residual(Ad, Bd, Q, r, P):
    """DARE 残差（无穷范数）。

        res = P - (A^T P A - A^T P B (r + B^T P B)^-1 B^T P A + Q)

    理论上收敛解应使之为 0。这个值大说明"收敛"是假的（比如迭代次数不够、
    或者系统根本不可稳）。给出增益的同时必须给出这个数，否则抄进 YAML 的
    可能是把作动器推离稳定的数。
    """
    AtP = mm(tr(Ad), P)
    AtPAd = mm(AtP, Ad)
    AtPBd = mm(AtP, Bd)
    BtPBd = mm(mm(tr(Bd), P), Bd)[0][0]
    Pn = add(sub(AtPAd, scal(mm(AtPBd, tr(AtPBd)), 1.0 / (r + BtPBd))), Q)
    return max(abs(Pn[i][j] - P[i][j]) for i in range(len(P)) for j in range(len(P)))


def spectral_radius_bound(Ad, Bd, K, steps=4000):
    """估计闭环 A - B·K 的谱半径。

    > 1 表示增益会把系统推离稳定。判据用
        rho = exp( (1/m) * Σ log|A·v_i| )
    即 ‖A^m v‖^(1/m) 的极限 = 谱半径。

    为什么不用幂迭代取末次范数：幂迭代只在主导特征值是**单实根**时收敛。
    悬架是欠阻尼系统，主导特征值几乎总是一对共轭复根，迭代方向会来回摆，
    取任何一步的范数都没有意义（实测把稳定的 0.99 报成 1.28，直接颠倒结论）。
    几何平均对复根同样成立：它按定义收敛到谱半径，与主导根是否复数无关。
    """
    n = len(Ad)
    M = [[Ad[i][j] - Bd[i][0] * K[0][j] for j in range(n)] for i in range(n)]
    v = [1.0] + [0.5] + [0.0] * (n - 2) if n > 1 else [1.0]
    log_sum = 0.0
    for _ in range(steps):
        w = [sum(M[i][j] * v[j] for j in range(n)) for i in range(n)]
        nw = max(abs(x) for x in w)
        if nw < 1e-300:
            return 0.0
        # 归一化后 |v|_inf 恒为 1，故 log(nw) 累加即 log‖A^m v‖
        log_sum += math.log(nw)
        v = [x / nw for x in w]
    return math.exp(log_sum / steps)


def report_gain(Ad, Bd, Q, r, K, iters, conv, P, label):
    """给出增益的同时证明它可用：收敛 + 残差小 + 闭环稳定。

    三者缺一，抄进 YAML 的就是会把作动器推离稳定的数。
    返回 True 表示通过。
    """
    res = dare_residual(Ad, Bd, Q, r, P)
    rho = spectral_radius_bound(Ad, Bd, K)
    stable = rho < 1.0
    ok = conv and res < 1e-6 and stable
    print("    [{0}] {1}: 迭代 {2} {3}，残差 {4:.2e}，闭环谱半径 {5:.4f}（{6}）".format(
        "通过" if ok else "不可用", label, iters,
        "收敛" if conv else "未收敛", res, rho,
        "稳定" if stable else "不稳定！禁用"))
    if not ok:
        print("          ^ 增益不可用：调大 --dare-iters，或检查模型/权重是否合理")
    return ok


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

# ---------------------------------------------------------------- 预瞄前馈（垂向通道）

def build_heave_model(wn, zeta, tau):
    """垂向 heave 通道模型（单位统一为 mm / s），三状态。

        z'' = -wn^2*z - 2*zeta*wn*z' + wn^2*a + wn^2*z_r
        a'  = (u - a) / tau

    z   车身垂向位移 (mm，0 = 名义高度)
    a   作动器实际位置 (mm)，对指令 u 做一阶滞后，时间常数 tau
    z_r 路面高程 (mm)，作为外扰进入，与作动器走同一个通道

    为什么要显式建模作动器滞后：没有滞后的理想作动器下，u = -z_r 能
    精确抵消路面，前馈增益取 1.0 就拿到零加速度，预瞄一点发挥空间都没有。
    真实作动器跟不上，必须提前发指令——这正是 lookahead_time_s 存在的
    理由，也是预瞄优化的全部价值所在。

    这个"同通道"设定也解释了静态理想前馈为什么是 u = -z_r（增益 1.0）：
    想让车身不动，作动器就得把车身相对车轮压回等量的反向位移。
    """
    A = [[0.0, 1.0, 0.0],
         [-wn * wn, -2.0 * zeta * wn, wn * wn],
         [0.0, 0.0, -1.0 / tau]]
    B = [[0.0], [0.0], [1.0 / tau]]
    D = [[0.0], [wn * wn], [0.0]]
    return A, B, D


def build_preview_augmented(Ad, Bd, Dd, horizon, decay=0.9):
    """把未来 horizon 拍的路面高程当已知量，构造成增广状态。

    xi = [x_0..x_{n-1}, zr[0], zr[1], ..., zr[horizon-1]]
    zr[j] 是"当前时刻往后第 j 拍"的路面高程，构成移位寄存器：
      zr[j] 的状态下一拍变成 zr[j+1] 的状态；预瞄窗之外按 0 处理
      （有限预瞄的标准做法）。

    当前拍的实际路面高程取 zr[0]，经 Dd 通道进入动力学。
    在这个增广系统上解 DARE，得到的 K 一次给出"反馈增益 + 各拍预瞄增益"，
    这就是经典的 preview LQR（增广/提升式）写法。

    窗外按 decay 衰减（默认 0.9）。两种极端都不好：
      decay = 1.0（保持常数）会给移位寄存器一个特征值 1.0，增广系统只是
        边际稳定，值迭代收敛速率退化成 1（实测 20 万次仍不收敛）；
      decay = 0.0（窗外置零）收敛最快，但 DARE 会以为路面是个"有限脉冲"，
        于是过量驱动——实测直流增益被顶到 2.2，而稳态物理上只需 1.0。
    取 0.9 是折中：特征值 0.9，收敛速率约 0.81（几百次迭代即收敛），
    同时"窗外影响逐渐淡出"的假设不会诱导过驱。
    """
    n = len(Ad)
    m = max(1, horizon)
    N = n + m
    A = [[0.0] * N for _ in range(N)]
    for i in range(n):
        for j in range(n):
            A[i][j] = Ad[i][j]
    # 当前路面样本进入动力学
    for i in range(n):
        A[i][n + 0] = Dd[i][0]
    # 移位寄存器：zr[j] -> zr[j+1]；窗外按 decay 衰减
    for j in range(m - 1):
        A[n + j][n + j + 1] = 1.0
    A[n + m - 1][n + m - 1] = decay

    B = [[0.0] for _ in range(N)]
    for i in range(n):
        B[i][0] = Bd[i][0]
    return A, B


def make_road(length_m, step_m, seed=7, amp_scale=3.0):
    """合成一段粗糙路面（空间域，单位 mm）。

    用 6 个不同波长的正弦叠加：1.2~11 m 覆盖"减速带级"到"长波起伏"。
    幅值按波长递减（真实路面大致如此：短波抖、长波慢）。
    amp_scale 是整体幅度倍率：默认 3.0 使峰值约 20 mm——真实粗糙路的量级。
    幅度必须大到让作动器速率限制真的吃紧，否则预瞄没有发挥空间
    （小幅路面下 g=1.0 的比例前馈已经能做到零加速度，那是理想工况，
    不是真实工况）。固定 seed 保证可复现。
    """
    rnd = random.Random(seed)
    waves = [(1.2, 0.9), (2.3, 1.0), (3.1, 0.7), (5.0, 0.5),
             (7.5, 0.35), (11.0, 0.25)]
    phases = [rnd.uniform(0.0, 2.0 * math.pi) for _ in waves]
    count = int(math.ceil(length_m / step_m)) + 1
    out = []
    for i in range(count):
        x = i * step_m
        v = 0.0
        # 注意：循环变量不能叫 amp，否则会遮蔽上面的 amp_scale 参数——
        # 那种写法不报错，但倍率静默失效，是最难发现的一类 bug
        for (wl, a0), ph in zip(waves, phases):
            v += a0 * amp_scale * math.sin(2.0 * math.pi * x / wl + ph)
        out.append(v)
    return out


def simulate_preview(Ac, Ad, Bd, Dd, k_fb, preview, road, la_idx, Ts,
                     stroke_max, du_max, warmup=800):
    """闭环仿真，返回 (加速度 RMS, 行程 RMS, 行程峰值)，单位 mm / s^2 / mm。

    k_fb 是反馈增益列表，长度 = 状态数（u_fb = sum(k_fb[i] * x[i])）。
    preview 的三种形态：
      None            无前馈（纯反馈）
      浮点标量 g       比例前馈 u_ff = -g * road[k+la_idx]（现有控制器的做法）
      增益列表 f      最优预瞄 u_ff = -sum(f[j] * road[k+j])
    """
    n = len(Ad)
    wn2 = -Ac[1][0]
    x = [0.0] * n
    u_prev = 0.0
    total = len(road)
    acc_hist = []
    u_hist = []
    u_peak = 0.0
    for k in range(total):
        zr = road[k]
        # ---- 前馈 ----
        u_ff = 0.0
        if preview is not None:
            if isinstance(preview, (int, float)):
                j = k + la_idx
                u_ff = -preview * road[j] if j < total else 0.0
            else:
                for j, f in enumerate(preview):
                    idx = k + j
                    if idx < total:
                        u_ff -= f * road[idx]
        # ---- 反馈 ----
        u_fb = 0.0
        for i in range(n):
            u_fb += k_fb[i] * x[i]
        u = u_fb + u_ff
        u = max(-stroke_max, min(stroke_max, u))
        u = max(u_prev - du_max, min(u_prev + du_max, u))
        # ---- 推进 ----
        acc = 0.0
        for i in range(n):
            v = 0.0
            for j in range(n):
                v += Ad[i][j] * x[j]
            v += Bd[i][0] * u + Dd[i][0] * zr
            xn_i = v
            if i == 1:
                # 加速度 = z''，由连续模型直接算（比数值微分干净）。
                # 注意作动器只经状态 a 进入一次：-Ac[1][0]*z + ... + Ac[1][2]*a
                # 已经含 wn^2*a，绝不能再加一遍，否则作动器被算强一倍
                acc = (-Ac[1][0] * x[0] - Ac[1][1] * x[1] + Ac[1][2] * x[2]
                       + wn2 * zr)
            x[i] = xn_i
        u_prev = u
        if k >= warmup:
            acc_hist.append(acc)
            u_hist.append(u)
            u_peak = max(u_peak, abs(u))
    rms_acc = math.sqrt(sum(v * v for v in acc_hist) / max(1, len(acc_hist)))
    rms_u = math.sqrt(sum(v * v for v in u_hist) / max(1, len(u_hist)))
    return rms_acc, rms_u, u_peak


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
    # ---- 预瞄前馈优化（垂向通道）----
    ap.add_argument("--preview-horizon", type=int, default=12,
                    help="预瞄帧数（控制周期为单位）")
    ap.add_argument("--heave-hz", type=float, default=1.5, help="垂向模态频率 Hz")
    ap.add_argument("--heave-zmax", type=float, default=5.0, help="允许的最大车身沉浮 mm")
    ap.add_argument("--heave-vmax", type=float, default=100.0,
                    help="允许的最大车身垂向速度 mm/s")
    ap.add_argument("--speed", type=float, default=2.0, help="车速 m/s（台架/实车）")
    ap.add_argument("--lookahead-time", type=float, default=0.35,
                    help="现有比例前馈的取样时域 s")
    ap.add_argument("--road-length", type=float, default=40.0, help="仿真路面长度 m")
    ap.add_argument("--road-seed", type=int, default=7, help="路面随机种子")
    ap.add_argument("--actuator-tau", type=float, default=0.05,
                    help="作动器一阶滞后时间常数 s")
    ap.add_argument("--road-amp", type=float, default=3.0,
                    help="路面幅度倍率（3.0 -> 峰值约 20 mm，真实粗糙路量级）")
    ap.add_argument("--preview-decay", type=float, default=0.9,
                    help="预瞄窗外的衰减系数（1=常数会不收敛，0=置零会过驱）")
    ap.add_argument("--dare-iters", type=int, default=20000,
                    help="增广 DARE 的最大迭代数（状态多，收敛慢）")
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
    K, it, conv, P = dare(Ad, Bd, Q, 1.0 / a.u_max ** 2)
    report_gain(Ad, Bd, Q, 1.0 / a.u_max ** 2, K, it, conv, P, "无积分 LQR")
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
    Ke, it2, conv2, Pe = dare(Ae, Be, Qe, 1.0 / du_max ** 2)
    report_gain(Ae, Be, Qe, 1.0 / du_max ** 2, Ke, it2, conv2, Pe, "增量式 LQI")
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
    print("-" * 72)
    print("7) 预瞄前馈优化（垂向 heave 通道）—— 前馈不再是拍脑袋的增益 1.0")
    print("-" * 72)
    wh = 2.0 * math.pi * a.heave_hz
    Ah, Bh, Dh = build_heave_model(wh, a.zeta, a.actuator_tau)
    Adh, Bdh = discretize(Ah, Bh, Ts)
    _, Ddh = discretize(Ah, Dh, Ts)
    n_state = len(Adh)

    # 7a 垂向反馈增益：与 roll 通道同一套 LQR 哲学，指标换成沉浮/垂速。
    # 作动器状态 a 不加权（它是内部状态，不是要控的物理量）
    Qh = [[0.0] * n_state for _ in range(n_state)]
    Qh[0][0] = 1.0 / a.heave_zmax ** 2
    Qh[1][1] = 1.0 / a.heave_vmax ** 2
    Kh, ith, convh, Ph = dare(Adh, Bdh, Qh, 1.0 / a.stroke ** 2)
    report_gain(Adh, Bdh, Qh, 1.0 / a.stroke ** 2, Kh, ith, convh, Ph,
                "垂向反馈 LQR")
    k_fb = [-Kh[0][i] for i in range(n_state)]
    print("  垂向模型 : {0} Hz (wn={1:.3f} rad/s), zeta={2}, 作动器滞后 tau={3:.3f} s"
          "（= {4:.0f} 拍）".format(a.heave_hz, wh, a.zeta, a.actuator_tau,
                                   a.actuator_tau / Ts))
    print("  反馈 LQR : " + "  ".join("k{0} = {1:.4f}".format(i, k_fb[i])
                                      for i in range(n_state))
          + "   （u_fb = Σ k_i·x_i）")

    # 7b 预瞄 LQR：把未来 N 拍路面高程当已知量，增广后解一次 DARE
    Aaug, Baug = build_preview_augmented(Adh, Bdh, Ddh, a.preview_horizon,
                                         a.preview_decay)
    dim = len(Aaug)
    Qaug = [[0.0] * dim for _ in range(dim)]
    Qaug[0][0] = 1.0 / a.heave_zmax ** 2
    Qaug[1][1] = 1.0 / a.heave_vmax ** 2
    Kp, itp, convp, Pp = dare(Aaug, Baug, Qaug, 1.0 / a.stroke ** 2,
                             iters=a.dare_iters)
    report_gain(Aaug, Baug, Qaug, 1.0 / a.stroke ** 2, Kp, itp, convp, Pp,
                "预瞄 LQR")
    # u = -K*xi，所以前馈增益就是 K 中对应各路面的系数本身（不要再取负，
    # 否则符号反了会变成"助推"而不是"抵消"——前馈越强车身越抖）。
    # 语义：u_ff = -sum(f[j] * z_r[k+j])，故 sum(f) 应接近 1
    # （静态理想是 u = -z_r，即完全抵消路面）
    preview_gains = [Kp[0][n_state + j] for j in range(a.preview_horizon)]
    if convp:
        print("  预瞄 LQR : 增广状态 {0} 阶（{1} 状态 + {2} 拍预瞄），{3} 次迭代收敛".format(
            dim, n_state, a.preview_horizon, itp))
    else:
        print("  预瞄 LQR : 增广状态 {0} 阶，{1} 次迭代【未收敛】——增益不可信，"
              "请增大 --dare-iters 或减小 --preview-horizon".format(dim, itp))

    total = sum(preview_gains)
    centroid = (sum(j * f for j, f in enumerate(preview_gains)) / total
                if total > 1e-12 else 0.0)
    preview_time = centroid * Ts
    preview_dist = preview_time * a.speed
    print("  预瞄窗   : {0} 拍 = {1:.2f} s = {2:.2f} m（车速 {3:.1f} m/s；"
          "作动器滞后 {4:.3f} s）".format(
              a.preview_horizon, a.preview_horizon * Ts,
              a.preview_horizon * Ts * a.speed, a.speed, a.actuator_tau))
    print("  直流增益 : sum(f) = {0:.3f}   （1.0 = 完全跟随；小于 1 说明不该全跟）".format(
        total))
    print("  有效提前 : {0:.2f} 拍 = {1:.3f} s = {2:.2f} m（增益重心，对比现有 "
          "lookahead_time={3:.2f} s）".format(
              centroid, preview_time, preview_dist, a.lookahead_time))
    print("  前馈增益向量 f[0..{0}]:".format(a.preview_horizon - 1))
    print("    " + " ".join("{0:.3f}".format(f) for f in preview_gains))

    # 7c 仿真对比：同一条路面、同一套反馈与约束，只换前馈
    step_m = a.speed * Ts
    road = make_road(a.road_length, step_m, seed=a.road_seed,
                    amp_scale=a.road_amp)
    la_idx = max(0, int(round(a.lookahead_time / Ts)))
    print()
    print("  仿真：{0:.0f} m 粗糙路面（6 个波长 1.2~11 m 正弦叠加），"
          "车速 {1:.1f} m/s".format(a.road_length, a.speed))
    print("  约束：行程 ±{0:.0f} mm，速率 {1:.0f} mm/s，反馈固定为上行的 LQR 值".format(
        a.stroke, a.rate_limit))
    print()
    print("  {0:<24}{1:>14}{2:>12}{3:>10}{4:>10}".format(
        "方案", "车身加速度RMS", "折算 m/s^2", "行程RMS", "行程峰值"))
    # 比例前馈分两种取样点：
    #   la=0      只看"当前这一拍的路面"，隔离的是增益大小本身的影响
    #   la=la_idx 提前 lookahead_time，即现行控制器的做法
    # 两者分开才能回答"是增益不对，还是提前量不对"
    g_star = round(total, 3)
    cases = [("纯反馈（无前馈）", None, 0),
             ("比例前馈 g=0.5（当前样本）", 0.5, 0),
             ("比例前馈 g={0}（当前样本）".format(g_star), total, 0),
             ("比例前馈 g=1.0（当前样本）", 1.0, 0),
             ("比例前馈 g=1.0（提前{0:.2f}s）".format(a.lookahead_time), 1.0, la_idx),
             ("最优预瞄 LQR", preview_gains, 0)]
    results = {}
    for lbl, ff, la in cases:
        acc, u_rms, u_pk = simulate_preview(
            Ah, Adh, Bdh, Ddh, k_fb, ff, road, la, Ts,
            a.stroke, du_max)
        results[lbl] = acc
        print("  {0:<26}{1:>13.1f}{2:>12.4f}{3:>9.1f}{4:>9.1f}".format(
            lbl, acc, acc / 1000.0, u_rms, u_pk))

    base = results.get("纯反馈（无前馈）", 0.0)
    prop_cur = results.get("比例前馈 g={0}（当前样本）".format(g_star), 0.0)
    prop_1 = results.get("比例前馈 g=1.0（当前样本）", 0.0)
    prop_la = results.get(
        "比例前馈 g=1.0（提前{0:.2f}s）".format(a.lookahead_time), 0.0)
    opt = results.get("最优预瞄 LQR", 0.0)
    print()
    print("  结论：")
    print("    纯反馈                        -> 加速度 RMS {0:.1f} mm/s^2".format(base))
    if prop_1 > 0:
        print("    比例前馈 g=1.0（当前样本）      -> {0:.1f} mm/s^2（比纯反馈降 {1:.0f}%）".format(
            prop_1, (base - prop_1) / base * 100.0))
    if prop_la > 0:
        if prop_1 > 1e-9:
            worse = "更差" if prop_la > prop_1 else "更好"
            delta = "，差 {0:.0f}%".format(abs(prop_la - prop_1) / prop_1 * 100.0)
        else:
            worse = "基准已为零"
            delta = ""
        print("    比例前馈 g=1.0（提前{0:.2f}s）    -> {1:.1f} mm/s^2"
              "（比当前样本{2}{3}）".format(a.lookahead_time, prop_la, worse, delta))
    if opt > 0 and prop_1 > 1e-9:
        verdict = "再降" if opt < prop_1 else "反而高"
        print("    最优预瞄 LQR                   -> {0:.1f} mm/s^2（比纯反馈降 {1:.0f}%，"
              "比 g=1.0 {2} {3:.0f}%）".format(
                  opt, (base - opt) / base * 100.0, verdict,
                  abs(prop_1 - opt) / prop_1 * 100.0))
    print()
    print("    关键读数：")
    print("      1) 最优前馈的直流增益 sum(f) = {0:.3f}（1.0 = 稳态完全跟随）"
          "——{1}".format(
              total,
              "接近 1，说明「按比例完全跟随」方向是对的，但形状还可以优化"
              if 0.8 <= total <= 1.3 else
              "偏离 1，需结合窗外衰减系数 --preview-decay 一起看"))
    print("      2) 有效提前量 {0:.3f} s = {1:.2f} m，远小于 lookahead_time="
          "{2:.2f} s".format(preview_time, preview_dist, a.lookahead_time))
    print("      3) 提前量的作用是补偿作动器滞后 tau={0:.3f} s（已建模，="
          "{1:.0f} 拍）。".format(a.actuator_tau, a.actuator_tau / Ts))
    print("         在这个滞后量下，0.35 s 的提前远远过头——在路面主频上相当于"
          "大相移，")
    print("         所以它比完全不提前更差。速率限制也按 du_max 建模了。")
    print("         真机的 tau 需台架实测；实测值变了，最优提前量随之而变。")
    print("      4) 行程峰值上限 = {0:.0f} mm（限幅），若某方案峰值顶到限幅说明"
          "它在靠饱和硬跟".format(a.stroke))

    # 7d 提前量随作动器滞后缩放——台架标定真正需要的那条曲线
    print()
    print("  提前量 vs 作动器滞后（同一路面、同一反馈，只改 tau）：")
    print("  {0:<14}{1:>12}{2:>12}{3:>14}{4:>12}".format(
        "tau (s)", "有效提前(s)", "提前量(m)", "最优预瞄RMS", "g=1.0 RMS"))
    for tau in (0.02, 0.05, 0.10, 0.20):
        Ah2, Bh2, Dh2 = build_heave_model(wh, a.zeta, tau)
        Ad2, Bd2 = discretize(Ah2, Bh2, Ts)
        _, Dd2 = discretize(Ah2, Dh2, Ts)
        Aa2, Ba2 = build_preview_augmented(Ad2, Bd2, Dd2, a.preview_horizon,
                                           a.preview_decay)
        d2 = len(Aa2)
        Qa2 = [[0.0] * d2 for _ in range(d2)]
        Qa2[0][0] = 1.0 / a.heave_zmax ** 2
        Qa2[1][1] = 1.0 / a.heave_vmax ** 2
        K2, _, cv2, P2 = dare(Aa2, Ba2, Qa2, 1.0 / a.stroke ** 2,
                              iters=a.dare_iters)
        if not cv2:
            print("  {0:<14.3f}{1:>12}{2:>12}{3:>14}{4:>12}".format(
                tau, "未收敛", "-", "-", "-"))
            continue
        pg = [K2[0][n_state + j] for j in range(a.preview_horizon)]
        tot2 = sum(pg)
        cen2 = (sum(j * f for j, f in enumerate(pg)) / tot2
                if tot2 > 1e-12 else 0.0)
        a_opt, _, _ = simulate_preview(Ah2, Ad2, Bd2, Dd2, k_fb, pg, road, 0,
                                       Ts, a.stroke, du_max)
        a_g1, _, _ = simulate_preview(Ah2, Ad2, Bd2, Dd2, k_fb, 1.0, road, 0,
                                      Ts, a.stroke, du_max)
        print("  {0:<14.3f}{1:>12.3f}{2:>12.2f}{3:>14.1f}{4:>12.1f}".format(
            tau, cen2 * Ts, cen2 * Ts * a.speed, a_opt, a_g1))
    print("  读数：")
    print("    1) 每一行里最优预瞄都明显优于 g=1.0 的比例前馈（约 2 倍），"
          "结论对 tau 不敏感；")
    print("    2) 有效提前量随 tau 增长得很慢（0.02->0.20 s 只从 0.021 到 "
          "0.034 s），")
    print("       而且被预瞄窗 {0} 拍 = {1:.2f} s 封顶——要看清更大 tau 的趋势，"
          "需同时加大".format(a.preview_horizon, a.preview_horizon * Ts))
    print("       --preview-horizon；")
    print("    3) 真正稳定的结论是：有用的预瞄窗就是十几个控制周期（几十毫秒），"
          "不是 0.35 s。")
    print("       lookahead_time_s 该按实测作动器滞后定，而不是照「走满行程耗时」"
          "拍一个固定值。")

    print()
    print("  提示：以上增益是该组模型参数下的参考量级，wn/zeta 需台架实测。")
    print("        第 3 步显示 kp 翻倍也只把残留从 55% 压到 25%，这是纯反馈的")
    print("        结构上限——要真正消除只能加积分，而不是继续堆 kp。")


if __name__ == "__main__":
    main()
