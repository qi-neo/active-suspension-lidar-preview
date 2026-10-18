"""由试验记录生成 README 配图。

用法：
    python make_figures.py --data ../试验记录 --out .

数据列：Time(s) / Mode / Roll(deg) / Pitch(deg) / Acc_Z(g) / Gyro_Roll / Gyro_Pitch，
原始采样 250 Hz。试验记录未随仓库发布，需要时向作者索取。

处理：去掉首尾 0.5 s 过渡段 → 去均值 → 36 ms 滑动平均。图内不标百分比，
改善率以 README「实测结果」表格口径为准。
"""
import argparse
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd

PASSIVE = "#9aa0a6"
ACTIVE = "#1f6fb2"

plt.rcParams.update({
    "font.sans-serif": ["Heiti SC", "Hiragino Sans GB", "Arial Unicode MS"],
    "axes.unicode_minus": False,
    "font.size": 9,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "axes.edgecolor": "#c8ccd2",
    "axes.grid": True,
    "grid.color": "#dfe3e8",
    "grid.linewidth": 0.6,
    "grid.linestyle": (0, (3, 3)),
    "figure.dpi": 200,
    "savefig.dpi": 200,
})


def load(data_dir, name, smooth=9):
    df = pd.read_excel(os.path.join(data_dir, name + ".xlsx")).dropna(subset=["Time(s)"])
    t = df["Time(s)"].to_numpy()
    df = df[(t > 0.5) & (t < t.max() - 0.5)].reset_index(drop=True)
    out = {"t": df["Time(s)"].to_numpy()}
    out["roll"] = df["Roll(deg)"]
    out["pitch"] = df["Pitch(deg)"]
    out["acc"] = df["Acc_Z(g)"] / 9.80665  # 该列实际单位是 m/s²
    for k in ("roll", "pitch", "acc"):
        out[k] = (out[k] - out[k].mean()).rolling(smooth, center=True, min_periods=1).mean()
    return out


def attitude(data_dir, out_dir):
    p, a = load(data_dir, "被动悬架01"), load(data_dir, "主动悬架01")
    fig, axes = plt.subplots(2, 1, figsize=(7.0, 4.6), sharex=True, gridspec_kw={"hspace": 0.18})
    for ax, key, label in ((axes[0], "roll", "横滚角 / deg"), (axes[1], "pitch", "俯仰角 / deg")):
        ax.plot(p["t"], p[key], color=PASSIVE, lw=0.8, label="被动悬架")
        ax.plot(a["t"], a[key], color=ACTIVE, lw=0.8, label="主动悬架")
        ax.set_ylabel(label)
    axes[0].legend(loc="upper right", frameon=False, fontsize=8, ncol=2,
                   handlelength=1.4, columnspacing=1.2)
    axes[1].set_xlabel("时间 / s")
    axes[1].set_xlim(0, 15)
    fig.subplots_adjust(left=0.1, right=0.98, top=0.96, bottom=0.12)
    fig.savefig(os.path.join(out_dir, "attitude_compare.png"), bbox_inches="tight")
    plt.close(fig)


def obstacle(data_dir, out_dir):
    p, a = load(data_dir, "被动悬架03"), load(data_dir, "主动悬架03")
    fig, ax = plt.subplots(figsize=(7.0, 2.9))
    ax.plot(p["t"], p["pitch"], color=PASSIVE, lw=0.9, label="被动悬架")
    ax.plot(a["t"], a["pitch"], color=ACTIVE, lw=0.9, label="主动悬架")
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("俯仰角 / deg")
    ax.set_xlim(0, 27)
    ax.legend(loc="upper right", frameon=False, fontsize=8, ncol=2,
              handlelength=1.4, columnspacing=1.2)
    fig.subplots_adjust(left=0.1, right=0.98, top=0.94, bottom=0.2)
    fig.savefig(os.path.join(out_dir, "obstacle_pitch.png"), bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="../试验记录", help="试验记录 xlsx 目录")
    ap.add_argument("--out", default=".", help="图片输出目录")
    args = ap.parse_args()
    attitude(args.data, args.out)
    obstacle(args.data, args.out)
    print("done:", args.out)
