"""由试验记录生成 README 配图（极简风格）。

用法：
    pip install matplotlib pandas openpyxl fonttools
    python make_figures.py --data ../试验记录 --out .

依赖说明：fonttools 只用于从 macOS 的 PingFang.ttc 中抽出「苹方-简」的
Regular/Medium 两个面（matplotlib 默认只认 ttc 的第一个面）；缺失时脚本会
降级到后备中文字体并提示，不会中断出图。

数据列：Time(s) / Mode / Roll(deg) / Pitch(deg) / Acc_Z(g) / Gyro_Roll / Gyro_Pitch，
原始采样 250 Hz。试验记录未随仓库发布，需要时向作者索取。

处理：去掉首尾 0.5 s 过渡段 → 去均值 → 36 ms 滑动平均。图内不标百分比，
改善率以 README「实测结果」表格口径为准。

设计约定（改风格只动下面这组常量）：
  - 字体：苹方（PingFang SC）。macOS 上 matplotlib 只把 ttc 的第一个面
    （PingFang HK）注册进来，所以这里会把「苹方-简」的 Regular/Medium 两个面
    抽到临时目录再注册；没有 fontTools / 不是 macOS 时按后备字体链降级，
    并在 stderr 提示，不会让出图中断。
  - 白底、无上/右边框、只保留极淡的横向网格；被动灰、主动深墨蓝，主动线略粗。
  - 主动线加一层很淡的波动带。口径要说准：单次采集没有重复样本，且曲线本身
    已经是 36 ms 平滑后的均值（相对它的残差 σ 只有 0.01° 量级，画出来不足 1 px）。
    所以这里定义成"原始信号相对 1 s 趋势线的局部 ±1σ"——它描述的是 1 s 尺度上的
    观测波动幅度（0.1–0.3° 量级），**不是**统计置信区间；图注里也按这个说法写。
  - 300 dpi 输出，README 里按 660–720 px 宽度显示，视网膜屏下依然清晰。
"""
from __future__ import annotations

import argparse
import glob
import os
import pathlib
import sys
import tempfile

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import pandas as pd
from matplotlib import font_manager
from matplotlib.patches import Patch
from matplotlib.ticker import MaxNLocator

# ---------------------------------------------------------------- 设计常量
PASSIVE = "#B6BDC7"        # 被动：冷灰
ACTIVE = "#2C5F8A"         # 主动：深墨蓝
INK = "#1F2328"            # 正文
MUTED = "#6B7280"          # 刻度标签
GRID = "#F1F3F6"           # 横向网格
AXIS = "#E5E7EB"           # 轴线
ZERO = "#D0D7DE"           # 零线

LW_PASSIVE = 0.9
LW_ACTIVE = 1.15
FIG_W = 7.0
DPI = 300

SAMPLE_HZ = 250        # 试验记录采样率

BAND_ALPHA = 0.12      # 波动带透明度（要"很淡"，别压过主曲线）
BAND_SIGMA = 1.0       # ±1σ
BAND_BASE_S = 1.0      # 趋势基线窗：相对 1 s 趋势线取残差
BAND_WINDOW_S = 1.0    # 残差局部 σ 的窗（1 s）
BAND_SMOOTH = 21       # σ 曲线再做一次平滑，避免锯齿

_PINGFANG_POSTSCRIPT = ("PingFangSC-Regular", "PingFangSC-Medium")
_FALLBACK_FAMILIES = ("Hiragino Sans GB", "Noto Sans CJK SC", "Heiti SC",
                      "Source Han Sans SC", "Arial Unicode MS")


# ---------------------------------------------------------------- 字体解析
def _installed_family(name: str) -> bool:
    return any(f.name == name for f in font_manager.fontManager.ttflist)


def _find_pingfang_ttc() -> str | None:
    patterns = (
        "/System/Library/Fonts/PingFang.ttc",
        "/System/Library/Fonts/Supplemental/PingFang.ttc",
        "/Library/Fonts/PingFang.ttc",
        # 较新的 macOS 把系统中文字体放在字体资产目录里
        "/System/Library/AssetsV2/com_apple_MobileAsset_Font*/**/PingFang.ttc",
    )
    for pattern in patterns:
        hits = sorted(glob.glob(pattern, recursive=True))
        if hits:
            return hits[0]
    return None


def _extract_pingfang_sc(ttc: str, cache_dir: pathlib.Path) -> list[pathlib.Path]:
    """从 ttc 中抽出「苹方-简」的 Regular/Medium，返回可用字体文件路径。"""
    from fontTools.ttLib import TTCollection  # 可选依赖

    collection = TTCollection(ttc, lazy=True)
    picked = {}
    for font in collection.fonts:
        postscript = font["name"].getDebugName(6)
        if postscript in _PINGFANG_POSTSCRIPT and postscript not in picked:
            picked[postscript] = font

    paths = []
    for postscript, font in picked.items():
        target = cache_dir / f"{postscript}.ttf"
        if not target.exists() or target.stat().st_size == 0:
            font.save(str(target))
        paths.append(target)
    return paths


def setup_font(verbose: bool = True) -> str:
    """注册并返回主字体族名；失败时降级到后备字体。"""
    family = ""
    if _installed_family("PingFang SC"):
        family = "PingFang SC"
    else:
        ttc = _find_pingfang_ttc()
        if ttc:
            cache = pathlib.Path(tempfile.gettempdir()) / "suspension_figures_fonts"
            try:
                cache.mkdir(parents=True, exist_ok=True)
                for path in _extract_pingfang_sc(ttc, cache):
                    font_manager.fontManager.addfont(str(path))
                if _installed_family("PingFang SC"):
                    family = "PingFang SC"
            except ImportError:
                if verbose:
                    print("[font] 未安装 fontTools，无法从 ttc 抽出「苹方-简」；"
                          "pip install fonttools 后重试，或接受后备字体。",
                          file=sys.stderr)
            except Exception as exc:  # noqa: BLE001 - 字体问题不应中断出图
                if verbose:
                    print(f"[font] 抽取苹方失败：{exc}", file=sys.stderr)

    fallbacks = [name for name in _FALLBACK_FAMILIES if _installed_family(name)]
    if not family:
        family = fallbacks[0] if fallbacks else "DejaVu Sans"
        if verbose:
            print(f"[font] 未找到苹方，降级为 {family}", file=sys.stderr)
    elif verbose:
        print(f"[font] 使用 {family}")

    plt.rcParams.update({
        "font.family": [family] + fallbacks,
        "font.size": 9.5,
        "axes.unicode_minus": False,
        "figure.facecolor": "white",
        "savefig.facecolor": "white",
        "figure.dpi": DPI,
        "savefig.dpi": DPI,
        "svg.fonttype": "path",  # 将来若输出 SVG，文字转路径，字体不丢
    })
    return family


# ---------------------------------------------------------------- 数据处理
def load(data_dir: str, name: str, smooth: int = 9) -> dict:
    df = pd.read_excel(os.path.join(data_dir, name + ".xlsx")).dropna(subset=["Time(s)"])
    t = df["Time(s)"].to_numpy()
    df = df[(t > 0.5) & (t < t.max() - 0.5)].reset_index(drop=True)
    out = {"t": df["Time(s)"].to_numpy()}
    columns = {"roll": "Roll(deg)", "pitch": "Pitch(deg)",
               "acc": "Acc_Z(g)"}
    for key, column in columns.items():
        raw = df[column].astype(float)
        if key == "acc":
            raw = raw / 9.80665  # 该列实际单位是 m/s²
        raw = raw - raw.mean()
        out[key] = raw.rolling(smooth, center=True, min_periods=1).mean()
        # 保留去均值后的原始序列：波动带 = 原始 - 平滑 的局部 σ
        out[key + "_raw"] = raw
    return out


# ---------------------------------------------------------------- 版式
def _style_axes(ax) -> None:
    """统一的极简坐标轴：只留左/下轴线与极淡横向网格。"""
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(AXIS)
        ax.spines[side].set_linewidth(0.8)
    ax.tick_params(axis="both", length=0, pad=6, colors=MUTED, labelsize=8.5)
    ax.yaxis.set_major_locator(MaxNLocator(5, prune=None))
    ax.xaxis.set_major_locator(MaxNLocator(6))
    ax.yaxis.grid(True, color=GRID, linewidth=0.8)
    ax.xaxis.grid(False)
    ax.set_axisbelow(True)
    ax.axhline(0.0, color=ZERO, linewidth=0.7, zorder=1)


def _decorate_axes(axes) -> None:
    for ax in axes:
        for label in ax.get_yticklabels() + ax.get_xticklabels():
            label.set_color(MUTED)
        ax.title.set_color(INK)


def _confidence_band(ax, t, smooth, raw) -> None:
    """主动线的很淡波动带：原始信号相对 1 s 趋势线的局部 ±1σ。

    注意这不是统计置信区间（单次采集没有重复样本），而是 1 s 尺度上的
    观测波动幅度；相对 36 ms 平滑均值的残差量级太小，画出来不可见。
    """
    base = max(3, int(BAND_BASE_S * SAMPLE_HZ))
    win = max(3, int(BAND_WINDOW_S * SAMPLE_HZ))
    trend = raw.rolling(base, center=True, min_periods=1).mean()
    residual = raw - trend
    sigma = residual.rolling(win, center=True, min_periods=win // 4).std()
    sigma = sigma.bfill().ffill().fillna(0.0)
    sigma = sigma.rolling(BAND_SMOOTH, center=True, min_periods=1).mean()
    ax.fill_between(t, smooth - BAND_SIGMA * sigma, smooth + BAND_SIGMA * sigma,
                    color=ACTIVE, alpha=BAND_ALPHA, linewidth=0, zorder=1.6)


def _legend(ax, labels) -> None:
    """图例放到坐标区上方：一行、无边框、细线段 + 波动带色块。"""
    handles = [plt.Line2D([], [], color=PASSIVE, lw=LW_PASSIVE),
               plt.Line2D([], [], color=ACTIVE, lw=LW_ACTIVE),
               Patch(facecolor=ACTIVE, alpha=0.25, edgecolor="none")]
    entries = list(labels) + ["±1σ 波动带"]
    ax.legend(handles, entries, loc="lower left", bbox_to_anchor=(0.0, 1.01),
              ncol=3, frameon=False, fontsize=9, handlelength=2.2,
              handletextpad=0.6, columnspacing=1.8, borderaxespad=0.0)


def attitude(data_dir: str, out_dir: str) -> str:
    passive, active = load(data_dir, "被动悬架01"), load(data_dir, "主动悬架01")
    fig, axes = plt.subplots(2, 1, figsize=(FIG_W, 4.2), sharex=True,
                             gridspec_kw={"hspace": 0.20})
    for ax, key, ylabel in ((axes[0], "roll", "横滚角 / °"),
                            (axes[1], "pitch", "俯仰角 / °")):
        _confidence_band(ax, active["t"], active[key], active[key + "_raw"])
        ax.plot(passive["t"], passive[key], color=PASSIVE, lw=LW_PASSIVE,
                label="被动悬架", zorder=2)
        ax.plot(active["t"], active[key], color=ACTIVE, lw=LW_ACTIVE,
                label="主动悬架", zorder=3)
        ax.set_ylabel(ylabel, fontsize=9.5, color=INK, labelpad=6)
        ax.set_xlim(0, 15)
        _style_axes(ax)

    _legend(axes[0], ["被动悬架", "主动悬架"])
    axes[-1].set_xlabel("时间 / s", fontsize=9.5, color=INK, labelpad=8)
    _decorate_axes(axes)
    fig.subplots_adjust(left=0.095, right=0.985, top=0.90, bottom=0.15)

    path = os.path.join(out_dir, "attitude_compare.png")
    fig.savefig(path, dpi=DPI, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    return path


def obstacle(data_dir: str, out_dir: str) -> str:
    passive = load(data_dir, "被动悬架03")
    active = load(data_dir, "主动悬架03")
    fig, ax = plt.subplots(figsize=(FIG_W, 2.5))
    _confidence_band(ax, active["t"], active["pitch"], active["pitch_raw"])
    ax.plot(passive["t"], passive["pitch"], color=PASSIVE, lw=LW_PASSIVE,
            label="被动悬架", zorder=2)
    ax.plot(active["t"], active["pitch"], color=ACTIVE, lw=LW_ACTIVE,
            label="主动悬架", zorder=3)
    ax.set_xlabel("时间 / s", fontsize=9.5, color=INK, labelpad=8)
    ax.set_ylabel("俯仰角 / °", fontsize=9.5, color=INK, labelpad=8)
    ax.set_xlim(0, 27)
    # 纵轴范围钉住，避免波动带把自动缩放撑大、刻度变成 0/6/12
    ax.set_ylim(-4.6, 17.4)
    _style_axes(ax)
    _legend(ax, ["被动悬架", "主动悬架"])
    fig.subplots_adjust(left=0.095, right=0.985, top=0.82, bottom=0.19)

    path = os.path.join(out_dir, "obstacle_pitch.png")
    fig.savefig(path, dpi=DPI, bbox_inches="tight", pad_inches=0.05)
    plt.close(fig)
    return path


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default="../试验记录", help="试验记录 xlsx 目录")
    parser.add_argument("--out", default=".", help="图片输出目录")
    parser.add_argument("--dpi", type=int, default=DPI)
    args = parser.parse_args()
    DPI = args.dpi

    setup_font()
    os.makedirs(args.out, exist_ok=True)
    for path in (attitude(args.data, args.out), obstacle(args.data, args.out)):
        print(f"written {path} ({os.path.getsize(path) // 1024} KB)")
