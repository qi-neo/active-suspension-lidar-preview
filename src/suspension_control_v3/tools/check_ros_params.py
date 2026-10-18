#!/usr/bin/env python3
"""检查 ROS 节点声明的参数与 YAML 配置文件的键是否一一对应。

为什么需要这个检查：ROS2 里 YAML 的键名是纯字符串匹配。键名写错、或者代码里
改了参数名而 YAML 没跟着改，结果都是**参数被静默忽略**——节点照常启动、照常
跑，只是那个值从来没生效过。反过来，节点读了某参数但 YAML 里没写，用户翻遍
配置文件也找不到它，只能吃代码里的默认值。

两个方向都要拦：
  YAML 有、节点不读  -> 用户以为改了，其实没用（最危险）
  节点读、YAML 没写  -> 用户根本不知道有这个可调项

用法：
  python3 tools/check_ros_params.py
  python3 tools/check_ros_params.py --node <cpp> --yaml <yaml>

退出码：0 = 一一对应；1 = 有漂移（并逐条列出）。
"""

import argparse
import os
import re
import sys

# declare_parameter<T>("name", ...) 里的名字。
# 用非贪婪的 .+? 而不是 [^>]+：模板参数可能是 std::vector<double>，
# 那里面有 '>'，[^>]+ 会提前收尾从而漏掉这种声明。
# 再允许换行：代码里常见的写法是
#     ctrl_.preview_gains = declare_parameter<std::vector<double>>(
#         "preview_gains", ctrl_.preview_gains);
DECL_RE = re.compile(r'declare_parameter<.+?>\(\s*\n?\s*"([a-z0-9_]+)"', re.S)

# YAML 顶层键。参数文件形如
#     suspension_control_node:
#       ros__parameters:
#         control_rate_hz: 200.0
# 所以真正的参数在 4 空格缩进处；再深的缩进是列表续行或注释。
YAML_KEY_RE = re.compile(r'^\s{0,4}([a-z0-9_]+):')

# YAML 的结构键，不是节点参数
STRUCTURAL = {"ros__parameters"}


def node_params(path):
    with open(path, encoding="utf-8") as f:
        src = f.read()
    return set(DECL_RE.findall(src))


def yaml_params(path):
    keys = set()
    with open(path, encoding="utf-8") as f:
        for line in f:
            if line.lstrip().startswith("#"):
                continue
            m = YAML_KEY_RE.match(line)
            if m:
                keys.add(m.group(1))
    # 节点名那一层也当结构键：它是 <node_name>: 而不是参数
    keys -= STRUCTURAL
    return keys


def main():
    # 不依赖调用者的当前目录：默认路径从脚本自身位置推出来
    pkg = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--node", default=f"{pkg}/src/suspension_control_node.cpp")
    ap.add_argument("--yaml", default=f"{pkg}/config/control_params.yaml")
    args = ap.parse_args()

    declared = node_params(args.node)
    keys = yaml_params(args.yaml)
    # YAML 里以节点名开头的那一行也算结构键
    keys = {k for k in keys if not k.endswith("_node")}

    orphan = sorted(keys - declared)
    missing = sorted(declared - keys)

    print(f"节点声明参数 {len(declared)} 个；YAML 键 {len(keys)} 个")

    if not orphan and not missing:
        print("参数与 YAML 一一对应。")
        return 0

    if orphan:
        print(f"\n[错误] YAML 里有 {len(orphan)} 个键节点从不读取"
              f"（用户以为改了，其实静默无效）：")
        for k in orphan:
            print(f"    {k}")
    if missing:
        print(f"\n[错误] 节点读取 {len(missing)} 个参数但 YAML 里没有"
              f"（用户发现不了这个可调项）：")
        for k in missing:
            print(f"    {k}")

    print("\n修法：键名对齐——YAML 里的键必须与 declare_parameter 的字符串完全一致。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
