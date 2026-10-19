#!/usr/bin/env python3
"""检查 ROS 节点声明的参数与 YAML 配置文件的键是否一一对应（多节点）。

为什么需要这个检查：ROS2 里 YAML 的键名是纯字符串匹配。键名写错、或者代码里
改了参数名而 YAML 没跟着改，结果都是**参数被静默忽略**——节点照常启动、照常
跑，只是那个值从来没生效过。反过来，节点读了某参数但 YAML 里没写，用户翻遍
配置文件也找不到它，只能吃代码里的默认值。

两个方向都要拦：
  YAML 有、节点不读  -> 用户以为改了，其实没用（最危险）
  节点读、YAML 没写  -> 用户根本不知道有这个可调项

覆盖范围（默认两者都查）：
  v3 悬架控制器  suspension_control_node.cpp + control_params.yaml
  v2 路面预瞄    road_preview_node.cpp      + preview_params.yaml

另一个同等危险的静默失效是 YAML 顶层节点名与节点实际名字不一致：ROS2 只把
名字匹配的那一块参数喂给节点，名字对不上时**整份 YAML 都不生效**，节点全用
代码默认值，日志没有任何提示。这里一并核对顶层节点名。

用法：
  python3 src/suspension_control_v3/tools/check_ros_params.py          # v3 + v2 都查
  python3 src/suspension_control_v3/tools/check_ros_params.py --target v3
  python3 src/suspension_control_v3/tools/check_ros_params.py --target v2
  python3 src/suspension_control_v3/tools/check_ros_params.py \\
      --node <cpp> --yaml <yaml>        # 自定义单对（覆盖 --target）

退出码：0 = 全部一一对应；1 = 有漂移（并逐条列出）。
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

# YAML 顶层节点名：suspension_control_node:
NODE_LINE_RE = re.compile(r'^([A-Za-z0-9_]+)\s*:\s*$')
# ros__parameters 行（缩进不固定，按实际长度推参数层缩进）
PARAMS_LINE_RE = re.compile(r'^(\s*)ros__parameters\s*:\s*$')
# 参数键：`key:` 或 `key: value`
KEY_LINE_RE = re.compile(r'^(\s*)([A-Za-z0-9_]+)\s*:')


def node_params(path):
    """返回节点源码里 declare_parameter 声明过的参数名集合。"""
    with open(path, encoding="utf-8") as f:
        src = f.read()
    return set(DECL_RE.findall(src))


def yaml_params(path):
    """返回 (YAML 顶层节点名, ros__parameters 块里的参数键集合)。

    不用 PyYAML：CI 容器里不保证有，而这里只需要键集合。按缩进解析：
    只有恰好在 ros__parameters 下一层缩进的 `key:` 才是该节点的参数，
    更深的缩进是列表续行/嵌套结构，更浅的是另一个参数块。
    """
    node_name = None
    params_indent = None
    keys = set()
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.rstrip("\n")
            stripped = line.strip()
            if not stripped or stripped.startswith("#"):
                continue
            indent = len(line) - len(line.lstrip(" "))

            if params_indent is not None and indent >= params_indent:
                m = KEY_LINE_RE.match(line)
                if m and len(m.group(1)) == params_indent:
                    keys.add(m.group(2))
                continue
            if params_indent is not None and indent < params_indent:
                params_indent = None

            m = PARAMS_LINE_RE.match(line)
            if m:
                # 参数在 ros__parameters 的下一层，通常是 +2 空格
                params_indent = len(m.group(1)) + 2
                continue

            m = NODE_LINE_RE.match(line)
            if m and node_name is None:
                node_name = m.group(1)
    return node_name, keys


def check_pair(label, node_path, yaml_path, expected_yaml_node=None):
    """检查一个节点/YAML 对。打印结果，返回问题条数（0 表示通过）。"""
    print(f"== {label} ==")
    problems = 0

    if not os.path.isfile(node_path):
        print(f"[错误] 找不到节点源码：{node_path}")
        return 1
    if not os.path.isfile(yaml_path):
        print(f"[错误] 找不到参数 YAML：{yaml_path}")
        return 1

    declared = node_params(node_path)
    yaml_node, keys = yaml_params(yaml_path)

    if not declared:
        print(f"[错误] 没从 {os.path.basename(node_path)} 里解析到任何 "
              f"declare_parameter：要么节点真的没声明参数，要么声明写法变了"
              f"（本检查的正则只认 declare_parameter<T>(\"name\") 形式）。")
        problems += 1
    if not keys:
        print(f"[错误] 没从 {os.path.basename(yaml_path)} 里解析到任何参数键："
              f"要么 ros__parameters 块结构变了，要么文件是空的。")
        problems += 1

    # YAML 顶层节点名与节点实际名字必须一致，否则整份 YAML 被静默忽略
    if expected_yaml_node is not None and yaml_node != expected_yaml_node:
        print(f"[错误] YAML 顶层节点名是 {yaml_node!r}，节点实际名字是 "
              f"{expected_yaml_node!r}：ROS2 只加载名字匹配的参数块，"
              f"这份 YAML 会整份被静默忽略（节点全用代码默认值，日志无提示）。")
        problems += 1

    orphan = sorted(keys - declared)     # YAML 有、代码不读
    missing = sorted(declared - keys)    # 代码读、YAML 没写

    print(f"节点声明参数 {len(declared)} 个；YAML 键 {len(keys)} 个")

    if orphan:
        problems += 1
        print(f"\n[错误] YAML 里有 {len(orphan)} 个键节点从不读取"
              f"（用户以为改了，其实静默无效）：")
        for k in orphan:
            print(f"    {k}")
    if missing:
        problems += 1
        print(f"\n[错误] 节点读取 {len(missing)} 个参数但 YAML 里没有"
              f"（用户发现不了这个可调项）：")
        for k in missing:
            print(f"    {k}")

    if problems:
        print("\n修法：键名对齐——YAML 里的键必须与 declare_parameter 的字符串"
              "完全一致，且顶层节点名必须是节点真实名字。")
        print()
        return problems

    print("参数与 YAML 一一对应。\n")
    return 0


def main():
    # 不依赖调用者的当前目录：默认路径从脚本自身位置推出来
    here = os.path.abspath(__file__)
    pkg_v3 = os.path.dirname(os.path.dirname(here))   # src/suspension_control_v3
    src_dir = os.path.dirname(pkg_v3)                  # src
    pkg_v2 = os.path.join(src_dir, "suspension_preview_v2")

    targets = {
        "v3": {
            "label": "v3 悬架控制器（suspension_control_node + control_params.yaml）",
            "node": os.path.join(pkg_v3, "src", "suspension_control_node.cpp"),
            "yaml": os.path.join(pkg_v3, "config", "control_params.yaml"),
            "yaml_node": "suspension_control_node",
        },
        "v2": {
            "label": "v2 路面预瞄（road_preview_node + preview_params.yaml）",
            "node": os.path.join(pkg_v2, "src", "road_preview_node.cpp"),
            "yaml": os.path.join(pkg_v2, "config", "preview_params.yaml"),
            "yaml_node": "road_preview_node",
        },
    }

    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", choices=["all"] + sorted(targets), default="all",
                    help="要检查哪一组（默认 all = v3 + v2 都查）")
    ap.add_argument("--node", help="自定义节点源码路径（需与 --yaml 同时给）")
    ap.add_argument("--yaml", help="自定义参数 YAML 路径（需与 --node 同时给）")
    args = ap.parse_args()

    if args.node or args.yaml:
        if not (args.node and args.yaml):
            ap.error("--node 与 --yaml 必须同时给（自定义单对检查）")
        pairs = [("自定义", {
            "label": f"自定义（{os.path.basename(args.node)} + "
                     f"{os.path.basename(args.yaml)}）",
            "node": args.node,
            "yaml": args.yaml,
            "yaml_node": None,
        })]
    else:
        names = sorted(targets) if args.target == "all" else [args.target]
        pairs = [(name, targets[name]) for name in names]

    total_problems = 0
    for _, spec in pairs:
        total_problems += check_pair(
            spec["label"], spec["node"], spec["yaml"], spec["yaml_node"])

    if total_problems:
        print(f"检查失败：共 {total_problems} 处需要修。")
        return 1
    print(f"全部 {len(pairs)} 组参数与 YAML 一一对应。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
