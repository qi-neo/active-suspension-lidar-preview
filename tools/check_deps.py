#!/usr/bin/env python3
"""核对每个 ROS2 包的 CMakeLists 与 package.xml 依赖声明是否一致。

为什么要查：package.xml 是 rosdep 的唯一依据。CMakeLists 里
`find_package(X)` 了但 package.xml 没声明 `<depend>X</depend>`，
在开发机上通常看不出来（别的包或全局环境已经把依赖装上了），
到了干净环境里 rosdep 不会装它，configure 或链接阶段才炸。

本项目已经因为这类脱节踩过两次：
  - unilidar_driver：CMakeLists 早已 find_package(geometry_msgs)，
    package.xml 漏了声明；
  - suspension_preview：CMakeLists 改用 find_package(PCL ...) 之后，
    package.xml 一直没补 libpcl-all-dev。

用法：
    python3 tools/check_deps.py            # 检查本仓库所有包
    python3 tools/check_deps.py src/xxx    # 只检查指定目录

退出码 0 表示一致，1 表示有漏声明。
"""

import os
import re
import sys
import xml.etree.ElementTree as ET

# 只在 CMakeLists 里出现、不需要（也不应该）在 package.xml 声明的东西
NOT_A_ROS_PACKAGE = {
    "ament_cmake",        # buildtool_depend，不是运行依赖
    "PCL",                # 系统库，用 rosdep key libpcl-all-dev 声明
    "Threads",
    "PkgConfig",
}

# 只在 package.xml 里出现是正常的（ament 生成的运行时/工具依赖，
# CMakeLists 不需要 find_package）
RUNTIME_ONLY_OK = {
    "launch", "launch_ros", "rosidl_default_runtime",
    "ament_copyright", "ament_flake8", "ament_lint_auto", "ament_pep257",
    "python3-pytest",
}

DEPEND_TAGS = ("depend", "build_depend", "exec_depend", "test_depend",
               "buildtool_depend")


def strip_cmake_comments(text):
    """去掉 CMake 注释，避免从注释里匹配出 find_package。

    本项目自己的注释里就出现过 "而 v3 通过 find_package(suspension_preview_v2)
    依赖它" 这种描述性写法，不去掉会当成真实依赖。
    CMake 里 # 到行尾都是注释（引号内的除外），对这种规模的文件足够。
    """
    return "\n".join(line.split("#", 1)[0] for line in text.split("\n"))


def find_package_targets(text):
    """取出 CMakeLists 里所有 find_package(X)。"""
    out = set()
    for m in re.finditer(r"find_package\(\s*([A-Za-z0-9_]+)", strip_cmake_comments(text)):
        out.add(m.group(1))
    return out


def declared_deps(package_xml):
    """取出 package.xml 里声明的所有依赖名。"""
    root = ET.parse(package_xml).getroot()
    return {e.text.strip() for e in root
            if e.tag in DEPEND_TAGS and e.text and e.text.strip()}


def rosdep_keys_of(dep_names):
    """把 ROS 包名映到 rosdep key。

    这里只处理本项目实际出现的情况：<pkg>_msgs 之类按下划线去掉后缀，
    以及 PCL 对应 libpcl-all-dev。其余按同名处理。
    """
    keys = set(dep_names)
    for d in dep_names:
        if d.endswith("_msgs"):
            keys.add(d[: -len("_msgs")])
        if d == "pcl_conversions" or d == "pcl_ros":
            keys.add("libpcl-all-dev")
    return keys


def check_package(pkg_dir):
    """检查单个包，返回 (缺失声明列表, 状态串)。缺失为空表示通过。"""
    name = os.path.basename(pkg_dir.rstrip("/"))
    cm = os.path.join(pkg_dir, "CMakeLists.txt")
    px = os.path.join(pkg_dir, "package.xml")
    if not os.path.isfile(cm):
        return [], f"{name:26s} 非 CMake 包，跳过"

    found = find_package_targets(open(cm, encoding="utf-8").read()) - NOT_A_ROS_PACKAGE
    declared = declared_deps(px)
    declared_keys = rosdep_keys_of(declared)

    missing = sorted(p for p in found
                     if p not in declared and p not in declared_keys)

    status = "OK" if not missing else "缺声明: " + ", ".join(missing)
    return missing, f"{name:26s} find_package {len(found)} 个，声明 {len(declared)} 个 -> {status}"


def main():
    roots = sys.argv[1:]
    if not roots:
        roots = ["src"]

    pkg_dirs = []
    for root in roots:
        if not os.path.isdir(root):
            print(f"路径不存在: {root}", file=sys.stderr)
            return 1
        for dirpath, dirnames, _ in os.walk(root):
            dirnames[:] = [d for d in dirnames
                           if d not in ("__pycache__", "build", "install", "log")]
            if os.path.isfile(os.path.join(dirpath, "package.xml")):
                pkg_dirs.append(dirpath)

    if not pkg_dirs:
        print("未找到任何含 package.xml 的包")
        return 0

    all_missing = {}
    print("=== CMakeLists 与 package.xml 依赖一致性 ===\n")
    for d in sorted(pkg_dirs):
        missing, line = check_package(d)
        print("  " + line)
        if missing:
            all_missing[d] = missing

    print()
    if all_missing:
        print("发现漏声明：")
        for d, missing in all_missing.items():
            print(f"  {d}")
            print("    在 package.xml 里补上，例如：")
            for m in missing:
                print(f"      <depend>{m}</depend>")
        print("\n漏声明只在干净环境里才会暴露（rosdep 靠 package.xml 装依赖）。")
        return 1

    print(f"全部 {len(pkg_dirs)} 个包的依赖声明一致。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
