#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SWD 采样 —— 不接串口，直接用调试器读惯导状态。

用途（就是"推车对图"那一步）：
    python 工具/SWD采样.py --secs 20
然后沿赛道推车，看打出来的表：
   * X / Y 应该沿着地图中心线走
   * YW 应该跟着赛道方向变（第一段直道稳定在 0 附近）
   * 最右一列"离线"是当前位姿到地图中心线的横向偏差 —— 推在线上应该很小
对不上（比如 YW 往反方向走）= 陀螺符号反了 -> 改 User/NAV.h 的 NAV_YAW_SIGN

为什么不用串口：C 车的 USART1 绑在 JDY-31 蓝牙上，得先跟电脑配对才能收遥测；
调试器本来就在手上，直接读 RAM 更快，也不用抢蓝牙（手机 APP 还要用）。

要连调试器 + 供电。工具会自动找 openocd 和符号地址（读 Listings/Project.map）。

参数：
    --secs N      采样多少秒（默认 15）
    --interval MS 采样间隔毫秒（默认 250；越小越密，但每次采样会短暂停一下 CPU）
"""

import argparse
import math
import re
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
MAP_FILE = REPO / "Listings" / "Project.map"
FW_MAP = REPO / "User" / "nav_map_fw.h"

OCD_CANDIDATES = [
    Path.home() / "AppData/Local/Programs/xpack-openocd-0.12.0-7/openocd",
    Path("C:/Users/asus/AppData/Local/Programs/xpack-openocd-0.12.0-7/openocd"),
]

# 要采的变量（名字 -> 每次采样要读几个 32 位字）
WATCH = {
    "s_x10":      1,   # 位置 x，0.1mm
    "s_y10":      1,   # 位置 y，0.1mm
    "s_yaw_acc":  1,   # 航向积分，0.001 度
    "s_yawc16":   1,   # 光电修出来的补偿角，1/16 度
    "s_odo10":    1,   # 累计里程，0.1mm
    "s_idx":      1,   # 路表下标
    "s_ydot":     1,   # 带符号角速度，度/秒（右转为正）
    "g_ms_tick":  1,
}


def find_openocd():
    import shutil
    w = shutil.which("openocd")
    if w:
        return Path(w)
    for p in OCD_CANDIDATES:
        if (p / "bin" / "openocd.exe").exists():
            return p / "bin" / "openocd.exe"
        if p.exists() and p.is_file():
            return p
    sys.exit("找不到 openocd.exe，改一下本脚本里的 OCD_CANDIDATES")


def load_symbols():
    if not MAP_FILE.exists():
        sys.exit(f"找不到 {MAP_FILE}，先在 Keil 里编译一次")
    s = MAP_FILE.read_bytes().decode("utf-8", errors="replace")
    addr = {}
    for name in WATCH:
        m = re.search(rf"\b{re.escape(name)}\s+(0x[0-9a-fA-F]+)", s)
        if not m:
            sys.exit(f"map 里找不到符号 {name}")
        addr[name] = int(m.group(1), 16)
    return addr


def load_map():
    txt = FW_MAP.read_text(encoding="ascii")
    ds = int(re.search(r"#define\s+NAV_MAP_DS_MM\s+(\d+)", txt).group(1))
    pts = [(int(a), int(b), int(k)) for a, b, k in
           re.findall(r"\{\s*(-?\d+),\s*(-?\d+),\s*(\d+)\}", txt)]
    return pts, ds


def build_openocd_cmd(ocd, addr, secs, interval_ms):
    # xPack 的布局是 <前缀>/bin/openocd.exe + <前缀>/openocd/scripts，
    # 但别的发行版可能不一样，所以几个候选都试一遍。
    candidates = [
        ocd.parent.parent / "openocd" / "scripts",   # xPack
        ocd.parent.parent / "scripts",               # 部分发行版
        ocd.parent / "scripts",
        ocd.parent.parent / "share" / "openocd" / "scripts",
    ]
    scripts = next((c for c in candidates if c.is_dir()), None)
    if scripts is None:
        sys.exit("找不到 openocd 的 scripts 目录，试过：\n  "
                 + "\n  ".join(str(c) for c in candidates))

    reads = []
    for name in WATCH:
        reads += ["-c", f"mdw 0x{addr[name]:08x} {WATCH[name]}"]

    n = max(1, int(secs * 1000 / interval_ms))
    cmd = [str(ocd), "-s", str(scripts),
           "-f", "interface/cmsis-dap.cfg",
           "-f", "target/stm32f1x.cfg",
           "-c", "adapter speed 1000",
           "-c", "init"]
    for _ in range(n):
        cmd += ["-c", "halt"] + reads + ["-c", "resume",
                                         "-c", f"sleep {interval_ms}"]
    cmd += ["-c", "shutdown"]
    return cmd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--secs", type=float, default=15.0)
    ap.add_argument("--interval", type=int, default=250)
    args = ap.parse_args()

    ocd = find_openocd()
    addr = load_symbols()
    pts, ds = load_map()
    n = len(pts)
    print(f"openocd: {ocd}")
    print(f"地图: {n} 点 @ {ds}mm，单圈 {n*ds} mm；采样 {args.secs}s / 每 {args.interval}ms")
    print()

    cmd = build_openocd_cmd(ocd, addr, args.secs, args.interval)
    try:
        pr = subprocess.run(cmd, capture_output=True, text=True,
                            timeout=args.secs + 120, errors="replace")
    except subprocess.TimeoutExpired:
        sys.exit("openocd 超时 —— 检查调试器是否插好、车是否上电")
    # 【注意】openocd 把自己的输出全写到 stderr，不是 stdout —— 两边都要收
    out = (pr.stdout or "") + (pr.stderr or "")

    # 每采样点形如： "0x20000098: 0000080c"
    rows = []
    cur = {}
    for line in out.splitlines():
        m = re.match(r"^(0x[0-9a-fA-F]+):\s*([0-9a-fA-F ]+)$", line.strip())
        if not m:
            continue
        a = int(m.group(1), 16)
        val = int(m.group(2).split()[0], 16)
        if val >= 0x80000000:      # mdw 读出来是无符号，这些变量都是 int32，要转回有符号
            val -= 0x100000000
        for name, ad in addr.items():
            if a == ad:
                cur[name] = val
                break
        if len(cur) == len(WATCH):
            rows.append(dict(cur))
            cur = {}

    if not rows:
        print(f"openocd 退出码 {pr.returncode}")
        print("--- stdout 末尾 ---")
        print("\n".join(out.splitlines()[-15:]))
        if pr.stderr:
            print("--- stderr 末尾 ---")
            print("\n".join(pr.stderr.splitlines()[-15:]))
        sys.exit("没采到任何数据")

    print(f"{'tick':>9} {'X':>7} {'Y':>7} {'YW':>8} {'YC':>7} {'里程':>8} "
          f"{'idx':>5} {'kind':>4} {'ω':>6} {'离线':>7}")
    print("-" * 82)
    for r in rows:
        x = r["s_x10"] / 10.0          # mm
        y = r["s_y10"] / 10.0
        yw = r["s_yaw_acc"] / 1000.0   # 度
        yc = r["s_yawc16"] / 16.0
        odo = r["s_odo10"] / 10.0
        idx = r["s_idx"] % n
        kind = pts[idx][2]
        # 当前位姿到地图中心线的横向偏差（用地图点的前进方向算）
        j = (idx + 2) % n
        vx, vy = pts[j][0] - pts[idx][0], pts[j][1] - pts[idx][1]
        L = math.hypot(vx, vy) or 1.0
        off = ((x - pts[idx][0]) * (-vy) + (y - pts[idx][1]) * vx) / L
        print(f"{r['g_ms_tick']:9d} {x:7.0f} {y:7.0f} {yw:8.1f} {yc:7.1f} "
              f"{odo:8.0f} {idx:5d} {kind:4d} {r['s_ydot']:6d} {off:+7.1f}")

    print()
    first, last = rows[0], rows[-1]
    dt = (last["g_ms_tick"] - first["g_ms_tick"]) / 1000.0
    print(f"共 {len(rows)} 个采样，覆盖 {dt:.1f} 秒")
    print(f"X {first['s_x10']/10:+.0f} -> {last['s_x10']/10:+.0f} mm，"
          f"Y {first['s_y10']/10:+.0f} -> {last['s_y10']/10:+.0f} mm，"
          f"航向 {first['s_yaw_acc']/1000:+.1f} -> {last['s_yaw_acc']/1000:+.1f} 度，"
          f"里程 {last['s_odo10']/10:.0f} mm")
    print("\n判读：X/Y 沿中心线走、YW 跟着赛道方向、最后一列|离线|小 -> 符号对")
    print("      YW 往反方向走 -> 把 User/NAV.h 的 NAV_YAW_SIGN 改成 (1)")


if __name__ == "__main__":
    main()
