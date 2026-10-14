#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
生成固件地图 nav_map_fw.h —— 给 C 车（SMARTCAR2）惯导用。

输入：导航地图/赛道中心线-mm.json（10mm 全精度，队友从 CAD 图提取）
输出：导航地图/nav_map_fw.h    （int16 x/y + uint8 kind，50mm，**只占 Flash，不进 RAM**）

为什么要有这个固件版，而不是直接用 nav_map.h：
  1. nav_map.h 是 float×5 = 20 字节/点（869 点 ≈ 17KB Flash）；
     固件版 int16 x/y + uint8 kind = 5 字节/点（≈4.2KB Flash）。C8T6 只有 64KB Flash。
  2. **yaw 列不要**。表里的 yaw_deg 是【平滑过的航向】，在 s=0（闭环处）有明显伪影：
     首段 0~500mm 明明沿 +X 直走，表里却写 yaw=-45°，要到第 50 点才收敛到 0°。
     直接用它做起始航向对齐会一开始就偏 45 度。固件里行进方向**从地图几何现算**
     （前瞻点的弦方向），所以这一列不需要。
  3. **raw radius_mm 不要**。测出来一段笔直的线上 R 也报 110mm（曲率估计在端点和
     直线段上噪声很大）。kind（直道分类）在这里离线用**弦角法**算好，比固件里现算稳。

kind 约定（与 ins-nav 的 NAV.c 一致）：
  0 = 普通（弯道）
  1 = 直道端部（过渡段，**不**允许光电修航向）
  2 = 直道中段（**唯一**允许光电修航向的段）

头文件里的注释一律 ASCII —— 因为 C 源码是 GBK 编码，混编码的注释在 Keil 里
会显示成乱码；数据本身全是数字，注释用英文最省事。中文说明看本脚本和 README。

用法：
    python 生成固件地图.py                    # 原地生成 nav_map_fw.h
    python 生成固件地图.py --check-only       # 只做自检，不写文件
"""

import argparse
import json
import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
JSON_PATH = HERE / "赛道中心线-mm.json"
REF_H_PATH = HERE / "nav_map.h"          # 用来交叉自检
# 输出到 User/ 而不是本目录：本目录名是中文，把它加进 Keil 的 IncludePath
# 会引入路径编码风险（B 车踩过编码的坑）。User/ 是纯 ASCII 路径，稳。
OUT_PATH = HERE.parent / "User" / "nav_map_fw.h"

# ---- 采样：nav_map.h 就是本 JSON 每 5 个点取一个（10mm -> 50mm）----
DECIMATE = 5

# ---- 直道分类参数（离线，可用浮点，所以能做得比固件里粗算更稳）----
CHORD_PTS = 2          # 弦跨几个点（2 x 50mm = 100mm）
STRAIGHT_TURN_DEG = 4.0  # 两段弦夹角小于它算"近直"（100mm+100mm 上 4 度 ≈ R>2800mm）
MIN_STRAIGHT_MM = 200.0  # 连续近直至少这么长才算一段直道
TRIM_MM = 90.0           # 直道两端各裁这么多当过渡（不修航向）
START_YAW_PTS = 10       # 起始航向用多长的弦算（10 x 50mm = 500mm）


def heading_deg(p, i, k):
    """从 p[i] 指向 p[i+k] 的行进方向，单位度。坐标系：X 右、Y 下，顺时针为正。"""
    n = len(p)
    a = p[i % n]
    b = p[(i + k) % n]
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


def ang_diff(a, b):
    """a-b 归一到 (-180, 180]"""
    return (a - b + 180.0) % 360.0 - 180.0


def load_json():
    with open(JSON_PATH, encoding="utf-8") as f:
        m = json.load(f)
    if abs(m["resample_mm"] - 10.0) > 1e-6:
        sys.exit(f"失败：JSON 的 resample_mm = {m['resample_mm']}，本脚本按 10mm 写死")
    if not m["closed"]:
        sys.exit("失败：地图不是闭环，惯导靠里程循环取点，必须是闭环")
    return m


def decimate(pts):
    """每 DECIMATE 个点取一个。"""
    out = pts[::DECIMATE]
    # 闭环：最后一个点不重复首点
    return out


def crosscheck_ref(pts50):
    """跟同目录的 nav_map.h（已验证过的 50mm 表）交叉核对 x/y。"""
    if not REF_H_PATH.exists():
        print(f"  [跳过] 找不到 {REF_H_PATH.name}，不做交叉核对")
        return True

    rows = []
    pat = re.compile(r"^\s*\{\s*([-\d.]+)f,\s*([-\d.]+)f,\s*([-\d.]+)f,")
    with open(REF_H_PATH, encoding="utf-8") as f:
        for line in f:
            mm = pat.match(line)
            if mm:
                rows.append((float(mm.group(2)), float(mm.group(3))))

    if len(rows) != len(pts50):
        print(f"  [不通过] nav_map.h 有 {len(rows)} 点，本脚本算出 {len(pts50)} 点")
        return False

    worst = 0.0
    for (rx, ry), (x, y) in zip(rows, pts50):
        worst = max(worst, abs(rx - x), abs(ry - y))
    ok = worst <= 0.05
    print(f"  [{'通过' if ok else '不通过'}] 与 nav_map.h 逐点比对，最大差 {worst:.3f} mm（限 0.05）")
    return ok


def classify(pts50):
    """弦角法标直道 -> kind 数组。0=普通 1=直道端部 2=直道中段"""
    n = len(pts50)
    ds = 50.0

    # 第一遍：逐点判"近直"
    near = [0] * n
    for i in range(n):
        h1 = heading_deg(pts50, i, CHORD_PTS)
        h2 = heading_deg(pts50, i + CHORD_PTS, CHORD_PTS)
        if abs(ang_diff(h2, h1)) < STRAIGHT_TURN_DEG:
            near[i] = 1

    # 第二遍：连续段够长的，中段标 2、两端标 1
    kind = [0] * n
    min_pts = max(2, int(round(MIN_STRAIGHT_MM / ds)))
    trim_pts = max(1, int(round(TRIM_MM / ds)))

    i = 0
    while i < n:
        if not near[i]:
            i += 1
            continue
        st = i
        while i < n and near[i]:
            i += 1
        run = i - st
        if run >= min_pts:
            for k in range(st, i):
                if (k - st) < trim_pts or ((i - 1) - k) < trim_pts:
                    kind[k] = 1
                else:
                    kind[k] = 2
    return kind


def emit(pts50, kind, out_path):
    n = len(pts50)
    lap = n * int(50)

    # 起始航向：用长一点的弦，避开闭环处的小折角
    yaw0 = heading_deg(pts50, 0, START_YAW_PTS)
    yaw0_mdeg = int(round(yaw0 * 1000.0))

    n2 = sum(1 for k in kind if k == 2)
    n1 = sum(1 for k in kind if k == 1)
    n0 = n - n1 - n2

    lines = []
    lines.append("/* Track centerline for the firmware map - GENERATED, DO NOT EDIT BY HAND.")
    lines.append(" *")
    lines.append(" * Source : the 10 mm full-precision track centerline JSON in this folder")
    lines.append(" * Maker  : the generator script in this folder (see this folder's README)")
    lines.append(f" * Points : {n} @ {50} mm, closed loop, lap = {lap} mm")
    lines.append(" * Layout : int16 x/y (mm) + uint8 kind  ->  5 B/point, Flash only, 0 B RAM")
    lines.append(" *")
    lines.append(" * Frame  : origin = field top-left; X right; Y down;")
    lines.append(" *          yaw 0 deg = +X, clockwise positive (== increasing yaw goes +Y)")
    lines.append(" *")
    lines.append(" * kind   : 0 = normal (corner), 1 = straight end (transition, no optical fix),")
    lines.append(" *          2 = straight middle (ONLY segment where optical may trim heading)")
    lines.append(" *")
    lines.append(" * Note   : the yaw column of nav_map.h is deliberately NOT included - it is a")
    lines.append(" *          smoothed heading and has an artifact at s=0 (reads -45 deg while the")
    lines.append(" *          first 500 mm actually run along +X). The firmware derives travel")
    lines.append(" *          direction from the map geometry (look-ahead chord) instead.")
    lines.append(" * Note   : the raw radius_mm column is also not included - it is noisy even on")
    lines.append(" *          dead-straight sections (reported R=110 mm there). kind is computed")
    lines.append(" *          offline with a chord-angle test instead.")
    lines.append(" *")
    lines.append(" * Comments kept ASCII on purpose: the C sources in this project are GBK, and")
    lines.append(" * mixing encodings in comments shows up as garbage inside Keil.")
    lines.append(" */")
    lines.append("#ifndef NAV_MAP_FW_H")
    lines.append("#define NAV_MAP_FW_H")
    lines.append("")
    lines.append("#include <stdint.h>")
    lines.append("")
    lines.append(f"#define NAV_MAP_N          {n}")
    lines.append(f"#define NAV_MAP_DS_MM      {50}")
    lines.append(f"#define NAV_MAP_LAP_MM     (NAV_MAP_N * NAV_MAP_DS_MM)")
    lines.append("#define NAV_MAP_CLOSED     1")
    lines.append("")
    lines.append("/* Start point (s = 0): where the car is placed, and which way it faces.")
    lines.append(" * Place the car at (START_X_MM, START_Y_MM) in field coords, nose along")
    lines.append(" * START_YAW_MDEG. Unit: 0.001 deg. */")
    lines.append(f"#define NAV_MAP_START_X_MM      {int(round(pts50[0][0]))}")
    lines.append(f"#define NAV_MAP_START_Y_MM      {int(round(pts50[0][1]))}")
    lines.append(f"#define NAV_MAP_START_YAW_MDEG  {yaw0_mdeg}")
    lines.append("")
    lines.append("typedef struct")
    lines.append("{")
    lines.append("    int16_t x;      /* mm, field coords */")
    lines.append("    int16_t y;      /* mm, field coords */")
    lines.append("    uint8_t kind;   /* 0 normal / 1 straight end / 2 straight middle */")
    lines.append("} nav_fw_pt_t;")
    lines.append("")
    lines.append(f"static const nav_fw_pt_t NAV_MAP[NAV_MAP_N] =")
    lines.append("{")
    for i, ((x, y), k) in enumerate(zip(pts50, kind)):
        lines.append(f"    {{{int(round(x)):5d}, {int(round(y)):5d}, {k}}},"
                     + (f"   /* {i} */" if i % 10 == 0 else ""))
    lines.append("};")
    lines.append("")
    lines.append("#endif /* NAV_MAP_FW_H */")
    lines.append("")

    out_path.write_text("\n".join(lines), encoding="ascii")

    print(f"  输出 {out_path.name}：{n} 点，单圈 {lap} mm")
    print(f"  发车点 ({int(round(pts50[0][0]))}, {int(round(pts50[0][1]))}) mm，"
          f"起始航向 {yaw0:.2f} 度")
    print(f"  kind 分布：普通 {n0} / 直道端部 {n1} / 直道中段 {n2}"
          f"（可修航向的占 {100.0*n2/n:.0f}%）")
    return n, lap


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check-only", action="store_true", help="只自检，不写文件")
    ap.add_argument("--out", default=str(OUT_PATH), help="输出头文件路径")
    args = ap.parse_args()

    print("=== 生成固件地图 ===")
    m = load_json()
    print(f"  源 {JSON_PATH.name}：{m['n_points']} 点 @ {m['resample_mm']}mm，"
          f"单圈 {m['total_len_mm']:.1f} mm")

    pts50 = decimate([tuple(p) for p in m["pts_mm"]])
    print(f"  每 {DECIMATE} 点取一 -> {len(pts50)} 点 @ 50mm")

    if not crosscheck_ref(pts50):
        sys.exit("失败：与 nav_map.h 不一致，先查数据源")

    kind = classify(pts50)
    total = len(pts50)
    # 覆盖率自检：直道不能说占了大半
    frac2 = sum(1 for k in kind if k == 2) / total
    if frac2 > 0.60:
        print(f"  [警告] 直道中段占 {100*frac2:.0f}%，阈值可能太松了")
    else:
        print(f"  [通过] 直道中段占 {100*frac2:.0f}%（限 <60%）")

    if args.check_only:
        print("=== 仅自检，未写文件 ===")
        return

    emit(pts50, kind, Path(args.out))
    print("=== 完成 ===")


if __name__ == "__main__":
    main()
