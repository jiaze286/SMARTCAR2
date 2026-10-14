"""把「赛道图-5000x5000-黑底白线.png」转成惯导可用的赛道中心线地图。

流程：
  1. 灰度阈值（>100 取线条全宽）+ 裁掉图像自身白边
  2. 形态学开运算（腐蚀 r2 → 膨胀 r2）去掉细标注线/文字/外框
  3. 取最大连通域 = 赛道本体
  4. Zhang-Suen 细化到 1 像素骨架
  5. 沿骨架"带方向行走"排序：
       · 普通路段 → 选最顺、最近的
       · 路口(>=2 候选) → ★优先选"转向与来向相反"的分支（相切点处换圆）；
                          来向接近直线时改选最直的（十字处直行）
       · 缺口 → 近距跳接（≤GAP_MAX_PX）
  6. 「漏走补接」把走丢的骨架弧补回，★连接跳变 >MAX_JUMP_MM 一律不接
  7. 方向校正：让 s 沿真实跑向递增（发车点在 S-G 净空、车头朝 S ⇒ 顶直线往 -X）
  8. s=0 挪到发车点中点
  9. 标定 mm/px（外框 862px = 5000mm）→ 等弧长重采样 → 逐点航向/曲率
 10. 自检：覆盖率 100% + 无 >MAX_JUMP_MM 的直线跳变
 11. 输出 JSON / C 头文件 / 叠加预览图

依赖：只用 numpy + PIL（无 cv2 / scipy）。
用法：python 赛道图转惯导地图.py [源图] [输出目录]
"""
import json
import math
import sys
from collections import deque
from pathlib import Path

import numpy as np
from PIL import Image

SRC = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("文档/赛道图-5000x5000-黑底白线.png")
OUT = Path(sys.argv[2]) if len(sys.argv) > 2 else Path("地图导出")
OUT.mkdir(parents=True, exist_ok=True)

# ---- 标定（实测）：场地外框像素范围 ----
FIELD_PX = dict(left=47.5, right=910.0, top=82.0, bottom=943.5)
FIELD_MM = 5000.0
SCALE = FIELD_MM / ((FIELD_PX["right"] - FIELD_PX["left"] + FIELD_PX["bottom"] - FIELD_PX["top"]) / 2.0)

RESAMPLE_MM = 10.0     # JSON/预览 重采样间隔（全精度）
HEADER_MM = 50.0       # nav_map.h 间隔（省 Flash：50mm → 约 870 点 ≈ 17KB）
GAP_MAX_PX = 17        # 行走时允许的缺口跳接（≈98mm）
MAX_JUMP_MM = 100.0    # ★任何"直线跳变"不得超过这个值（补接/连接都受此约束）
CONE_DEG = 80.0        # 前进锥角
LOOKAHEAD_STEPS = 16   # 路口判定时前探步数（≈16px≈93mm）

# 发车点：S-G 净空中点（mm，场地坐标）；车头朝 S ⇒ 沿顶直线 -X
START_X_MM = 2772.0
START_Y_MM = 203.0


# ---------------- 形态学 ----------------
def _shift(b, dy, dx, fill=False):
    out = np.full_like(b, fill)
    h, w = b.shape
    out[max(0, -dy):h - max(0, dy), max(0, -dx):w - max(0, dx)] = \
        b[max(0, dy):h - max(0, -dy), max(0, dx):w - max(0, -dx)]
    return out


def disk(r):
    y, x = np.ogrid[-r:r + 1, -r:r + 1]
    return (x * x + y * y) <= r * r


def _morph(b, r, op):
    d = disk(r)
    out = b.copy()
    for dy in range(-r, r + 1):
        for dx in range(-r, r + 1):
            if d[dy + r, dx + r]:
                out = op(out, _shift(b, dy, dx, False))
    return out


def erode(b, r=1):
    return _morph(b, r, np.logical_and)


def dilate(b, r=1):
    return _morph(b, r, np.logical_or)


def largest_component(b):
    h, w = b.shape
    lab = np.zeros((h, w), np.int32)
    best, best_n, cur = None, 0, 0
    for sy, sx in zip(*np.nonzero(b)):
        if lab[sy, sx]:
            continue
        cur += 1
        q = deque([(sy, sx)])
        lab[sy, sx] = cur
        cells = [(sy, sx)]
        while q:
            y, x = q.popleft()
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = y + dy, x + dx
                if 0 <= ny < h and 0 <= nx < w and b[ny, nx] and not lab[ny, nx]:
                    lab[ny, nx] = cur
                    q.append((ny, nx))
                    cells.append((ny, nx))
        if len(cells) > best_n:
            best_n, best = len(cells), cells
    out = np.zeros((h, w), bool)
    for y, x in best:
        out[y, x] = True
    return out


# ---------------- Zhang-Suen 细化 ----------------
def thin(img, max_iter=40):
    img = img.copy().astype(np.uint8)
    for _ in range(max_iter):
        changed = False
        for step in (0, 1):
            p = np.pad(img, 1)
            P = [p[0:-2, 1:-1], p[0:-2, 2:], p[1:-1, 2:], p[2:, 2:],
                 p[2:, 1:-1], p[2:, 0:-2], p[1:-1, 0:-2], p[0:-2, 0:-2]]
            B = sum(P)
            A = sum(((P[i] == 0) & (P[(i + 1) % 8] == 1)).astype(np.int32) for i in range(8))
            if step == 0:
                cond = (P[0] * P[2] * P[4] == 0) & (P[2] * P[4] * P[6] == 0)
            else:
                cond = (P[0] * P[2] * P[6] == 0) & (P[0] * P[4] * P[6] == 0)
            rm = (img == 1) & (B >= 2) & (B <= 6) & (A == 1) & cond
            if rm.any():
                img[rm] = 0
                changed = True
        if not changed:
            break
    return img.astype(bool)


# ---------------- 行走 ----------------
class SkelIndex:
    """骨架点的网格索引（让局部邻域查询是 O(局部)）。"""

    def __init__(self, pts_f, cell=6.0):
        self.pts = pts_f
        self.cell = cell
        self.grid = {}
        for i, (y, x) in enumerate(pts_f):
            self.grid.setdefault((int(y // cell), int(x // cell)), []).append(i)

    def near(self, p, r):
        c = self.cell
        y0, x0 = int((p[0] - r) // c), int((p[1] - r) // c)
        y1, x1 = int((p[0] + r) // c), int((p[1] + r) // c)
        out = []
        for gy in range(y0, y1 + 1):
            for gx in range(x0, x1 + 1):
                out.extend(self.grid.get((gy, gx), []))
        return out


def _lookahead_far(idx_obj, start_idx, direction, steps=34):
    """长基线前探：沿该候选分支往前往 ~34px，返回终点方向单位向量。
    相切点的两条分支要 ~30px 才能分开足够角度，短前探分不出来。"""
    pts = idx_obj.pts
    cur = pts[start_idx].copy()
    d = np.array(direction, float)
    d /= (np.linalg.norm(d) + 1e-9)
    seen = {start_idx}
    for _ in range(steps):
        best, bs, bv = None, -1e9, None
        for j in idx_obj.near(cur, 5.0):
            if j in seen:
                continue
            v = pts[j] - cur
            n = float(np.hypot(v[0], v[1]))
            if n < 0.5:
                continue
            vv = v / n
            al = float(vv @ d)
            if al < 0.55:
                continue
            s = al - 0.05 * n
            if s > bs:
                bs, best, bv = s, j, vv
        if best is None:
            break
        d = 0.5 * d + 0.5 * bv
        d /= (np.linalg.norm(d) + 1e-9)
        cur = pts[best]
        seen.add(best)
    v = cur - pts[start_idx]
    n = float(np.linalg.norm(v))
    return v / n if n > 1e-9 else None


def _cross(a, b):
    """二维叉积 z 分量：>0 表示左转（图像坐标 y 向下时为顺时针）。"""
    return a[0] * b[1] - a[1] * b[0]


FAR_PX = 15.0          # "远处候选"的门槛：相切点两条分支要到这个距离才分得开
FAR_SPREAD_DEG = 12.0  # 远处候选的方向张角超过这个值 → 认为遇到真路口


def trace(skel, start_yx, init_dir, cone_deg=CONE_DEG):
    pts = np.argwhere(skel)
    pts_f = pts.astype(float)
    used = np.zeros(len(pts), bool)
    idx_obj = SkelIndex(pts_f)

    def nearest_idx(p):
        best, bd = None, 1e9
        for j in idx_obj.near(p, 4.0):
            dd = float(np.hypot(pts_f[j, 0] - p[0], pts_f[j, 1] - p[1]))
            if dd < bd:
                bd, best = dd, j
        if best is None:
            best = int(np.argmin(np.hypot(pts_f[:, 0] - p[0], pts_f[:, 1] - p[1])))
        return int(best)

    order = [tuple(float(v) for v in start_yx)]
    used[nearest_idx(start_yx)] = True
    cur = np.array(start_yx, float)
    d = np.array(init_dir, float)
    d /= (np.linalg.norm(d) + 1e-9)
    cos_lim = math.cos(math.radians(cone_deg))
    n_junction, n_gap = 0, 0

    for _ in range(len(pts) * 3):
        good = []
        for j in idx_obj.near(cur, GAP_MAX_PX):
            if used[j]:
                continue
            v = pts_f[j] - cur
            n = float(np.hypot(v[0], v[1]))
            if n <= 0.5 or n > GAP_MAX_PX:
                continue
            vv = v / n
            al = float(vv @ d)
            if al >= cos_lim:
                good.append((j, al, n, vv))
        if not good:
            # 缺口：跳到最近的未用点（grazing 虚线用）
            pool = [j for j in idx_obj.near(cur, GAP_MAX_PX) if not used[j]]
            if not pool:
                break
            j = min(pool, key=lambda k: float(np.hypot(pts_f[k, 0] - cur[0], pts_f[k, 1] - cur[1])))
            n_gap += 1
            idx, nv = j, None
        else:
            far = [t for t in good if t[2] >= FAR_PX]
            nxt = None
            if len(far) >= 2:
                # 远处候选方向的张角：>阈值 ⇒ 真路口（相切点/十字）
                base = max(t[1] for t in far)
                dirs = [t for t in far if t[1] >= base - 0.35]
                maxang = 0.0
                for a in range(len(dirs)):
                    for b in range(a + 1, len(dirs)):
                        cc = float(np.clip(dirs[a][3] @ dirs[b][3], -1, 1))
                        maxang = max(maxang, math.degrees(math.acos(cc)))
                if maxang > FAR_SPREAD_DEG:
                    k = min(22, len(order) - 1)
                    if k >= 6:
                        back = np.array(order[-1 - k], float)
                        din = cur - back
                        din /= (np.linalg.norm(din) + 1e-9)
                        turn_in = _cross(din, d)
                    else:
                        turn_in = 0.0
                    best, bs = None, -1e18
                    for j, al, n, vv in dirs:
                        tc = _cross(d, vv)
                        if abs(turn_in) > 0.04:
                            # ★与来向相反的转向 = 相切点处换到另一个圆
                            sc = -math.copysign(1.0, turn_in) * tc + 0.05 * al
                        else:
                            # 来向接近直线 → 选最直的（十字处直行）
                            sc = -abs(tc) + 0.05 * al
                        if sc > bs:
                            bs, best = sc, (j, vv)
                    if best is not None:
                        nxt = best
                        n_junction += 1
            if nxt is None:
                g = max(good, key=lambda t: t[1] - 0.03 * t[2])
                nxt = (g[0], g[3])
            idx, nv = nxt[0], nxt[1]
        nxtp = pts_f[idx]
        v = nxtp - cur
        v /= (np.linalg.norm(v) + 1e-9)
        d = 0.75 * d + 0.25 * v
        d /= (np.linalg.norm(d) + 1e-9)
        cur = nxtp
        used[idx] = True
        order.append((nxtp[0], nxtp[1]))
    print(f"  路口判定 {n_junction} 次；缺口跳接 {n_gap} 次")
    return np.array(order)


def splice_uncovered(skel, path, tol=3.0, min_arc=5):
    """把行走漏掉的骨架弧补接回来；★连接跳变超过 MAX_JUMP_MM 的一律不接。"""
    max_jump_px = MAX_JUMP_MM / SCALE
    sy, sx = np.nonzero(skel)
    P = np.column_stack([sy, sx]).astype(float)

    def dist_to_path(pts):
        out = np.full(len(pts), 1e9)
        for i in range(0, len(path), 400):
            ch = path[i:i + 400]
            out = np.minimum(out, np.sqrt(((pts[:, None, :] - ch[None, :, :]) ** 2).sum(-1)).min(1))
        return out

    for _ in range(8):
        mind = dist_to_path(P)
        unc = P[mind > tol]
        if len(unc) < min_arc:
            break
        compset = {(int(a), int(b)) for a, b in unc}

        def nbrs(p):
            return [(p[0] + dy, p[1] + dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1)
                    if (dy or dx) and (p[0] + dy, p[1] + dx) in compset]

        seen, arcs = set(), []
        for p in compset:
            if p in seen:
                continue
            stack, comp = [p], []
            seen.add(p)
            while stack:
                q = stack.pop()
                comp.append(q)
                for r in nbrs(q):
                    if r not in seen:
                        seen.add(r)
                        stack.append(r)
            if len(comp) >= min_arc:
                arcs.append(comp)
        if not arcs:
            break
        comp = max(arcs, key=len)
        ends = [q for q in comp if len(nbrs(q)) <= 1]
        cur = ends[0] if ends else comp[0]
        order, vis = [], set()
        while True:
            order.append(cur)
            vis.add(cur)
            nxt = [q for q in nbrs(cur) if q not in vis]
            if not nxt:
                break
            cur = nxt[0]
        arc = np.array(order, float)
        d0 = np.sqrt(((path - arc[0]) ** 2).sum(1))
        d1 = np.sqrt(((path - arc[-1]) ** 2).sum(1))
        cands = []
        # 4 种摆法：插在 i0 或 i1 之后 × 弧的正反 —— 取连接最短的
        for i in (int(np.argmin(d0)), int(np.argmin(d1))):
            if i + 1 >= len(path):
                continue
            for rev in (False, True):
                a = arc[::-1] if rev else arc
                c0 = float(np.hypot(a[0, 0] - path[i, 0], a[0, 1] - path[i, 1]))
                c1 = float(np.hypot(a[-1, 0] - path[i + 1, 0], a[-1, 1] - path[i + 1, 1]))
                cands.append((max(c0, c1), i, a))
        if not cands:
            break
        worst, i, a = min(cands, key=lambda t: t[0])
        if worst > max_jump_px:
            print(f"  ⚠ 放弃补接一段 {len(arc)} 点：最短连接跳变 {worst*SCALE:.0f}mm "
                  f"> {MAX_JUMP_MM:.0f}mm")
            break
        path = np.vstack([path[:i + 1], a, path[i + 1:]])
        print(f"  补接一段 {len(arc)} 点（第 {i} 点处，连接 {worst*SCALE:.0f}mm）")
    return path


# ---------------- E 区（4 个相切圆）解析重建 ----------------
def fit_circle(P):
    """代数最小二乘拟合圆，返回 (cx, cy, r)。"""
    x, y = P[:, 0], P[:, 1]
    A = np.column_stack([x, y, np.ones(len(x))])
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = sol[0] / 2, sol[1] / 2
    r = math.sqrt(sol[2] + cx * cx + cy * cy)
    return cx, cy, r


def replace_E(path, skel, centers_px, win=48.0):
    """把 E 区那段换成解析生成的蛇形线。

    走法（按真实跑向）：第 4 圆右下进 → 4 右半(逆时针) → 3 左半(顺时针)
                      → 2 右半(逆时针) → 1 左半(顺时针) → 第 1 圆顶点出去。
    角度 φ 以"12 点为零、顺时针为正"：P(φ) = (cx + R sinφ, cy - R cosφ)
    """
    sy, sx = np.nonzero(skel)
    P = np.column_stack([sx, sy]).astype(float)
    circles = []
    for (cy0, cx0) in centers_px:
        sel = (np.abs(P[:, 1] - cy0) < win) & (np.abs(P[:, 0] - cx0) < win)
        if sel.sum() < 40:
            print("  ⚠ E 区圆拟合点数不足，跳过解析重建")
            return path
        cx, cy, r = fit_circle(P[sel])
        circles.append((cx, cy, r))
    R = float(np.mean([c[2] for c in circles]))
    print("  E 区圆拟合：", "  ".join(f"({c[0]:.1f},{c[1]:.1f},R{c[2]:.1f})" for c in circles),
          f" → 取 R={R:.1f}px = {R*SCALE:.0f}mm")

    def pt(ci, deg):
        cx, cy, _ = circles[ci]
        a = math.radians(deg)
        return np.array([cx + R * math.sin(a), cy - R * math.cos(a)])

    def arc(ci, d0, d1, cw):
        """从 d0 度到 d1 度（cw=True 顺时针增角），按 ~1px 步长采样。"""
        span = (d1 - d0) % 360 if cw else -((d0 - d1) % 360)
        n = max(8, int(abs(math.radians(span)) * R))
        return np.array([pt(ci, d0 + span * k / n) for k in range(n + 1)])

    serp = [arc(3, 135.0, 0.0, False),      # 第4圆：右下 → 顶点（逆时针）
            arc(2, 180.0, 360.0, True),     # 第3圆：底 → 顶（顺时针，走左半）
            arc(1, 180.0, 0.0, False),      # 第2圆：底 → 顶（逆时针，走右半）
            arc(0, 180.0, 360.0, True)]     # 第1圆：底 → 顶（顺时针，走左半）
    serp = np.vstack(serp)                   # (x,y)

    # 找路径上离蛇形线两端最近的点，替换中间那段
    a0, a1 = serp[0], serp[-1]
    da = np.hypot(path[:, 1] - a0[0], path[:, 0] - a0[1])
    db = np.hypot(path[:, 1] - a1[0], path[:, 0] - a1[1])
    ia, ib = int(np.argmin(da)), int(np.argmin(db))
    if da[ia] > 60 or db[ib] > 60:
        print(f"  ⚠ E 区端点到路径 {max(da[ia], db[ib])*SCALE:.0f}mm，跳过解析重建")
        return path
    if ia == ib:
        print("  ⚠ E 区两端落在同一点，跳过")
        return path
    a = np.column_stack([serp[:, 1], serp[:, 0]])      # → (y,x)
    if ia < ib:
        new = np.vstack([path[:ia + 1], a, path[ib:]])
    else:
        new = np.vstack([path[:ib + 1], a[::-1], path[ia:]])
    print(f"  E 区解析重建：替换路径第 {min(ia,ib)}..{max(ia,ib)} 点（{abs(ib-ia)} 点 → {len(a)} 点）")
    return new


def main():
    rgb = Image.open(SRC).convert("RGB")
    g = np.array(rgb.convert("L"))
    h, w = g.shape
    print(f"源图 {w}x{h}  标定 {SCALE:.4f} mm/px")

    inner = np.zeros_like(g, bool)
    inner[40:926, 40:896] = True
    white = (g > 100) & inner

    opened = dilate(erode(white, 2), 2)
    track = largest_component(opened)
    print("赛道掩膜像素:", int(track.sum()))

    skel = thin(track.T).T
    print("骨架像素:", int(skel.sum()))
    Image.fromarray((skel * 255).astype(np.uint8)).save(OUT / "_dbg_skel.png")

    ys, xs = np.nonzero(skel)
    s0 = int(np.argmin(ys * 10000 + xs))
    start = (ys[s0], xs[s0])
    near = np.argwhere(skel[max(0, start[0] - 6):start[0] + 7, max(0, start[1] - 6):start[1] + 7])
    init = (1, 0)
    if len(near) > 1:
        m = near.mean(0)
        init = (m[0] - 6, m[1] - 6)
        if np.hypot(*init) < 0.5:
            init = (1, 0)
    print("起点 (y,x) =", start, " 初始方向", init)

    path = trace(skel, start, init)
    print("行走点数:", len(path))
    path = splice_uncovered(skel, path)
    print("补接后点数:", len(path))
    # E 区（4 个相切圆）解析重建：不靠行走器的分支选择
    path = replace_E(path, skel, [(290, 617), (390, 617), (490, 617), (590, 617)])
    print("E 区重建后点数:", len(path))

    # ---- 像素 → 毫米（场地坐标系：原点=场地左上角，X 右，Y 下）----
    mm = np.column_stack([
        (path[:, 1] - FIELD_PX["left"]) * SCALE,
        (path[:, 0] - FIELD_PX["top"]) * SCALE,
    ])

    # ---- 方向校正：发车点处应沿 -X 前进 ----
    d_start = np.hypot(mm[:, 0] - START_X_MM, mm[:, 1] - START_Y_MM)
    i_start = int(np.argmin(d_start))
    n = len(mm)
    dx_next = mm[(i_start + 1) % n, 0] - mm[i_start, 0]
    if dx_next > 0:
        mm = mm[::-1]
        # 反向索引后重新定位发车点
        d_start = np.hypot(mm[:, 0] - START_X_MM, mm[:, 1] - START_Y_MM)
        i_start = int(np.argmin(d_start))
        i_start = (n - 1 - i_start)
        d_start = np.hypot(mm[:, 0] - START_X_MM, mm[:, 1] - START_Y_MM)
        i_start = int(np.argmin(d_start))
        print("整表反向：s 现在沿真实跑向（顶直线 -X）递增")
    else:
        print("方向已正确（顶直线 -X）")

    # ---- s=0 挪到发车点 ----
    mm = np.roll(mm, -i_start, axis=0)
    print(f"s=0 设在发车点 (px 距离 {np.hypot(mm[0,0]-START_X_MM, mm[0,1]-START_Y_MM):.0f}mm 内)")
    print(f"  起点 = ({mm[0,0]:.0f}, {mm[0,1]:.0f}) mm")

    # ---- 自检 1：无 >MAX_JUMP_MM 的直线跳变 ----
    step = np.hypot(np.diff(mm[:, 0]), np.diff(mm[:, 1]))
    max_step = float(step.max()) if len(step) else 0.0
    ok_jump = max_step <= MAX_JUMP_MM + 1e-6
    print(f"[自检] 最大单步跳变 {max_step:.1f} mm （限 {MAX_JUMP_MM:.0f}）→ "
          f"{'通过' if ok_jump else '★不通过'}")

    # ---- 等弧长重采样 ----
    seg = np.hypot(np.diff(mm[:, 0]), np.diff(mm[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1] + np.hypot(mm[0, 0] - mm[-1, 0], mm[0, 1] - mm[-1, 1])
    nre = max(int(total / RESAMPLE_MM), 8)
    s_new = np.linspace(0, total, nre, endpoint=False)
    mm_c = np.vstack([mm, mm[:1]])
    s_c = np.concatenate([s, [total]])
    x_new = np.interp(s_new, s_c, mm_c[:, 0])
    y_new = np.interp(s_new, s_c, mm_c[:, 1])

    yaw = np.zeros(nre)
    R = np.zeros(nre)
    k = max(3, int(150 / RESAMPLE_MM))
    for i in range(nre):
        a, b = (i - k) % nre, (i + k) % nre
        dx, dy = x_new[b] - x_new[a], y_new[b] - y_new[a]
        yaw[i] = math.degrees(math.atan2(dy, dx))
        ax, ay = x_new[(i - k) % nre], y_new[(i - k) % nre]
        bx, by = x_new[i], y_new[i]
        cx, cy = x_new[(i + k) % nre], y_new[(i + k) % nre]
        A = bx - ax; Bc = by - ay; C = cx - ax; D = cy - ay
        E = A * (ax + bx) + Bc * (ay + by)
        F = C * (ax + cx) + D * (ay + cy)
        G = 2 * (A * (cy - by) - Bc * (cx - bx))
        if abs(G) < 1e-9:
            R[i] = 1e9
        else:
            ux = (D * E - Bc * F) / G
            uy = (A * F - C * E) / G
            R[i] = math.hypot(ax - ux, ay - uy)

    # ---- 自检 2：覆盖率（对交付折线做，10mm 采样 ≈ 连续折线）----
    sy, sx = np.nonzero(skel)
    P = np.column_stack([sx, sy]).astype(float)
    back = np.column_stack([x_new / SCALE + FIELD_PX["left"], y_new / SCALE + FIELD_PX["top"]])
    mind = np.full(len(P), 1e9)
    for i in range(0, len(back), 400):
        ch = back[i:i + 400]
        mind = np.minimum(mind, np.sqrt(((P[:, None, :] - ch[None, :, :]) ** 2).sum(-1)).min(1))
    # E 区（4 个相切圆）故意只走半边，另半边是"车不走的分支" → 覆盖率统计排除该框
    EX, EY0, EY1 = (540.0, 690.0), 215.0, 665.0
    inE = (P[:, 0] >= EX[0]) & (P[:, 0] <= EX[1]) & (P[:, 1] >= EY0) & (P[:, 1] <= EY1)
    unc_all = int((mind > 2).sum())
    unc_out = int(((mind > 2) & (~inE)).sum())
    print(f"[自检] 骨架未覆盖点（对交付折线）：全区 {unc_all}，**E 区之外 {unc_out}** / {len(P)} → "
          f"{'通过' if unc_out == 0 else '★不通过'}")
    print(f"        （E 区内 {unc_all - unc_out} 点是设计上不走的另半边圆，正常）")
    if unc_out:
        uu = P[(mind > 2) & (~inE)]
        print(f"        E 区外未覆盖点样本(px): {uu[::max(1, len(uu)//8)][:8].astype(int).tolist()}")
    if not (ok_jump and unc_out == 0):
        print("⚠ 自检未过，产物仍会写出，但请先查预览图")

    info = dict(
        source=SRC.name, scale_mm_per_px=round(SCALE, 5), resample_mm=RESAMPLE_MM,
        closed=True, total_len_mm=round(float(total), 1), n_points=nre,
        start_mm=[round(START_X_MM, 1), round(START_Y_MM, 1)],
        direction="s 递增 = 真实跑向：发车点在 S-G 净空、车头朝 S ⇒ 顶直线沿 -X",
        frame="原点=场地左上角; X向右; Y向下(与赛道图同向); yaw: 0°=+X, 顺时针为正",
        checks=dict(max_jump_mm=round(max_step, 2),
                    uncovered_pts_all=unc_all, uncovered_pts_outside_E=unc_out),
        pts_mm=[[round(float(a), 1), round(float(b), 1)] for a, b in zip(x_new, y_new)],
        yaw_deg=[round(float(v), 2) for v in yaw],
        radius_mm=[None if v > 1e8 else round(float(v), 1) for v in R],
    )
    (OUT / "赛道中心线-mm.json").write_text(json.dumps(info, ensure_ascii=False), encoding="utf-8")

    # ---- C 头文件 ----
    stp = max(1, int(round(HEADER_MM / RESAMPLE_MM)))
    sel = np.arange(0, nre, stp)
    nh = len(sel)
    lines = [
        "/* 赛道中心线（由 赛道图-5000x5000-黑底白线.png 自动提取，勿手改）",
        f" * 标定 {SCALE:.4f} mm/px；等弧长重采样 {HEADER_MM:.0f}mm；共 {nh} 点；闭环=1",
        f" * 周长 ≈ {total/1000:.2f} m；本表占 Flash ≈ {nh*20/1024:.1f} KB（float×5）",
        " * 坐标系：原点 = 场地左上角；X 向右；Y 向下（与赛道图同向）",
        " * yaw_deg：0° = +X 方向，顺时针为正；radius_mm：直线段填 1e9",
        " * ★s=0 = 发车点（S-G 净空中点）；s 递增方向 = 真实跑向（车头朝 S，顶直线 -X）",
        " * 同目录 赛道中心线-mm.json 是 10mm 全精度版，可用脚本重生成任意间隔",
        " */",
        "#ifndef NAV_MAP_H",
        "#define NAV_MAP_H",
        "",
        "#define NAV_MAP_N          %d" % nh,
        "#define NAV_MAP_RESAMPLE_MM %.1ff" % HEADER_MM,
        "#define NAV_MAP_CLOSED     1",
        "",
        "typedef struct { float s_mm, x_mm, y_mm, yaw_deg, radius_mm; } nav_pt_t;",
        "",
        "static const nav_pt_t NAV_MAP[NAV_MAP_N] = {",
    ]
    for i in sel:
        r = "1e9f" if R[i] > 1e8 else f"{R[i]:.1f}f"
        lines.append(f"  {{{s_new[i]:8.1f}f,{x_new[i]:8.1f}f,{y_new[i]:8.1f}f,{yaw[i]:8.2f}f,{r:>9}}},")
    lines += ["};", "", "#endif /* NAV_MAP_H */", ""]
    (OUT / "nav_map.h").write_text("\n".join(lines), encoding="utf-8")

    # ---- 预览 ----
    px = np.array(rgb)
    for i in range(nre):
        cx = int(round(x_new[i] / SCALE + FIELD_PX["left"]))
        cy = int(round(y_new[i] / SCALE + FIELD_PX["top"]))
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                yy, xx = cy + dy, cx + dx
                if 0 <= yy < px.shape[0] and 0 <= xx < px.shape[1]:
                    px[yy, xx] = (255, 40, 40)
    for i in range(0, nre, max(1, nre // 45)):
        cx = int(round(x_new[i] / SCALE + FIELD_PX["left"]))
        cy = int(round(y_new[i] / SCALE + FIELD_PX["top"]))
        a = math.radians(yaw[i])
        for t in range(1, 17):
            yy = int(round(cy + math.sin(a) * t))
            xx = int(round(cx + math.cos(a) * t))
            if 0 <= yy < px.shape[0] and 0 <= xx < px.shape[1]:
                px[yy, xx] = (60, 255, 60)
    # 发车点标记（蓝方块）
    cx = int(round(START_X_MM / SCALE + FIELD_PX["left"]))
    cy = int(round(START_Y_MM / SCALE + FIELD_PX["top"]))
    px[max(0, cy - 4):cy + 5, max(0, cx - 4):cx + 5] = (60, 130, 255)
    Image.fromarray(px).save(OUT / "赛道中心线-预览.png")

    print(f"完成：{nre} 点，周长 {total/1000:.2f} m，闭环")
    print(f"输出：{OUT}/赛道中心线-mm.json  nav_map.h  赛道中心线-预览.png")


if __name__ == "__main__":
    main()
