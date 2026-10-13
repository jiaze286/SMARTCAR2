"""把「赛道图-5000x5000-黑底白线.png」转成惯导可用的赛道中心线地图。

流程：
  1. 灰度阈值（>100 取线条全宽）+ 裁掉图像自身白边
  2. 形态学开运算（腐蚀 r2 → 膨胀 r2）去掉细标注线/文字/外框
  3. 取最大连通域 = 赛道本体
  4. Zhang-Suen 细化到 1 像素骨架
  5. 沿骨架"带方向行走"排序（十字路口选直行；小缺口自动跳）
  6. 像素 → 毫米标定：外框 862px = 场地 5000mm ⇒ 5.80 mm/px
  7. 等弧长重采样 + 逐点航向/曲率半径
  8. 输出 JSON / C 头文件 / 叠加预览图

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

# ---- 标定（实测，见文件头注释）：场地外框像素范围 ----
FIELD_PX = dict(left=47.5, right=910.0, top=82.0, bottom=943.5)
FIELD_MM = 5000.0
SCALE = FIELD_MM / ((FIELD_PX["right"] - FIELD_PX["left"] + FIELD_PX["bottom"] - FIELD_PX["top"]) / 2.0)

RESAMPLE_MM = 10.0     # JSON/预览 的重采样间隔（全精度）
HEADER_MM = 50.0       # nav_map.h 的间隔（省 Flash：50mm → 约 870 点 ≈ 17KB）
GAP_MAX_PX = 22        # 允许跳过的缺口（像素）
CONE_DEG = 80.0        # 前进锥角


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


# ---------------- 沿骨架行走排序 ----------------
def trace(skel, start_yx, init_dir, cone_deg=CONE_DEG):
    pts = np.argwhere(skel)                       # (N,2) [y,x]
    pts_f = pts.astype(float)
    used = np.zeros(len(pts), bool)

    def nearest_idx(p):
        d = np.hypot(pts_f[:, 0] - p[0], pts_f[:, 1] - p[1])
        return int(np.argmin(d))

    order = [tuple(start_yx)]
    used[nearest_idx(start_yx)] = True
    cur = np.array(start_yx, float)
    d = np.array(init_dir, float)
    d /= (np.linalg.norm(d) + 1e-9)
    cos_lim = math.cos(math.radians(cone_deg))

    for _ in range(len(pts) * 3):
        rel = pts_f - cur
        dist = np.hypot(rel[:, 0], rel[:, 1])
        with np.errstate(invalid="ignore", divide="ignore"):
            unit = rel / dist[:, None]
        align = unit @ d
        cand = (~used) & (dist > 0.5) & (dist <= GAP_MAX_PX) & (align >= cos_lim)
        if not cand.any():
            # 放宽：只按距离找最近的未用点（跳缺口）
            cand2 = (~used) & (dist > 0.5) & (dist <= GAP_MAX_PX)
            if not cand2.any():
                break
            idx = int(np.argmin(np.where(cand2, dist, 1e9)))
        else:
            score = align - 0.03 * dist          # 越顺、越近越好
            idx = int(np.argmax(np.where(cand, score, -1e9)))
        nxt = pts_f[idx]
        v = nxt - cur
        v /= (np.linalg.norm(v) + 1e-9)
        d = 0.75 * d + 0.25 * v                  # 方向平滑
        d /= (np.linalg.norm(d) + 1e-9)
        cur = nxt
        used[idx] = True
        order.append((nxt[0], nxt[1]))
    return np.array(order)


def splice_uncovered(skel, path, tol=3.0, max_gap=20.0, min_arc=5):
    """把行走时漏掉的骨架弧补接回路径（在相切点/路口被走丢的分支）。"""
    sy, sx = np.nonzero(skel)
    P = np.column_stack([sy, sx]).astype(float)

    def dist_to_path(pts):
        out = np.full(len(pts), 1e9)
        for i in range(0, len(path), 400):
            ch = path[i:i + 400]
            out = np.minimum(out, np.sqrt(((pts[:, None, :] - ch[None, :, :]) ** 2).sum(-1)).min(1))
        return out

    for _ in range(6):                       # 最多补 6 段
        mind = dist_to_path(P)
        unc = P[mind > tol]
        if len(unc) < min_arc:
            break
        compset = {(int(a), int(b)) for a, b in unc}
        idx = {p: i for i, p in enumerate(compset)}

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
        # 取最大的那段补
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
        i0, i1 = int(np.argmin(d0)), int(np.argmin(d1))
        if d0[i0] > max_gap and d1[i1] > max_gap:
            # 两端都离路径太远：放弃这段，避免接出乱线
            break
        if i1 < i0:
            arc = arc[::-1]
        path = np.vstack([path[:i0 + 1], arc, path[i0 + 1:]])
        print(f"  补接一段 {len(arc)} 点（接在路径第 {i0} 点处）")
    return path


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

    skel = thin(track.T).T                       # 细化（转置避开 numpy 形状坑）
    print("骨架像素:", int(skel.sum()))
    Image.fromarray((skel * 255).astype(np.uint8)).save(OUT / "_dbg_skel.png")

    ys, xs = np.nonzero(skel)
    s0 = int(np.argmin(ys * 10000 + xs))          # 最上最左的骨架点作起点
    start = (ys[s0], xs[s0])
    # 初始方向：附近骨架点的走向
    near = np.argwhere(skel[max(0, start[0] - 6):start[0] + 7, max(0, start[1] - 6):start[1] + 7])
    init = (2, 2)
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

    # 像素 → 毫米（场地坐标系：原点 = 场地左上角，X 右，Y 下；与赛道图一致）
    mm = np.column_stack([
        (path[:, 1] - FIELD_PX["left"]) * SCALE,
        (path[:, 0] - FIELD_PX["top"]) * SCALE,
    ])

    # 等弧长重采样
    seg = np.hypot(np.diff(mm[:, 0]), np.diff(mm[:, 1]))
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    n = max(int(total / RESAMPLE_MM), 8)
    s_new = np.linspace(0, total, n, endpoint=False)
    x_new = np.interp(s_new, s, mm[:, 0])
    y_new = np.interp(s_new, s, mm[:, 1])

    # 航向 & 曲率
    yaw = np.zeros(n)
    R = np.zeros(n)
    k = max(3, int(150 / RESAMPLE_MM))            # ±150mm 拟合
    for i in range(n):
        a, b = (i - k) % n, (i + k) % n
        dx, dy = x_new[b] - x_new[a], y_new[b] - y_new[a]
        yaw[i] = math.degrees(math.atan2(dy, dx))
        # 三点拟合圆半径
        ax, ay = x_new[(i - k) % n], y_new[(i - k) % n]
        bx, by = x_new[i], y_new[i]
        cx, cy = x_new[(i + k) % n], y_new[(i + k) % n]
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

    closed = math.hypot(mm[-1, 0] - mm[0, 0], mm[-1, 1] - mm[0, 1]) < 200.0

    info = dict(
        source=SRC.name, scale_mm_per_px=round(SCALE, 5), resample_mm=RESAMPLE_MM,
        closed=bool(closed), total_len_mm=round(float(total), 1), n_points=n,
        frame="原点=场地左上角; X向右; Y向下(与赛道图同向); yaw: 0°=+X, 顺时针为正",
        pts_mm=[[round(float(a), 1), round(float(b), 1)] for a, b in zip(x_new, y_new)],
        yaw_deg=[round(float(v), 2) for v in yaw],
        radius_mm=[None if v > 1e8 else round(float(v), 1) for v in R],
    )
    (OUT / "赛道中心线-mm.json").write_text(json.dumps(info, ensure_ascii=False), encoding="utf-8")

    # ---- C 头文件（按 HEADER_MM 抽稀，省 Flash；等弧长重采样后直接隔点取）----
    step = max(1, int(round(HEADER_MM / RESAMPLE_MM)))
    sel = np.arange(0, n, step)
    nh = len(sel)
    lines = [
        "/* 赛道中心线（由 赛道图-5000x5000-黑底白线.png 自动提取，勿手改）",
        f" * 标定 {SCALE:.4f} mm/px；等弧长重采样 {HEADER_MM:.0f}mm；共 {nh} 点；闭环={closed}",
        f" * 周长 ≈ {total/1000:.2f} m；本表占 Flash ≈ {nh*20/1024:.1f} KB（float×5）",
        " * 坐标系：原点 = 场地左上角；X 向右；Y 向下（与赛道图同向）",
        " * yaw_deg：0° = +X 方向，顺时针为正；radius_mm：直线段填 1e9",
        " * 同目录 赛道中心线-mm.json 是 10mm 全精度版，可用脚本重生成任意间隔",
        " */",
        "#ifndef NAV_MAP_H",
        "#define NAV_MAP_H",
        "",
        "#define NAV_MAP_N          %d" % nh,
        "#define NAV_MAP_RESAMPLE_MM %.1ff" % HEADER_MM,
        "#define NAV_MAP_CLOSED     %d" % (1 if closed else 0),
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

    # ---- 预览：原图 + 提取中心线 + 方向箭头 ----
    prev = rgb.copy()
    px = np.array(prev)
    for i in range(n):
        cx = int(round(x_new[i] / SCALE + FIELD_PX["left"]))
        cy = int(round(y_new[i] / SCALE + FIELD_PX["top"]))
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                yy, xx = cy + dy, cx + dx
                if 0 <= yy < px.shape[0] and 0 <= xx < px.shape[1]:
                    px[yy, xx] = (255, 40, 40)
    for i in range(0, n, max(1, n // 40)):
        cx = int(round(x_new[i] / SCALE + FIELD_PX["left"]))
        cy = int(round(y_new[i] / SCALE + FIELD_PX["top"]))
        a = math.radians(yaw[i])
        for t in range(1, 16):
            yy = int(round(cy + math.sin(a) * t))
            xx = int(round(cx + math.cos(a) * t))
            if 0 <= yy < px.shape[0] and 0 <= xx < px.shape[1]:
                px[yy, xx] = (60, 255, 60)
    Image.fromarray(px).save(OUT / "赛道中心线-预览.png")

    print(f"完成：{n} 点，周长 {total/1000:.2f} m，闭环={closed}")
    print(f"输出：{OUT}/赛道中心线-mm.json  nav_map.h  赛道中心线-预览.png")


if __name__ == "__main__":
    main()
