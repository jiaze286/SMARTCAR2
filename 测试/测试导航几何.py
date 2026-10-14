#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
导航几何 / 符号链 离线验算 —— 不接板子就能跑。

为什么要有这个文件：
  惯导最容易错、又最难在板子上看出来的东西是【符号约定】。推错一个符号，
  车会朝背离赛道的一侧一路打死，而上板时你只看到"车甩出去了"，
  根本不知道是哪一环反了。所以这里把整条链路在电脑上先跑通：
      地图几何 -> 前瞻点 -> 横向分量 lat10 -> E: 量纲的 err
      -> PD 出 corr -> 差速 -> 车头角速度 -> 回到地图几何
  最后一环"差速->角速度"是【独立实现】的（差速车物理，不抄固件），
  所以它能验出固件里的符号错。

覆盖：
  1. 地图数据自检（点数 / 闭环 / 单圈 / kind 分布 / 起始航向）
  2. 车停在中心线上 -> err ≈ 0
  3. 车偏到【自己右边】-> err < 0（该左转回来）；偏左 -> err > 0
  4. 闭环仿真：给一个横向+航向扰动，看车能不能回到中心线（而不是冲出去）
  5. 反证：把 NAV_YAW_SIGN 写成 +1 应该发散 —— 证明"只有 -1 是对的"

跑法：
    python 测试/测试导航几何.py
"""

import math
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPO = HERE.parent
MAP_H = REPO / "User" / "nav_map_fw.h"

# ---- 与固件保持一致的常量（改固件要同步改这里；文件末尾会自查一遍）----
DS_MM = 50
GYRO_DB = 2.0            # NAV_GYRO_DB
ERR_FILT_N = 8           # NAV_ERR_FILT_N
LOOK_MIN = 2             # NAV_LOOK_MIN
PWM_MAX = 7199           # MOTOR_SPEED_MAX
KP = 70.0                # g_kp
KD = 0.0                 # g_kd
KGYRO = 2.0              # g_nav_kgyro
VMAX = 3900.0            # g_nav_vmax
LOOK_MM = 150.0          # g_nav_look_mm
MM_PER_E = 1.0           # g_nav_mm_per_e

# ---- 仿真用（只在模型里出现，不是固件参数）----
WHEELBASE_MM = 100.0     # 后轮轮距（未知量，下面扫几个值都要求收敛）
PWM_TO_MMPS = 0.154      # PWM -> mm/s 的粗略换算，只为让仿真量级像真的

DEG = 180.0 / math.pi


# ======================================================================
# 地图
# ======================================================================
def load_map():
    txt = MAP_H.read_text(encoding="ascii")
    n = int(re.search(r"#define\s+NAV_MAP_N\s+(\d+)", txt).group(1))
    ds = int(re.search(r"#define\s+NAV_MAP_DS_MM\s+(\d+)", txt).group(1))
    yaw0 = int(re.search(r"#define\s+NAV_MAP_START_YAW_MDEG\s+(-?\d+)", txt).group(1))
    pts = [(int(a), int(b), int(k)) for a, b, k in
           re.findall(r"\{\s*(-?\d+),\s*(-?\d+),\s*(\d+)\}", txt)]
    assert ds == DS_MM, f"nav_map_fw.h 的 DS={ds}，与测试里的 {DS_MM} 不一致"
    assert len(pts) == n, f"解析出 {len(pts)} 点，头文件说 {n} 点"
    return pts, yaw0


def chord_yaw(pts, i, k=10):
    """地图上 i -> i+k 的行进方向（度）。地图坐标系：0 度 = +X，yaw 增大朝 +Y。"""
    n = len(pts)
    a = pts[i % n]
    b = pts[(i + k) % n]
    return math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))


# ======================================================================
# 固件里那几步（1:1 转写；改 NAV.c 要同步改这里）
# ======================================================================
def nav_lat10(pts, i, j, cx, cy, yaw_deg):
    """前瞻点 j 相对车、沿【车的右侧】的分量，0.1mm。
    lat10 = (-sin, cos)·(dx, dy)，而 (-sin, cos) 是车头方向再转 +90 度 = 车的右侧。"""
    n = len(pts)
    dx = pts[j % n][0] * 10 - cx
    dy = pts[j % n][1] * 10 - cy
    s = math.sin(math.radians(yaw_deg))
    c = math.cos(math.radians(yaw_deg))
    # 固件里是 Q15（sin/cos 存成 x32768、最后 /32768）；这里用浮点，分子分母
    # 正好抵消，所以直接算即可。单位 0.1mm。
    return (-s * dx + c * dy)


def nav_err(lat10, mm_per_e=MM_PER_E):
    """换到 E: 的量纲。lat10>0 = 目标在车右边 = 线在右边 -> err>0（该右转）。"""
    per = mm_per_e if mm_per_e > 0 else 1
    return lat10 / 10.0 / per


def nav_steer(pts, odo_mm, cx10, cy10, yaw_deg, ydot_dps, err_f16, last_err, kp=KP):
    """NAV_Run 的纯计算部分 -> (l, r, err, err_f16, last_err)。"""
    n = len(pts)
    idx = int(odo_mm // DS_MM) % n
    k = max(LOOK_MIN, int(LOOK_MM // DS_MM))
    j = (idx + k) % n

    lat10 = nav_lat10(pts, idx, j, cx10, cy10, yaw_deg)
    err = nav_err(lat10)
    err = max(-50.0, min(50.0, err))

    err_f16 += (err * 16.0 - err_f16) / ERR_FILT_N
    err = err_f16 / 16.0

    corr = -(kp * err + KD * (err - last_err))
    corr += KGYRO * ydot_dps          # 阻尼：NAV.c 里的 corr += k*Nav_YDot()
    l = max(0.0, min(float(PWM_MAX), VMAX - corr))
    r = max(0.0, min(float(PWM_MAX), VMAX + corr))
    return l, r, err, err_f16, err


# ======================================================================
# 差速车物理（独立实现 —— 验符号的关键一环，不抄固件）
# ======================================================================
def kinematics(v_l_pwm, v_r_pwm, wheelbase_mm):
    """返回 (前进速度 mm/s, 偏航角速度 度/秒)。
    约定：yaw_dot > 0 = 车右转（= 地图 yaw 增大）。
    物理：左轮快 -> 车右转 -> yaw_dot > 0。"""
    v_l = v_l_pwm * PWM_TO_MMPS
    v_r = v_r_pwm * PWM_TO_MMPS
    v = 0.5 * (v_l + v_r)
    yaw_dot_cw = (v_l - v_r) / wheelbase_mm * DEG      # 左快 -> 正
    return v, yaw_dot_cw


def gyro_reading(yaw_dot_cw, yaw_sign):
    """固件里 IMU 读到的是 ω_raw，之后乘 NAV_YAW_SIGN 得到"右转为正"。
    所以 ω_raw = yaw_dot_cw / NAV_YAW_SIGN。"""
    return yaw_dot_cw / yaw_sign


def simulate(pts, yaw_sign, kp, lat_off_mm=30.0, yaw_off_deg=8.0, sim_s=3.0,
             dt=0.005, wheelbase_mm=WHEELBASE_MM, start_idx=200):
    """闭环仿真：从中心线第 start_idx 点出发，给横向 + 航向扰动，看收不收敛。
    返回 (最终横向偏差mm, 过程最大横向偏差mm, 是否发散)。

    kp 是参数：这个测试验的是【符号】，不是增益。增益多大合适是调参器的事；
       这里只要求"符号对的时候存在一个能让车回到线上的增益"。"""
    n = len(pts)
    base_yaw = chord_yaw(pts, start_idx)

    bx, by = pts[start_idx][0], pts[start_idx][1]
    u = (-math.sin(math.radians(base_yaw)), math.cos(math.radians(base_yaw)))  # 车的右侧
    x = bx + u[0] * lat_off_mm
    y = by + u[1] * lat_off_mm
    yaw = base_yaw + yaw_off_deg

    odo = start_idx * DS_MM     # 【关键】里程要和起点下标对上：
                                #  固件里 NAV_StartRun 是"里程 0 + 位姿=地图第 0 点"，
                                #  测试从第 start_idx 点起步，里程就得从那儿起算。
                                #  忘了这一步，惯导会以为自己在第 0 点，
                                #  前瞻点跑到赛道另一头，直接满舵冲出去。
    err_f16 = 0.0
    last_err = 0.0
    ydot_true = 0.0                      # 上一拍的真实角速度（局部变量，别用全局）
    worst = abs(lat_off_mm)
    off = lat_off_mm

    for _ in range(int(sim_s / dt)):
        ydot = gyro_reading(ydot_true, yaw_sign)
        if abs(ydot) < GYRO_DB:          # 死区
            ydot = 0.0
        else:
            ydot -= math.copysign(GYRO_DB, ydot)

        l, r, err, err_f16, last_err = nav_steer(
            pts, odo, x * 10.0, y * 10.0, yaw, ydot, err_f16, last_err, kp)

        v, ydot_true = kinematics(l, r, wheelbase_mm)

        yaw += ydot_true * dt
        x += v * math.cos(math.radians(yaw)) * dt
        y += v * math.sin(math.radians(yaw)) * dt
        odo += v * dt

        idx = int(odo // DS_MM) % n
        px, py = pts[idx][0], pts[idx][1]
        ang = chord_yaw(pts, idx)
        uu = (-math.sin(math.radians(ang)), math.cos(math.radians(ang)))
        off = (x - px) * uu[0] + (y - py) * uu[1]
        worst = max(worst, abs(off))

        if abs(off) > 400.0:             # 跑出 400mm 就算发散
            return off, worst, True

    return off, worst, False


# ======================================================================
# 位置积分的整数截断（浮点测试抓不到的那一类 bug）
# ======================================================================
def q15_cos(yaw_deg):
    """与固件 Nav_Cos 同量级的 Q15 余弦。整数域测试只需要同一个量级。"""
    return round(math.cos(math.radians(yaw_deg)) * 32767)


def c_div(a, b):
    """C 的整数除法是【向零取整】，Python 的 // 是向下取整，负数会不同。"""
    q = abs(a) // abs(b)
    return q if (a >= 0) == (b >= 0) else -q


def integrate_int(n_steps, ds_per_step, use_remainder, yaw_deg=0.0):
    """用固件里的整数数学累加位置。
    返回 (积分出来的距离, 实际走过的距离)，单位 0.1mm。
    use_remainder=False 复现修复前的写法：s_x10 += ds*cos/32768"""
    c = q15_cos(yaw_deg)
    x, fx = 0, 0
    for _ in range(n_steps):
        if use_remainder:
            fx += ds_per_step * c
            d = c_div(fx, 32768)
            x += d
            fx -= d * 32768
        else:
            x += c_div(ds_per_step * c, 32768)
    return x, n_steps * ds_per_step


def test_integration_truncation():
    """慢速推车时，整除截断会按【速度】丢距离 —— 实测就是这么丢的 30%。"""
    print("  [整数截断] 每 5ms 一拍，一拍走 3 个单位（0.3mm）= 60mm/s 的慢推：")
    n_steps, ds = 1843, 3            # 1843 拍 ≈ 9.2 秒，共 5529 个单位 = 552.9mm
    old_x, total = integrate_int(n_steps, ds, use_remainder=False)
    new_x, _ = integrate_int(n_steps, ds, use_remainder=True)
    print(f"         实走 {total} 个单位（{total/10:.1f} mm）")
    print(f"         修复前积分 {old_x} 个单位 -> 丢 {100*(total-old_x)/total:.0f}%"
          f"（实测推 553mm 只积出 384mm，丢 30%，对得上）")
    print(f"         修复后积分 {new_x} 个单位 -> 丢 {100*(total-new_x)/total:.2f}%"
          f"（剩下的这点是 Q15 的 cos=32767/32768 固有，可忽略）")
    # 不能要求绝对相等：Q15 的 cos 峰值是 32767 不是 32768，天生差 0.003%
    assert (total - new_x) <= total * 0.005, "带余数累加后不该再按速度丢距离"
    assert old_x < total * 0.8, "复现不出截断丢距离，这个测试没验住东西"
    print("  [通过] 余数累加修好了截断丢距离；旧写法确实会丢\n")


# ======================================================================
def main():
    print("=== 导航几何 / 符号链 离线验算 ===")
    pts, yaw0_mdeg = load_map()
    n = len(pts)

    # ---- 1. 地图数据自检 ----
    kinds = [k for _, _, k in pts]
    dists = [math.dist(pts[i][:2], pts[(i + 1) % n][:2]) for i in range(n)]
    print(f"  [地图] {n} 点 @ {DS_MM}mm，单圈 {n * DS_MM} mm")
    print(f"  [地图] kind: 普通 {kinds.count(0)} / 端部 {kinds.count(1)} / 中段 {kinds.count(2)}")
    print(f"  [地图] 相邻点距 min/mean/max = "
          f"{min(dists):.1f}/{sum(dists) / n:.1f}/{max(dists):.1f} mm")
    print(f"  [地图] 起始航向（头文件写的）= {yaw0_mdeg / 1000.0:.2f} 度，"
          f"首点 500mm 弦 = {chord_yaw(pts, 0, 10):.2f} 度")
    assert abs(yaw0_mdeg / 1000.0 - chord_yaw(pts, 0, 10)) < 1.0, \
        "nav_map_fw.h 的 START_YAW_MDEG 与首点弦方向对不上"
    assert max(dists) < 200.0, "相邻点距过大，地图有断点"
    print("  [通过] 地图自检\n")

    # ---- 2. 直道上、车在中心线且朝向弦方向 -> err 应≈0 ----
    #   只查 kind==2（直道中段）：弯道上"150mm 前瞻点"相对"车头切线"本来就
    #   横向偏出去一点，那是纯跟踪的【前馈项】（正是它让车跟着弯走），
    #   不是误差。所以"err≈0"这条只能在直道上验。
    straight = [i for i, (_, _, k) in enumerate(pts) if k == 2]
    errs = []
    for i in straight:
        yaw = chord_yaw(pts, i, 4)          # 直道上用 200mm 弦就够
        errs.append(abs(nav_err(nav_lat10(pts, i, (i + 3) % n,
                                          pts[i][0] * 10, pts[i][1] * 10, yaw))))
    errs_sorted = sorted(errs)
    med = errs_sorted[len(errs_sorted) // 2]
    # 容差 6.0：直道末端的前瞻点已经探进弯里，那里 err 不为 0 是正常的
    bad = sum(1 for e in errs if e > 6.0)
    print(f"  [在线上/直道] 抽查 {len(straight)} 个直道点："
          f"|err| 中位 {med:.2f}、最大 {max(errs):.2f}、>6.0 的有 {bad} 个")
    assert med < 1.0, f"直道上 err 中位数 {med:.2f} 偏大，符号或前瞻有问题"
    assert bad == 0, "直道上 err 超过容差"

    # 弯道上给个信息量：这是纯跟踪的前馈量，不算错
    curved = [i for i, (_, _, k) in enumerate(pts) if k == 0]
    ce = [abs(nav_err(nav_lat10(pts, i, (i + 3) % n, pts[i][0] * 10,
                                pts[i][1] * 10, chord_yaw(pts, i, 4))))
          for i in curved[:200]]
    print(f"  [参考] 弯道上（车在线上）|err| 中位 {sorted(ce)[len(ce)//2]:.1f}、"
          f"最大 {max(ce):.1f} —— 这是前馈项，不是误差")
    print("  [通过] 在线上 -> err ≈ 0\n")

    # ---- 3. 横向偏移的符号与量级（在直道上测，避开前馈项）----
    i = straight[len(straight) // 2]
    yaw = chord_yaw(pts, i, 4)
    x0, y0 = pts[i][0] * 10.0, pts[i][1] * 10.0
    u = (-math.sin(math.radians(yaw)), math.cos(math.radians(yaw)))   # 车的右侧
    e_right = nav_err(nav_lat10(pts, i, (i + 3) % n,
                                x0 + u[0] * 200.0, y0 + u[1] * 200.0, yaw))
    e_left = nav_err(nav_lat10(pts, i, (i + 3) % n,
                               x0 - u[0] * 200.0, y0 - u[1] * 200.0, yaw))
    print(f"  [偏到车的右边 20mm] err = {e_right:+.2f}   (应为负 -> 该左转回来)")
    print(f"  [偏到车的左边 20mm] err = {e_left:+.2f}   (应为正 -> 该右转回来)")
    assert e_right < 0, "偏右时 err 应为负"
    assert e_left > 0, "偏左时 err 应为正"
    assert abs(abs(e_right) - 20.0) < 2.0, "偏移 -> err 的比例不对（应为 1mm=1 单位）"
    print("  [通过] 横向偏移符号与量级都对\n")

    # ---- 4. 闭环：扫增益，找"能收敛的增益"（验符号，不是验调参）----
    #   只从【直道】起步：直道上没有前馈项，能干净地看出"符号对不对"。
    straight_start = straight[len(straight) // 2]
    print(f"  [闭环] 从直道第 {straight_start} 点出发，初始横向 +30mm、航向 +8 度，"
          f"仿 3 秒，NAV_YAW_SIGN=-1：")
    good = []
    CONV_MM = 15.0    # 最终横向偏差小于它算收敛（这里还没上 D 项，会留一点小振荡）
    BOUND_MM = 60.0   # 过程最大偏差的界：超了说明跑飞了
    for kp in (1, 2, 4, 8, 16, 32, 64):
        final, worst, gone = simulate(pts, -1, kp, start_idx=straight_start)
        settled = (not gone) and abs(final) < CONV_MM and worst < BOUND_MM
        tag = "发散" if gone else ("收敛" if settled else "未收敛/振荡大")
        print(f"         kp={kp:3d} -> 最终偏差 {final:+8.1f} mm，"
              f"过程最大 {worst:7.1f} mm  {tag}")
        if settled:
            good.append(kp)
    assert good, "没有任何增益能让车回到中心线 —— 符号链有问题"
    print(f"  [通过] 符号对：这些增益都能把车拉回线上 {good}\n")

    # ---- 5. 反证：同一个增益，符号写反必须收不回来 ----
    kp_ref = good[0]
    print(f"  [反证] 同一个增益 kp={kp_ref}，把 NAV_YAW_SIGN 写成 +1：")
    final_bad, worst_bad, gone_bad = simulate(pts, +1, kp_ref, start_idx=straight_start)
    settled_bad = (not gone_bad) and abs(final_bad) < CONV_MM and worst_bad < BOUND_MM
    print(f"         最终偏差 {final_bad:+8.1f} mm，过程最大 {worst_bad:7.1f} mm")
    print(f"         （初始扰动只有 30mm：写反了不但收不回来，还越跑越偏）")
    assert not settled_bad, "符号写反竟然也收敛，说明这个测试没验住符号，要重看"
    assert abs(final_bad) > 30.0, "符号写反时偏差应该变大，这里没变大"
    print("  [通过] 反证成立：只有 NAV_YAW_SIGN = -1 能把车收回线上\n")

    # ---- 6. 整数截断：浮点测不到的那一类 ----
    test_integration_truncation()

    print("=== 全部通过 ===")


if __name__ == "__main__":
    main()
