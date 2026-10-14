#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
调参迭代.py — B 车（STM32F103C8T6，惯导为主 + 光电为辅，5ms 控制周期）
"一圈一次迭代"自动调参器。

设计要点
--------
* 参数全部平铺在 调参参数.json，代码不写死任何可调数值（只保留键名列表、
  阶段顺序等结构性内容；ms/s 之类单位换算除外）。
* 两种后端，同一接口：
    - SerialTransport  : pyserial，实车 SET/GET/RUN + 解析 LAP 行（pyserial 可选）
    - SimTransport     : --sim 纯 Python 合成响应面 + 播种噪声，离线验证策略
    - ReplayTransport  : --replay 直接吃一段实测 LAP 行做离线复算
* 三阶段：A 诊断（不动参）→ B 压偏差（单参坐标下降）→ C 加速（约束偏差下压时间）。
* 硬性安全阀：单圈步长 <=5%；轮前备份"最后一组能跑完的参数"；连续 2 次不达标
  → 回退；偏差超阈 / 连续丢线 → 立即停本轮并回退；只改白名单键。
* 零第三方依赖（缺 pyserial 自动回退离线模式）。

串口协议（见 契约一）：SET <键> <值> -> OK ；GET <键> -> OK <值> ；RUN <n> -> ACK。
遥测（见 契约二）：每圈结束固件打印一行
  LAP,<圈号>,<跑完0/1>,<耗时ms>,<里程mm>,<线RMS>,<线P95>,<线MAX>,
      <惯导RMS>,<惯导P95>,<补偿角均值deg>,<脱线次数>,<结束原因>
"""

import argparse
import csv
import json
import math
import random
import sys
import time
from pathlib import Path

# ---- pyserial 可选：缺失时自动走离线（sim）模式 ----
try:
    import serial  # type: ignore
    HAVE_SERIAL = True
except Exception:  # pragma: no cover
    serial = None
    HAVE_SERIAL = False

HERE = Path(__file__).resolve().parent

# ==========================================================================
# 结构性名单（键名/顺序，不是可调数值）
# ==========================================================================
# 可经 SET 下发的固件车参白名单。电机方向 / 限流 / 引脚映射 等一律不在此列，
# 也根本不出现在 JSON 里，SafetyGuard 会拒绝任何越界写入。
FIRMWARE_KEYS = [
    "kp_steer", "kd_steer", "k_gyro",
    "kp_speed", "ki_speed", "kd_speed",
    "vmax", "vmin", "a_acc", "a_brake", "a_lat",
    "look_mm", "corr_kp", "corr_ki", "corr_kd", "corr_enc_k",
    "straight_min", "corner_margin", "straight_deg",
    "corr_on", "autorun",
]
# 建图固定参：存档用，不下发、不自动改
FIXED_KEYS = ["c_long", "c_lat", "span_th", "e_slow"]
# 调参元参（本地）
META_KEYS = [
    "lap_per_config", "diag_laps", "step_b", "vmax_step", "accept_eps",
    "max_step_pct", "p95_gate_mm", "ins_p95_gate_mm", "lost_max",
    "dev_abort_gate_mm", "lost_streak_abort", "max_b_sweeps",
    "vmax_ceiling", "vmax_floor", "vmax_min_step",
    "param_lo_factor", "param_hi_factor", "w_line", "w_ins",
    "w_time", "time_tol", "rollback_after_fail",
    "fail_score_penalty", "force_corr_on", "max_rounds",
    "serial_baud", "serial_timeout_s", "lap_timeout_s",
]
# 合成模型系数（仅 --sim）
SIM_KEYS = [
    "sim_seed", "sim_track_len_mm", "sim_perturb_pct", "sim_base_line_rms",
    "sim_p95_ratio_line", "sim_max_over_p95", "sim_p95_ratio_ins",
    "sim_corr_floor", "sim_comp_scale", "sim_drift_sign", "sim_gyro_drift_dps",
    "sim_drift_gain", "sim_corner_penalty", "sim_c_acc", "sim_noise_line",
    "sim_noise_ins", "sim_noise_time", "sim_c_look", "sim_c_kp_steer",
    "sim_c_kd_steer", "sim_c_k_gyro", "sim_c_corr_kp", "sim_c_corr_ki",
    "sim_c_v2", "sim_c_spike", "sim_vmax_stable", "sim_fail_slope",
    "sim_steering_margin_gain", "sim_aacc_slip", "sim_slip_slope",
]
REQUIRED_KEYS = FIRMWARE_KEYS + FIXED_KEYS + META_KEYS + SIM_KEYS

# B 阶段单参坐标下降顺序（用户指定）
PHASE_B_ORDER = ["look_mm", "kp_steer", "kd_steer", "k_gyro", "corr_kp", "corr_ki"]
# C 阶段 vmax 到边界后再试的键
PHASE_C_ORDER = ["look_mm", "a_acc"]

LAP_COLUMNS = [
    "round", "phase", "tune_key", "lap", "snapshot",
    "finished", "time_ms", "dist_mm",
    "line_rms", "line_p95", "line_max", "ins_rms", "ins_p95",
    "comp_deg_mean", "lost_count", "reason", "score", "decision", "note",
]

REASON_OK = "OK"


# ==========================================================================
# 小工具
# ==========================================================================
def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


def fmt_val(v):
    """SET 下发时的数值格式：整数不带小数点，浮点保留足够精度。"""
    f = float(v)
    if abs(f - round(f)) < 1e-12 and abs(f) < 1e15:
        return str(int(round(f)))
    return "%.8g" % f


def sig(v, n=6):
    try:
        f = float(v)
    except Exception:
        return str(v)
    if f == 0:
        return "0"
    return "%.*g" % (n, f)


# ==========================================================================
# LAP 行
# ==========================================================================
class LapRecord(object):
    __slots__ = ("lap", "finished", "time_ms", "dist_mm", "line_rms",
                 "line_p95", "line_max", "ins_rms", "ins_p95",
                 "comp_deg_mean", "lost_count", "reason")

    def __init__(self, lap, finished, time_ms, dist_mm, line_rms, line_p95,
                 line_max, ins_rms, ins_p95, comp_deg_mean, lost_count, reason):
        self.lap = int(lap)
        self.finished = int(finished)
        self.time_ms = float(time_ms)
        self.dist_mm = float(dist_mm)
        self.line_rms = float(line_rms)
        self.line_p95 = float(line_p95)
        self.line_max = float(line_max)
        self.ins_rms = float(ins_rms)
        self.ins_p95 = float(ins_p95)
        self.comp_deg_mean = float(comp_deg_mean)
        self.lost_count = int(lost_count)
        self.reason = str(reason)

    @property
    def ok(self):
        return self.finished == 1

    def to_line(self):
        return "LAP,%d,%d,%s,%s,%s,%s,%s,%s,%s,%s,%d,%s" % (
            self.lap, self.finished, fmt_val(self.time_ms), fmt_val(self.dist_mm),
            sig(self.line_rms), sig(self.line_p95), sig(self.line_max),
            sig(self.ins_rms), sig(self.ins_p95), sig(self.comp_deg_mean),
            self.lost_count, self.reason)


def parse_lap_line(line):
    """把一行 LAP,... 解析成 LapRecord；不是 LAP 行返回 None。"""
    if line is None:
        return None
    s = line.strip()
    if not s:
        return None
    parts = [p.strip() for p in s.split(",")]
    if parts[0].upper() != "LAP" or len(parts) < 13:
        return None
    try:
        return LapRecord(
            lap=int(float(parts[1])),
            finished=int(float(parts[2])),
            time_ms=float(parts[3]),
            dist_mm=float(parts[4]),
            line_rms=float(parts[5]),
            line_p95=float(parts[6]),
            line_max=float(parts[7]),
            ins_rms=float(parts[8]),
            ins_p95=float(parts[9]),
            comp_deg_mean=float(parts[10]),
            lost_count=int(float(parts[11])),
            reason=",".join(parts[12:]) or REASON_OK,
        )
    except (ValueError, IndexError):
        return None


# ==========================================================================
# 合成后端：纯 Python 响应面 + 播种噪声
# ==========================================================================
class SimTransport(object):
    """
    离线合成模型。真值最优点 = 载入时的种子车参；响应面为绕最优点的碗状惩罚，
    叠加速度耦合、陀螺漂移、失效概率与逐圈高斯噪声。仅用于验证"策略"是否
    不追噪声、能回退、能停止，不追求复现真车动力学。
    """
    name = "sim"

    def __init__(self, seed_params, meta):
        self.meta = meta
        # 隐藏真值最优点 = 未扰动的种子车参
        self.opt = {k: float(seed_params[k]) for k in FIRMWARE_KEYS if k in seed_params}
        self.work = dict(seed_params)
        self.rng = random.Random(int(meta["sim_seed"]))
        self._lap = 0

    # --- 传输接口 ---
    def set(self, key, val):
        self.work[key] = val
        return True

    def get(self, key):
        return self.work.get(key)

    def close(self):
        pass

    def run(self, n):
        return [self._one_lap() for _ in range(int(n))]

    # --- 响应面 ---
    def _dz(self, key):
        o = self.opt.get(key, 0.0)
        v = float(self.work.get(key, o))
        if o <= 0.0 or v <= 0.0:
            return 0.0
        return math.log(v / o)

    def _one_lap(self):
        m = self.meta
        w = self.work
        dz = self._dz

        # 转向碗：look_mm / kp_steer / kd_steer / k_gyro 偏离最优越多，线偏差越大
        pen_steer = (m["sim_c_look"] * dz("look_mm") ** 2
                     + m["sim_c_kp_steer"] * dz("kp_steer") ** 2
                     + m["sim_c_kd_steer"] * dz("kd_steer") ** 2
                     + m["sim_c_k_gyro"] * dz("k_gyro") ** 2)

        vmax = float(w["vmax"])
        a_acc = float(w["a_acc"])
        opt_v = self.opt.get("vmax", vmax) or vmax
        opt_a = self.opt.get("a_acc", a_acc) or a_acc

        # 速度越高，弯内线偏差按 ~v^2 放大
        v_ratio = vmax / opt_v if opt_v else 1.0
        speed_mult = 1.0 + m["sim_c_v2"] * max(-0.9, v_ratio * v_ratio - 1.0)

        # 单圈时间：里程 / 有效速度；弯越多、加速越慢则有效速度越低
        corner_pen = 1.0 + m["sim_corner_penalty"] + m["sim_c_acc"] * max(0.0, opt_a / a_acc - 1.0)
        v_eff = vmax / max(1e-6, corner_pen)
        time_s = m["sim_track_len_mm"] / max(1e-6, v_eff)

        # 失效概率：vmax 越过稳定裕度（转向越差裕度越低）+ 急加速打滑
        v_stable = m["sim_vmax_stable"] * (1.0 - m["sim_steering_margin_gain"] * pen_steer)
        z = clamp((vmax - v_stable) * m["sim_fail_slope"], -60.0, 60.0)
        p_fail = 1.0 / (1.0 + math.exp(-z))
        slip = max(0.0, a_acc - m["sim_aacc_slip"]) * m["sim_slip_slope"]
        p_fail = min(0.95, p_fail + slip)

        # 线偏差
        line_rms = max(0.05, m["sim_base_line_rms"] * (1.0 + pen_steer) * speed_mult
                       + self.rng.gauss(0.0, m["sim_noise_line"]))
        spike = 1.0 + m["sim_c_spike"] * (max(0.0, -dz("look_mm")) + max(0.0, -dz("kd_steer")))
        line_p95 = max(0.05, line_rms * m["sim_p95_ratio_line"] * (1.0 + 0.5 * (spike - 1.0))
                       + self.rng.gauss(0.0, m["sim_noise_line"]))
        line_max = line_p95 * m["sim_max_over_p95"] * spike + abs(self.rng.gauss(0.0, m["sim_noise_line"]))

        # 惯导横向偏差：陀螺漂移随圈时长积累；开矫正则被压到 floor + 增益碗
        ins_uncorr = m["sim_gyro_drift_dps"] * time_s * m["sim_drift_gain"]
        corr_on = float(w.get("corr_on", 0.0)) > 0.5
        pen_corr = m["sim_c_corr_kp"] * dz("corr_kp") ** 2 + m["sim_c_corr_ki"] * dz("corr_ki") ** 2
        ins_rms = max(0.05, ins_uncorr * ((m["sim_corr_floor"] + pen_corr) if corr_on else 1.0)
                      + self.rng.gauss(0.0, m["sim_noise_ins"]))
        ins_p95 = max(0.05, ins_rms * m["sim_p95_ratio_ins"] + self.rng.gauss(0.0, m["sim_noise_ins"]))
        comp_deg = m["sim_drift_sign"] * m["sim_comp_scale"] * ins_uncorr if corr_on else 0.0

        self._lap += 1
        if self.rng.random() < p_fail:
            # 未跑完：里程截断、偏差尖峰、脱线（走"未达标"软路径，不硬吹到 abort 阈）
            frac = 0.3 + 0.6 * self.rng.random()
            dist = m["sim_track_len_mm"] * frac
            t_ms = dist / max(1e-6, v_eff) * 1000.0 * (1.0 + self.rng.gauss(0.0, m["sim_noise_time"]))
            reason = "SPIN" if (a_acc > m["sim_aacc_slip"] and slip > 0.0) else "LOST"
            return LapRecord(self._lap, 0, max(1.0, t_ms), dist,
                             line_rms * 1.5, line_p95 * 2.0, line_max * 2.0,
                             ins_rms, ins_p95, comp_deg, 1, reason)
        t_ms = time_s * 1000.0 * (1.0 + self.rng.gauss(0.0, m["sim_noise_time"]))
        return LapRecord(self._lap, 1, max(1.0, t_ms), m["sim_track_len_mm"],
                         line_rms, line_p95, line_max, ins_rms, ins_p95,
                         comp_deg, 0, REASON_OK)


# ==========================================================================
# 串口后端（pyserial 可选）
# ==========================================================================
class SerialTransport(object):
    name = "serial"

    def __init__(self, port, baud, cmd_timeout, lap_timeout):
        if not HAVE_SERIAL:
            raise RuntimeError("pyserial 未安装，无法用串口后端（改用 --sim 离线模式）")
        self.port = port
        self.cmd_timeout = float(cmd_timeout)
        self.lap_timeout = float(lap_timeout)
        self.ser = serial.Serial(port=port, baudrate=int(baud), timeout=float(cmd_timeout))
        self._drain()

    def _drain(self):
        try:
            while self.ser.in_waiting:
                self.ser.read(self.ser.in_waiting)
        except Exception:
            pass

    def _send(self, text):
        self.ser.reset_input_buffer()
        self.ser.write((text.strip() + "\n").encode("ascii", "ignore"))
        self.ser.flush()

    def _readline(self, deadline):
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", "ignore").strip()
            if line:
                return line
        return None

    def set(self, key, val):
        """M1：写参后必须回读校验，回读不上或值不符一律判失败。

        实车最怕"以为写进去了其实没生效"：超时 / 丢帧 / 固件不认这个键，
        原来只等一个 OK 就返回，四种异常全都静默通过，之后每一圈观测都建在错参数上。
        这里发完 SET 立刻 GET 一次对值，失败重试一次。
        """
        want = fmt_val(val)
        for _ in (1, 2):
            self._send("SET %s %s" % (key, want))
            line = self._readline(time.time() + self.cmd_timeout)
            if not (line and line.upper().startswith("OK")):
                continue
            got = self.get(key)
            if got is None:
                continue
            try:
                if abs(float(got) - float(val)) <= max(1e-9, abs(float(val)) * 1e-3):
                    return True
            except (TypeError, ValueError):
                if str(got).strip() == want:
                    return True
            sys.stderr.write("[warn] %s 回读不符：期望 %s，读到 %s\n" % (key, want, got))
        sys.stderr.write("[warn] %s=%s 写入后回读校验失败（参数可能未生效）\n" % (key, want))
        return False

    def get(self, key):
        self._send("GET %s" % key)
        line = self._readline(time.time() + self.cmd_timeout)
        if line and line.upper().startswith("OK"):
            toks = line.split()
            if len(toks) >= 2:
                try:
                    return float(toks[1])
                except ValueError:
                    return toks[1]
        return None

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass

    def run(self, n):
        n = int(n)
        self._send("RUN %d" % n)
        # 等 ACK
        deadline = time.time() + self.cmd_timeout
        acked = False
        while time.time() < deadline:
            line = self._readline(deadline)
            if line is None:
                break
            if line.upper().startswith("ACK"):
                acked = True
                break
            if line.upper().startswith("LAP"):  # 有些固件不回 ACK 直接打 LAP
                rec = parse_lap_line(line)
                if rec:
                    return [rec] + self._collect(n - 1)
        if not acked:
            sys.stderr.write("[warn] RUN 未收到 ACK\n")
        return self._collect(n)

    def _collect(self, n):
        out = []
        if n <= 0:
            return out
        deadline = time.time() + max(self.lap_timeout, 1.0) * n + self.cmd_timeout
        while len(out) < n and time.time() < deadline:
            line = self._readline(deadline)
            if line is None:
                break
            rec = parse_lap_line(line)
            if rec:
                out.append(rec)
        return out


# ==========================================================================
# 回放后端：吃一段实测 LAP 行
# ==========================================================================
class ReplayTransport(object):
    name = "replay"

    def __init__(self, records, meta):
        self.records = list(records)
        self.meta = meta
        self.i = 0
        self.work = {}

    def set(self, key, val):
        self.work[key] = val
        return True

    def get(self, key):
        return self.work.get(key)

    def close(self):
        pass

    def run(self, n):
        out = []
        for _ in range(int(n)):
            if not self.records:
                break
            out.append(self.records[self.i % len(self.records)])
            self.i += 1
        return out


# ==========================================================================
# 参数存取 / 校验
# ==========================================================================
def load_params(path):
    with open(path, "r", encoding="utf-8") as f:
        d = json.load(f)
    return {k: v for k, v in d.items() if not str(k).startswith("_")}


def validate_params(d):
    missing = [k for k in REQUIRED_KEYS if k not in d]
    if missing:
        raise KeyError("调参参数.json 缺少必需键: %s" % ", ".join(missing))
    return True


def save_json(path, obj):
    with open(path, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=2, sort_keys=False)
        f.write("\n")


def snapshot_str(p):
    return ";".join("%s=%s" % (k, sig(p.get(k, 0.0))) for k in FIRMWARE_KEYS)


# ==========================================================================
# 日志 / 收敛曲线
# ==========================================================================
class CsvLogger(object):
    def __init__(self, path, dry=False):
        self.path = Path(path)
        self.dry = dry
        self.rows = 0
        if not self.dry:
            newfile = not self.path.exists() or self.path.stat().st_size == 0
            self.fh = open(self.path, "a", newline="", encoding="utf-8")
            self.w = csv.writer(self.fh)
            if newfile:
                self.w.writerow(LAP_COLUMNS)
                self.fh.flush()

    def log_lap(self, rnd, phase, tune_key, rec, snap, score, decision, note):
        row = [rnd, phase, tune_key, rec.lap, snap,
               rec.finished, "%.1f" % rec.time_ms, "%.1f" % rec.dist_mm,
               "%.3f" % rec.line_rms, "%.3f" % rec.line_p95, "%.3f" % rec.line_max,
               "%.3f" % rec.ins_rms, "%.3f" % rec.ins_p95, "%.3f" % rec.comp_deg_mean,
               rec.lost_count, rec.reason, "%.4f" % score, decision, note]
        self.rows += 1
        if not self.dry:
            self.w.writerow(row)
            self.fh.flush()

    def close(self):
        if not self.dry:
            try:
                self.fh.close()
            except Exception:
                pass


class Curve(object):
    """收敛曲线（ASCII，零依赖）：收集每轮 score / time / p95 / 决策。"""

    def __init__(self):
        self.pts = []

    def add(self, rnd, phase, key, score, time_ms, p95, decision):
        self.pts.append(dict(rnd=rnd, phase=phase, key=key, score=score,
                             time_ms=time_ms, p95=p95, decision=decision))

    def render(self, width=46):
        if not self.pts:
            return "(无数据)"
        scores = [p["score"] for p in self.pts if p["score"] < 1e8]
        smax = max(scores) if scores else 1.0
        smin = min(scores) if scores else 0.0
        span = (smax - smin) or 1.0
        lines = []
        lines.append("轮次 阶段 键            score    time_ms   p95    决策        score 条(越短越好)")
        for p in self.pts:
            frac = 0.0 if p["score"] >= 1e8 else (p["score"] - smin) / span
            bar = "#" * int(round(frac * width))
            lines.append("%4d %-4s %-13s %7.3f %9.0f %6.2f %-11s |%s" % (
                p["rnd"], p["phase"], (p["key"] or "")[:13],
                p["score"], p["time_ms"], p["p95"], p["decision"], bar))
        return "\n".join(lines)


# ==========================================================================
# 安全阀
# ==========================================================================
class RefuseWrite(Exception):
    pass


class SafetyGuard(object):
    def __init__(self, meta, seed):
        self.meta = meta
        self.seed = dict(seed)

    def guarded_set(self, transport, key, val):
        """只允许写白名单键；越界（电机方向/限流/引脚映射等）直接拒绝。"""
        if key not in FIRMWARE_KEYS:
            raise RefuseWrite("拒绝写入非白名单键: %s" % key)
        return transport.set(key, val)

    def clamp_step(self, old, new):
        """单圈参数步长 <= max_step_pct。"""
        old = float(old)
        if old == 0.0:
            return float(new)
        maxd = abs(old) * float(self.meta["max_step_pct"])
        d = clamp(float(new) - old, -maxd, maxd)
        return old + d

    def clamp_bounds(self, key, val):
        """把值钳到 [seed*lo, seed*hi]；vmax 另受 floor/ceiling 约束。"""
        val = float(val)
        # 注意：seed 的值为 0 时不能用 `or` 兜底（0 会被当假值误判），单独判一次
        base = float(self.seed.get(key, val))
        if base == 0.0:
            base = val
        lo = base * float(self.meta["param_lo_factor"])
        hi = base * float(self.meta["param_hi_factor"])
        val = clamp(val, min(lo, hi), max(lo, hi))
        if key == "vmax":
            val = clamp(val, float(self.meta["vmax_floor"]), float(self.meta["vmax_ceiling"]))
        return val

    def propose(self, key, cur, factor):
        """按 factor(如 1.03 / 0.97) 提议新值，先限步长再钳边界。"""
        raw = float(cur) * float(factor)
        stepped = self.clamp_step(cur, raw)
        return self.clamp_bounds(key, stepped)

    def check_abort(self, ev):
        """返回 (是否立即停本轮, 一句人话理由)。"""
        m = self.meta
        if ev.worst_line_max > float(m["dev_abort_gate_mm"]) or ev.worst_line_p95 > float(m["dev_abort_gate_mm"]):
            return True, "偏差超阈(%.1fmm>%.1fmm)，立即停本轮" % (
                max(ev.worst_line_max, ev.worst_line_p95), float(m["dev_abort_gate_mm"]))
        if ev.worst_lost >= int(m["lost_streak_abort"]):
            return True, "连续丢线(%d>=%d)，立即停本轮" % (ev.worst_lost, int(m["lost_streak_abort"]))
        return False, ""


# ==========================================================================
# 评估结果
# ==========================================================================
class EvalResult(object):
    def __init__(self, records, meta):
        self.records = list(records)
        self.meta = meta
        n = len(self.records)
        self.n_laps = n
        self.finished = n > 0 and all(r.ok for r in self.records)
        self.finished_any = any(r.ok for r in self.records)
        self.fail_eval = n > 0 and not self.finished  # 整档失败（含"连续2圈未跑完"）

        def worst(attr):
            return max((getattr(r, attr) for r in self.records), default=0.0)

        self.worst_line_rms = worst("line_rms")
        self.worst_line_p95 = worst("line_p95")
        self.worst_line_max = worst("line_max")
        self.worst_ins_rms = worst("ins_rms")
        self.worst_ins_p95 = worst("ins_p95")
        self.worst_lost = int(worst("lost_count"))
        done = [r.time_ms for r in self.records if r.ok]
        self.time_ms = (sum(done) / len(done)) if done else (
            (sum(r.time_ms for r in self.records) / n) if n else 0.0)
        self.comp_deg_mean = (sum(r.comp_deg_mean for r in self.records) / n) if n else 0.0
        self.reasons = sorted(set(r.reason for r in self.records))

        m = meta
        if not self.finished:
            self.score = float(m["fail_score_penalty"])
        else:
            # M4：时间必须进目标函数。否则 C 阶段"升 vmax"没有"更快"的度量，
            # 只能靠"vmax↑ ⇒ 时间↓"这个隐含假设，而弯多的赛道上它并不成立。
            self.score = (float(m["w_line"]) * self.worst_line_p95
                          + float(m["w_ins"]) * self.worst_ins_p95
                          + float(m["w_time"]) * (self.time_ms / 1000.0))
        self.meets_gate = (self.finished
                           and self.worst_line_p95 <= float(m["p95_gate_mm"])
                           and self.worst_ins_p95 <= float(m["ins_p95_gate_mm"])
                           and self.worst_lost <= int(m["lost_max"]))

    def reason_str(self):
        return "/".join(self.reasons) if self.reasons else "-"


# ==========================================================================
# 调参器
# ==========================================================================
class Tuner(object):
    def __init__(self, transport, params, meta, logger, curve, dry=False):
        self.tp = transport
        self.p = dict(params)
        self.seed = dict(params)
        self.meta = meta
        self.log = logger
        self.curve = curve
        self.dry = dry
        self.guard = SafetyGuard(meta, params)
        self.round = 0
        self.consec_fail = 0
        self.push_fail = 0                 # 连续"参数没写进去"的次数（M1）
        self.last_good = dict(params)      # 最后一组"能跑完"的参数（备份点）
        self.current = None                # 当前配置的 EvalResult
        self.best_params = dict(params)
        self.best_time = float("inf")
        self.best_score = float("inf")
        self.best_reason = "初始"
        if int(meta["force_corr_on"]) == 1:
            self.p["corr_on"] = 1

    # ---- 基础动作 ----
    def _next_round(self):
        self.round += 1
        return self.round

    def push_all(self):
        """把当前参数整组下发；任一键"写入+回读"失败都要炸出来（M1）。

        原来丢弃 guarded_set 的返回值：写失败 = 车带旧参数跑，而脚本以为已生效。
        """
        bad = [k for k in FIRMWARE_KEYS
               if k in self.p and not self.guard.guarded_set(self.tp, k, self.p[k])]
        if bad:                                   # 整组重试一次（串口偶发丢帧）
            bad = [k for k in bad
                   if not self.guard.guarded_set(self.tp, k, self.p[k])]
        if bad:
            raise RefuseWrite("参数下发失败（写入或回读校验不过）：%s" % ",".join(bad))

    def eval_config(self, phase, tune_key, decision_hint, laps=None):
        """下发当前参数 -> RUN laps -> 收 LAP -> 聚合。

        返回 EvalResult；两种"这一档不算数"的情况返回 None（M1/M3）：
          · 参数下发/回读校验失败 —— 写没写进去都不知道，跑出来的数没有意义；
          · 收不满请求圈数 —— "每档 2 圈取最差"的承诺塌了，不能拿 1 圈甚至 0 圈下结论。
        """
        laps = int(laps or self.meta["lap_per_config"])
        rnd = self._next_round()
        snap = snapshot_str(self.p)
        try:
            self.push_all()
        except RefuseWrite as e:
            self.push_fail += 1
            self.log.log_lap(rnd, phase, tune_key,
                             LapRecord(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "PUSHFAIL"),
                             snap, 0.0, "PUSHFAIL", str(e))
            self.curve.add(rnd, phase, tune_key, 0.0, 0.0, 0.0, "PUSHFAIL")
            self.say("  [停] 参数没写进去：%s" % e)
            if self.push_fail >= 3:
                raise RuntimeError("连续 3 次参数下发失败，链路不可信，停止（%s）" % e)
            return None
        self.push_fail = 0

        recs = self.tp.run(laps)
        if len(recs) < laps:                      # 先再要一次，仍不满就判本档作废
            more = self.tp.run(laps - len(recs))
            if more:
                recs = list(recs) + list(more)
        if len(recs) < laps:
            self.log.log_lap(rnd, phase, tune_key,
                             LapRecord(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "SHORT"),
                             snap, 0.0, "SHORT",
                             "只收到 %d/%d 圈，本档作废（不用少样本下结论）" % (len(recs), laps))
            self.curve.add(rnd, phase, tune_key, 0.0, 0.0, 0.0, "SHORT")
            self.say("  [作废] 只收到 %d/%d 圈，本档无效" % (len(recs), laps))
            return None

        ev = EvalResult(recs, self.meta)
        for r in recs:
            self.log.log_lap(rnd, phase, tune_key, r, snap, ev.score, decision_hint, "")
        self.curve.add(rnd, phase, tune_key, ev.score, ev.time_ms, ev.worst_line_p95, decision_hint)
        # 更新"最后一组能跑完"的备份
        if ev.finished:
            self.consec_fail = 0
            self.last_good = dict(self.p)
        else:
            self.consec_fail += 1
        # 更新全局 best
        self._maybe_best(ev)
        self.current = ev
        return ev

    def _maybe_best(self, ev):
        if ev.meets_gate and ev.time_ms < self.best_time:
            self.best_time = ev.time_ms
            self.best_params = dict(self.p)
            self.best_score = ev.score
            self.best_reason = "达标且最快(%.0fms)" % ev.time_ms
        elif (self.best_time == float("inf")) and ev.finished and ev.score < self.best_score:
            self.best_score = ev.score
            self.best_params = dict(self.p)
            self.best_reason = "尚无达标圈，取偏差最小(score=%.3f)" % ev.score

    def say(self, msg):
        print(msg)

    def rollback(self, phase, why):
        self.p = dict(self.last_good)
        self.push_all()
        rnd = self._next_round()
        self.log.log_lap(rnd, phase, "-", LapRecord(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "ROLLBACK"),
                         snapshot_str(self.p), self.current.score if self.current else 0.0,
                         "ROLLBACK", why)
        self.curve.add(rnd, phase, "-", self.current.score if self.current else 0.0,
                       self.current.time_ms if self.current else 0.0,
                       self.current.worst_line_p95 if self.current else 0.0, "ROLLBACK")
        self.current = None      # 回退后当前档案失效，调用侧会按新参数重新评估
        self.say("  [回退] %s -> 回到最后一组能跑完的参数" % why)

    # ---- A 诊断期 ----
    def phase_a(self):
        self.say("\n=== A 诊断期：不动参数，只读日志 ===")
        ev = self.eval_config("A", "-", "DIAG", laps=int(self.meta["diag_laps"]))
        if ev is None:
            self.say("  本档作废（参数没写进去 / 收不满圈），无法诊断：先查串口链路与固件每圈日志。")
            return
        if ev.n_laps == 0:
            self.say("  没读到 LAP 行，无法诊断（检查串口/固件每圈日志）。")
            return
        line = ev.worst_line_rms
        ins = ev.worst_ins_rms
        ratio = (ev.worst_line_max / ev.worst_line_p95) if ev.worst_line_p95 > 1e-9 else 1.0
        comp = ev.comp_deg_mean
        diag = []
        # 是线偏差还是惯导偏差
        if ins > line * 1.3:
            diag.append("偏差以【惯导横向】为主(ins_rms=%.2f > line_rms=%.2f)：漂移没被直道矫正压住，重点看 corr_kp/corr_ki 与 k_gyro" % (ins, line))
        elif line > ins * 1.3:
            diag.append("偏差以【线偏差】为主(line_rms=%.2f > ins_rms=%.2f)：跟线环本身的问题，重点看 look_mm/kp_steer/kd_steer" % (line, ins))
        else:
            diag.append("线偏差与惯导偏差相当(line=%.2f, ins=%.2f)：两条环都要压" % (line, ins))
        # 集中在哪一段（瞬态尖峰 vs 持续偏）
        if ratio >= 1.9:
            diag.append("集中在【弯道/出入弯】：MAX/P95=%.2f 偏高，是瞬态尖峰，说明前瞻/阻尼不足(look_mm 偏小或 kd_steer 偏小)" % ratio)
        else:
            diag.append("集中在【直道】：MAX/P95=%.2f 接近，属持续偏，多为惯导漂移或稳态增益问题" % ratio)
        # 哪个方向
        if abs(comp) > 1e-6:
            direction = "左" if comp > 0 else "右"
            diag.append("方向：补偿角均值 %.2f deg（符号=%s），惯导整体往【%s】漂，直道矫正一直在往回拧" % (comp, "+" if comp > 0 else "-", direction))
        else:
            diag.append("方向：补偿角均值为 0，矫正未开或未积累（确认 corr_on=1）")
        # 脱线
        if ev.worst_lost > 0 or not ev.finished:
            diag.append("脱线/未跑完(lost=%d, reasons=%s)：vmax 可能过高或转向 authority 不足，先别进 C 加速" % (ev.worst_lost, ev.reason_str()))
        for d in diag:
            self.say("  - " + d)
        self.say("  为什么这么决定：诊断期不改任何参数，仅据本圈日志判断偏差性质，为 B 阶段排顺序提供依据。")
        gate = "达标" if ev.meets_gate else "未达标"
        self.say("  当前闸门(%s)：line_p95=%.2f<=%.1f? %s；ins_p95=%.2f<=%.1f? %s" % (
            gate, ev.worst_line_p95, float(self.meta["p95_gate_mm"]),
            "是" if ev.worst_line_p95 <= float(self.meta["p95_gate_mm"]) else "否",
            ev.worst_ins_p95, float(self.meta["ins_p95_gate_mm"]),
            "是" if ev.worst_ins_p95 <= float(self.meta["ins_p95_gate_mm"]) else "否"))

    # ---- B 压偏差：单参坐标下降 ----
    def phase_b(self):
        self.say("\n=== B 压偏差：单参坐标下降，每档跑 %d 圈取最差 ===" % int(self.meta["lap_per_config"]))
        base = self.eval_config("B", "-", "BASE")
        if base is None:
            self.say("  起点档作废（参数没写进去 / 收不满圈），B 阶段不启动：先查链路。")
            return
        self.say("  起点 score=%.3f（line_p95=%.2f, ins_p95=%.2f, 完成=%s）"
                 % (base.score, base.worst_line_p95, base.worst_ins_p95, base.finished))
        step = float(self.meta["step_b"])
        eps = float(self.meta["accept_eps"])
        max_sweeps = int(self.meta["max_b_sweeps"])
        tried = {k: set() for k in PHASE_B_ORDER}

        for sweep in range(max_sweeps):
            improved_any = False
            self.say("  -- 扫描第 %d 遍 --" % (sweep + 1))
            for key in PHASE_B_ORDER:
                cur_val = float(self.p[key])
                cur_ev = self.current if self.current else self.eval_config("B", key, "BASE")
                if cur_ev is None:
                    self.say("  [跳过] %s 的当前档作废（链路 / 收圈不足）" % key)
                    continue
                cur_score = cur_ev.score
                best_val, best_score, best_ev = cur_val, cur_score, cur_ev
                aborted = False
                for sgn in (+1.0, -1.0):
                    cand = self.guard.propose(key, cur_val, 1.0 + sgn * step)
                    ck = round(cand, 9)
                    if ck in tried[key]:
                        continue
                    self.p[key] = cand
                    ev = self.eval_config("B", key, "PROBE")
                    if ev is None:            # M1/M3：本档作废，既不算好也不算坏
                        self.p[key] = cur_val
                        self.say("  [跳过] %s=%s 本档作废（链路 / 收圈不足），不计入分数" % (key, sig(cand)))
                        continue
                    if ev.finished:
                        tried[key].add(ck)  # 只有跑完的点才记账；偶发 DNF 不永久封死该方向
                    elif self.consec_fail >= int(self.meta["rollback_after_fail"]):
                        # M2：连续多档"跑不完"就真回退，别让 current 停在跑不完的参数上。
                        # （原来 self.consec_fail 只写不读，B 阶段永远不会走这一步。）
                        self.rollback("B", "连续 %d 档未跑完（%s）" % (self.consec_fail, ev.reason_str()))
                        aborted = True
                        break
                    abort, why = self.guard.check_abort(ev)
                    if abort:
                        self.rollback("B", why + "（试 %s=%s 时）" % (key, sig(cand)))
                        self.p[key] = cur_val
                        aborted = True
                        break
                    if ev.score < best_score:
                        best_val, best_score, best_ev = cand, ev.score, ev
                if aborted:
                    continue
                # 改善需超过阈值才接受；但单点改善与圈间噪声同量级（仿真实测：单点改善
                # 1.7~2.8%，同参数两圈波动可达 2.5%），单次样本分不开信号和噪声。
                # 所以先做一次"复测确认"：候选值再跑一遍，取两次里**较差**的那次参与判据。
                if best_val != cur_val:
                    self.p[key] = best_val
                    ev2 = self.eval_config("B", key, "CONFIRM")
                    if ev2 is None:
                        self.p[key] = cur_val
                        self.say("  [跳过] %s=%s 复测档作废，本键保持" % (key, sig(best_val)))
                        continue
                    if ev2.score > best_score:      # 取较差（较大）的分数
                        best_score, best_ev = ev2.score, ev2
                # 改善需超过阈值才接受，否则回退到本轮起点（不追噪声）
                if best_val != cur_val and (cur_score - best_score) > eps * max(cur_score, 1e-9):
                    self.p[key] = best_val
                    improved_any = True
                    gain = 100.0 * (cur_score - best_score) / max(cur_score, 1e-9)
                    self.log_decision("B", key, "ACCEPT",
                                      "%s: %s->%s，score %.3f->%.3f（改善 %.1f%%>阈 %.1f%%），接受"
                                      % (key, sig(cur_val), sig(best_val), cur_score, best_score,
                                         gain, 100.0 * eps))
                    self.say("  [接受] %s %s->%s：score %.3f->%.3f，改善 %.1f%% 超过阈值 %.1f%%"
                             % (key, sig(cur_val), sig(best_val), cur_score, best_score, gain, 100.0 * eps))
                    self.current = best_ev
                else:
                    self.p[key] = cur_val
                    if best_val == cur_val:
                        why = "±%.0f%% 两个方向都不比当前好（已在最优附近），保持" % (100.0 * step)
                    else:
                        delta = 100.0 * (cur_score - best_score) / max(cur_score, 1e-9)
                        why = "较好方向 %s 仅改善 %.2f%% < 阈 %.1f%%，属噪声不追，保持" % (sig(best_val), delta, 100.0 * eps)
                    self.log_decision("B", key, "REVERT", "%s=%s：%s" % (key, sig(cur_val), why))
                    self.say("  [回退] %s 保持 %s：%s" % (key, sig(cur_val), why))
                    self.current = cur_ev
            if not improved_any:
                self.say("  本遍无任何键被接受 -> 坐标下降收敛，停止 B。")
                break
        else:
            self.say("  达到 max_b_sweeps=%d，停止 B。" % max_sweeps)

    def log_decision(self, phase, key, decision, note):
        rnd = self.round
        ev = self.current
        rec = ev.records[-1] if ev and ev.records else LapRecord(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, "-")
        self.log.log_lap(rnd, phase, key, rec, snapshot_str(self.p),
                         ev.score if ev else 0.0, decision, note)

    # ---- C 加速 ----
    def phase_c(self):
        self.say("\n=== C 加速：约束偏差下最小化单圈时间 ===")
        cur = self.current if self.current else self.eval_config("C", "-", "BASE")
        if cur is None:
            self.say("  起点档作废（参数没写进去 / 收不满圈），C 阶段不启动：先查链路。")
            return
        if not cur.meets_gate:
            self.say("  当前不满足闸门（跑完=%s, line_p95=%.2f<=%.1f, ins_p95=%.2f<=%.1f, lost=%d<=%d），不进 C。"
                     % (cur.finished, cur.worst_line_p95, float(self.meta["p95_gate_mm"]),
                        cur.worst_ins_p95, float(self.meta["ins_p95_gate_mm"]),
                        cur.worst_lost, int(self.meta["lost_max"])))
            self.say("  为什么这么决定：没达标就加速只会把车甩飞，先回 B 压偏差。")
            return

        # C-1: 逐档升 vmax
        v_good = float(self.p["vmax"])
        t_good = float(cur.time_ms)          # 上一档"达标"的单圈时间（M4 时间门基准）
        step = float(self.meta["vmax_step"])
        min_step = float(self.meta["vmax_min_step"])
        ceil = float(self.meta["vmax_ceiling"])
        eps = float(self.meta["accept_eps"])
        time_tol = float(self.meta["time_tol"])
        consec_fail = 0
        guard_n = 0
        self.say("  -- C-1 升 vmax（起点 %s，步长 %.0f%%，上限 %s）--" % (sig(v_good), 100 * step, sig(ceil)))
        while step >= min_step and guard_n < int(self.meta["max_rounds"]):
            guard_n += 1
            v_try = self.guard.clamp_bounds("vmax", v_good * (1.0 + step))
            if v_try <= v_good * (1.0 + 1e-9):
                break
            self.p["vmax"] = v_try
            ev = self.eval_config("C", "vmax", "PUSH")
            if ev is None:                   # M1/M3：本档作废，退回上一档重试（不算失败）
                self.p["vmax"] = v_good
                self.say("  [跳过] vmax=%s 本档作废（链路 / 收圈不足）" % sig(v_try))
                continue
            abort, why = self.guard.check_abort(ev)
            if abort:
                self.rollback("C", why + "（vmax=%s）" % sig(v_try))
                v_good = float(self.p["vmax"])
                t_good = float(self.current.time_ms) if self.current else t_good
                consec_fail = 0
                step /= 2.0
                continue
            if ev.meets_gate and ev.time_ms <= t_good * (1.0 + time_tol):
                consec_fail = 0
                gain = 100.0 * (self.best_time - ev.time_ms) / max(self.best_time, 1e-9) if self.best_time < 1e8 else 0.0
                v_good = v_try
                t_good = min(t_good, float(ev.time_ms))
                self.log_decision("C", "vmax", "ACCEPT",
                                  "vmax 升到 %s 仍达标，单圈 %.0fms，继续升" % (sig(v_try), ev.time_ms))
                self.say("  [升] vmax=%s 达标，time=%.0fms（较 best %+.1f%%），继续" % (sig(v_try), ev.time_ms, gain))
                if v_try >= ceil * (1.0 - 1e-9):
                    self.say("  vmax 触顶 %s，停止升速。" % sig(ceil))
                    break
            elif ev.meets_gate:
                # M4：达标但更慢 —— 升速没换来更快（弯多时很常见），停在上一档，不再往上升。
                self.p["vmax"] = v_good
                self.log_decision("C", "vmax", "SLOW",
                                  "vmax=%s 达标但更慢（%.0fms > 上一档 %.0fms 的 %.0f%% 容差），停在 %s"
                                  % (sig(v_try), ev.time_ms, t_good, 100 * time_tol, sig(v_good)))
                self.say("  [停] vmax=%s 达标但更慢（%.0fms vs %.0fms），停在上一档 %s"
                         % (sig(v_try), ev.time_ms, t_good, sig(v_good)))
                break
            else:
                consec_fail += 1
                self.p["vmax"] = v_good
                self.log_decision("C", "vmax", "REVERT",
                                  "vmax=%s 不达标(完成=%s, line_p95=%.2f, 原因=%s)，回退到 %s（连续失败 %d）"
                                  % (sig(v_try), ev.finished, ev.worst_line_p95, ev.reason_str(), sig(v_good), consec_fail))
                self.say("  [退] vmax=%s 不达标（%s），回退到 %s，连续失败 %d"
                         % (sig(v_try), ev.reason_str(), sig(v_good), consec_fail))
                if consec_fail >= 2:
                    step /= 2.0
                    consec_fail = 0
                    self.log_decision("C", "vmax", "BISECT",
                                      "连续 2 次失败，二分批间距：步长减半到 %.2f%%" % (100 * step))
                    self.say("  [二分] 连续 2 次失败 -> 步长减半到 %.2f%%（在 %s 与失败点之间逼近上限）"
                             % (100 * step, sig(v_good)))
                    if step < min_step:
                        self.say("  步长已小于最小步 %.2f%%，vmax 收敛到 %s。" % (100 * min_step, sig(v_good)))
                        break
        self.p["vmax"] = v_good

        # C-2: vmax 到边界后，再试 look_mm 与 a_acc（达标且更快才接受）
        for key in PHASE_C_ORDER:
            base_val = float(self.p[key])
            base_ev = self.eval_config("C", key, "BASE")
            if base_ev is None:
                self.say("  [跳过] %s 的基准档作废（链路 / 收圈不足）" % key)
                continue
            if not base_ev.meets_gate:
                # 基准档都没达标（上一步升速可能把配置带到了临界点），不该在这里做探针
                self.say("  [停] %s 基准档未达标（完成=%s, line_p95=%.2f, ins_p95=%.2f），C-2 停止"
                         % (key, base_ev.finished, base_ev.worst_line_p95, base_ev.worst_ins_p95))
                break
            base_time = base_ev.time_ms
            best_val, best_time = base_val, base_time
            for sgn in (+1.0, -1.0):
                cand = self.guard.propose(key, base_val, 1.0 + sgn * step)
                self.p[key] = cand
                ev = self.eval_config("C", key, "PROBE")
                if ev is None:
                    self.p[key] = base_val
                    self.say("  [跳过] %s=%s 本档作废（链路 / 收圈不足）" % (key, sig(cand)))
                    continue
                abort, why = self.guard.check_abort(ev)
                if abort:
                    self.rollback("C", why + "（试 %s=%s 时）" % (key, sig(cand)))
                    self.p[key] = base_val
                    break
                if ev.meets_gate and ev.time_ms < best_time * (1.0 - eps):
                    best_val, best_time = cand, ev.time_ms
            if best_val != base_val:
                self.p[key] = best_val
                self.log_decision("C", key, "ACCEPT",
                                  "%s: %s->%s 达标且更快 %.0f->%.0fms，接受" % (key, sig(base_val), sig(best_val), base_time, best_time))
                self.say("  [接受] %s %s->%s：达标且单圈 %.0f->%.0fms 更快" % (key, sig(base_val), sig(best_val), base_time, best_time))
            else:
                self.p[key] = base_val
                self.log_decision("C", key, "REVERT", "%s: 改动未能在达标下更快，保持 %s" % (key, sig(base_val)))
                self.say("  [保持] %s=%s：改了也没在达标前提下更快" % (key, sig(base_val)))

    # ---- 收尾 ----
    def finalize(self, best_path):
        self.say("\n=== 收尾 ===")
        self.say("  best 理由：%s" % self.best_reason)
        if not self.dry:
            save_json(best_path, self.best_params)
            self.say("  已写 %s" % best_path)


# ==========================================================================
# --sim 起点扰动（制造一个"当前整定略偏"的局面，好让坐标下降有可收敛目标）
# ==========================================================================
def perturb_for_sim(seed, meta):
    pct = float(meta["sim_perturb_pct"])
    rng = random.Random(int(meta["sim_seed"]) + 7)
    work = dict(seed)
    moves = []
    for k in PHASE_B_ORDER:
        if k not in work:
            continue
        sgn = 1.0 if rng.random() < 0.5 else -1.0
        old = float(work[k])
        work[k] = old * (1.0 + sgn * pct)
        moves.append("%s %+.0f%%" % (k, 100 * sgn * pct))
    return work, moves


# ==========================================================================
# 自检（--check）
# ==========================================================================
def self_check(params, meta):
    print("=== 自检 --check ===")
    ok = True
    # 1) 必需键齐全
    try:
        validate_params(params)
        print("[PASS] 调参参数.json 必需键齐全（%d 个）" % len(REQUIRED_KEYS))
    except KeyError as e:
        ok = False
        print("[FAIL] %s" % e)
    # 2) 白名单外键被拒
    guard = SafetyGuard(meta, params)
    class _Dummy:
        def set(self, k, v):
            return True
    for bad in ("motor_dir", "current_limit", "pin_map", "kp_steer_evil"):
        try:
            guard.guarded_set(_Dummy(), bad, 1.0)
            ok = False
            print("[FAIL] 竟然允许写非白名单键: %s" % bad)
        except RefuseWrite:
            print("[PASS] 拒绝写非白名单键: %s" % bad)
    # 3) 步长 <=5%
    old = 100.0
    new = guard.clamp_step(old, old * 1.5)
    if abs(new - old) <= old * float(meta["max_step_pct"]) + 1e-9:
        print("[PASS] 单圈步长被钳到 <=%.0f%%（100->%.1f）" % (100 * float(meta["max_step_pct"]), new))
    else:
        ok = False
        print("[FAIL] 步长钳制失效")
    # 4) sim 能产出可解析 LAP 行 + 解析回环
    sim = SimTransport(params, meta)
    rec = sim.run(1)[0]
    line = rec.to_line()
    back = parse_lap_line(line)
    if back and back.finished == rec.finished and abs(back.line_p95 - rec.line_p95) <= max(1e-9, abs(rec.line_p95) * 1e-5):
        print("[PASS] sim LAP 行可解析回环: %s" % line)
    else:
        ok = False
        print("[FAIL] sim LAP 行解析不一致: %s" % line)
    print("=== 自检 %s ===" % ("通过" if ok else "未通过"))
    return ok


# ==========================================================================
# main
# ==========================================================================
def build_argparser():
    ap = argparse.ArgumentParser(description="B 车一圈一次迭代自动调参器（零第三方依赖，pyserial 可选）")
    ap.add_argument("--params", default=str(HERE / "调参参数.json"), help="平铺参数 JSON")
    ap.add_argument("--log", default=str(HERE / "调参日志.csv"), help="每轮追加的日志 CSV")
    ap.add_argument("--best", default=str(HERE / "调参参数_best.json"), help="最终 best 参数")
    ap.add_argument("--curve", default=str(HERE / "调参收敛.txt"), help="ASCII 收敛曲线输出")
    ap.add_argument("--sim", action="store_true", help="离线合成模式（无实车/无 pyserial 时自动启用）")
    ap.add_argument("--replay", default=None, help="从文件回放 LAP 行做离线复算")
    ap.add_argument("--port", default=None, help="串口号，如 COM5 或 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=None, help="波特率（默认取 JSON serial_baud）")
    ap.add_argument("--laps", type=int, default=None, help="每档圈数（默认取 JSON lap_per_config）")
    ap.add_argument("--phase", default="auto", choices=["A", "B", "C", "auto"], help="跑哪一阶段")
    ap.add_argument("--max-rounds", type=int, default=None, help="总轮数上限")
    ap.add_argument("--check", action="store_true", help="只跑自检不跑迭代")
    ap.add_argument("--dry", action="store_true", help="不写文件（日志/best/curve 只打印）")
    return ap


def main(argv=None):
    args = build_argparser().parse_args(argv)
    params = load_params(args.params)
    validate_params(params)
    meta = params  # 元参与车参同处一个平铺 JSON

    if args.check:
        return 0 if self_check(params, meta) else 2

    if args.laps:
        meta = dict(meta)
        meta["lap_per_config"] = int(args.laps)
    if args.max_rounds:
        meta = dict(meta)
        meta["max_rounds"] = int(args.max_rounds)

    # 选后端
    transport = None
    mode = None
    if args.replay:
        recs = []
        with open(args.replay, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                r = parse_lap_line(line)
                if r:
                    recs.append(r)
        if not recs:
            sys.stderr.write("回放文件里没有可解析的 LAP 行\n")
            return 2
        transport = ReplayTransport(recs, meta)
        mode = "replay"
        working = dict(params)
    elif args.sim or args.port is None or not HAVE_SERIAL:
        if args.port and not HAVE_SERIAL:
            print("[提示] 未安装 pyserial，自动切离线 --sim 模式")
        transport = SimTransport(params, meta)  # 先建，捕获真值最优点=种子
        mode = "sim"
        working, moves = perturb_for_sim(params, meta)
        print("[sim] 起点相对种子做了扰动（真值最优点=种子）：%s" % ", ".join(moves))
    else:
        baud = args.baud or int(meta["serial_baud"])
        transport = SerialTransport(args.port, baud, meta["serial_timeout_s"], meta["lap_timeout_s"])
        mode = "serial"
        working = dict(params)

    print("后端=%s；参数=%s" % (mode, args.params))

    logger = CsvLogger(args.log, dry=args.dry)
    curve = Curve()
    tuner = Tuner(transport, working, meta, logger, curve, dry=args.dry)

    try:
        ph = args.phase.upper()
        if ph in ("A", "AUTO"):
            tuner.phase_a()
        if ph in ("B", "AUTO"):
            tuner.phase_b()
        if ph in ("C", "AUTO"):
            tuner.phase_c()
        tuner.finalize(args.best)
    finally:
        logger.close()
        try:
            transport.close()
        except Exception:
            pass

    text = curve.render()
    print("\n=== 收敛曲线（score 条越短越好）===\n" + text)
    if not args.dry:
        with open(args.curve, "w", encoding="utf-8") as f:
            f.write(text + "\n")
        print("\n已写 %s / %s%s" % (args.log, args.best, "" if args.dry else " / " + args.curve))
    return 0


if __name__ == "__main__":
    sys.exit(main())
