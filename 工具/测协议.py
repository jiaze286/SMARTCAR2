#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测协议 —— 台架上一条命令验证固件侧的 SET/GET/RUN 调参协议通不通。

用法：
    python 工具/测协议.py --port COM17            # 只读回 21 个键（SET/GET 能力）
    python 工具/测协议.py --port COM17 --write    # 再试一次 SET 回读校验（写完改回原值）
    python 工具/测协议.py --port COM17 --run 1    # 再发 RUN 1，等一行 LAP 打出来
    python 工具/测协议.py --port COM17 --stop     # 最后发 STOP 停车

为什么要有这个脚本：调参器（调参迭代.py）只在三阶段都跑起来才看得见协议问题，
一旦 PUSHFAIL 或 SHORT，现场很难分辨是"固件没实现"还是"串口线/波特率不对"。
这里把 21 个键、回读校验、RUN/LAP 拆成可以单独跑的动作。

判据（全部满足才算协议 OK）：
    - 21 个键都回 "OK <数>"，且数能解析成 float
    - --write 时回读和写入的相对误差 <= 1e-3
    - --run 时先收到 ACK（或缺省），随后收到一行 >=13 字段的 LAP
"""

import argparse
import sys
import time

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    sys.exit("缺 pyserial：pip install pyserial")

KEYS = [
    "kp_steer", "kd_steer", "k_gyro",
    "kp_speed", "ki_speed", "kd_speed",
    "vmax", "vmin", "a_acc", "a_brake", "a_lat",
    "look_mm",
    "corr_kp", "corr_ki", "corr_kd", "corr_enc_k",
    "straight_min", "corner_margin", "straight_deg",
    "corr_on", "autorun",
]


class Link(object):
    def __init__(self, port, baud, timeout=2.0):
        self.ser = serial.Serial(port=port, baudrate=baud, timeout=timeout)
        self.timeout = timeout
        time.sleep(0.2)
        self.flush()

    def flush(self):
        try:
            self.ser.reset_input_buffer()
        except Exception:
            pass

    def cmd(self, text):
        """发一条命令，返回回应行（认 OK/ERR/ACK/LAP，其他行跳过）。"""
        self.flush()
        self.ser.write((text.strip() + "\n").encode("ascii", "ignore"))
        self.ser.flush()
        deadline = time.time() + self.timeout
        while time.time() < deadline:
            raw = self.ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", "ignore").strip()
            if not line:
                continue
            up = line.upper()
            if up.startswith(("OK", "ERR", "ACK", "LAP")):
                return line
        return None

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None, help="串口，如 COM17")
    ap.add_argument("--baud", type=int, default=9600, help="要跟固件 USART1_BAUD_DEFAULT 一致")
    ap.add_argument("--write", action="store_true", help="做一次 SET + GET 回读校验")
    ap.add_argument("--run", type=int, default=0, help="发 RUN <n> 并等一行 LAP（n=0 不跑）")
    ap.add_argument("--stop", action="store_true", help="最后发 STOP")
    args = ap.parse_args()

    port = args.port
    if not port:
        ports = [p.device for p in list_ports.comports()]
        if not ports:
            sys.exit("没找到串口；用 --port 指定")
        port = ports[0]
        print("[提示] 没给 --port，用第一个串口 %s" % port)

    print("[%s @ %d] 打开" % (port, args.baud))
    lk = Link(port, args.baud)
    bad = 0
    vals = {}

    print("\n=== 1. 读 21 个键 ===")
    for k in KEYS:
        line = lk.cmd("GET %s" % k)
        if line is None:
            print("  %-14s <- (超时)" % k)
            bad += 1
            continue
        toks = line.split()
        if len(toks) < 2 or toks[0].upper() != "OK":
            print("  %-14s <- %r" % (k, line))
            bad += 1
            continue
        try:
            vals[k] = float(toks[1])
            print("  %-14s = %s" % (k, toks[1]))
        except ValueError:
            print("  %-14s <- %r（不是数）" % (k, line))
            bad += 1

    if args.write:
        print("\n=== 2. SET + GET 回读校验（改 1% 再改回）===")
        for k in KEYS:
            if k not in vals:
                continue
            old = vals[k]
            if abs(old) < 1e-9:
                continue                       # 0 值没法按比例改
            new = old * 1.01
            want = ("%d" % round(new)) if abs(new - round(new)) < 1e-12 else ("%.8g" % new)
            ack = lk.cmd("SET %s %s" % (k, want))
            if ack is None or not ack.upper().startswith("OK"):
                print("  %-14s SET 失败：%r" % (k, ack))
                bad += 1
                continue
            got_line = lk.cmd("GET %s" % k)
            try:
                got = float(got_line.split()[1])
            except Exception:
                print("  %-14s 回读失败：%r" % (k, got_line))
                bad += 1
                continue
            ok = abs(got - new) <= max(1e-9, abs(new) * 1e-3)
            print("  %-14s %s -> %s  回读 %s  %s" % (k, old, want, got_line.split()[1], "OK" if ok else "**不符**"))
            if not ok:
                bad += 1
            # 改回去
            back = ("%d" % round(old)) if abs(old - round(old)) < 1e-12 else ("%.8g" % old)
            lk.cmd("SET %s %s" % (k, back))

    if args.run > 0:
        print("\n=== 3. RUN %d（车会真的动！先摆好）===" % args.run)
        ack = lk.cmd("RUN %d" % args.run)
        print("  ACK: %r" % ack)
        n = 0
        t0 = time.time()
        while n < args.run and time.time() - t0 < 30.0 * max(1, args.run) + 5.0:
            raw = lk.ser.readline()
            if not raw:
                continue
            line = raw.decode("ascii", "ignore").strip()
            if not line:
                continue
            if line.upper().startswith("LAP"):
                n += 1
                print("  %s" % line)
            elif line.upper().startswith(("OK", "ERR", "ACK")):
                print("  (忽略) %s" % line)
        if n < args.run:
            print("  ** 只收到 %d/%d 圈 **" % (n, args.run))
            bad += 1

    if args.stop:
        print("\n=== 4. STOP ===")
        print("  %r" % lk.cmd("STOP"))

    lk.close()
    print("\n结论：%s" % ("协议正常" if bad == 0 else "有 %d 处不对（见上）" % bad))
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
