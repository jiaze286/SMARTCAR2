#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
看遥测 —— 实时打印车上的串口行（默认 DAPLink 的 COM 口，9600）。

用法：
    python 工具/看遥测.py                 # COM17，一直打印
    python 工具/看遥测.py --port COM5     # 换串口（比如配好蓝牙后的那个）
    python 工具/看遥测.py --secs 20       # 只看 20 秒
    python 工具/看遥测.py --send 0x09     # 先发一个命令字节再开始看

行格式见 User/main.c 的注释：
  数据行 M:3 X:206 Y:197 YW:0 W:0 D:0 K:1 LP:2270            （推车对图）
  数据行 M:4 I:.. X:.. Y:.. YW:.. YC:.. E:.. D:.. W:.. K:.. L:..（惯导跑）
  状态行 S:P70 S32 B3900 D0 K0 I0 ID:112 ... LAP:43450 ...
"""

import argparse
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("缺 pyserial：pip install pyserial")

CMD = {0x00: "停止", 0x01: "前进", 0x02: "后退", 0x03: "手动左转", 0x04: "手动右转",
       0x05: "纯光电 暂停/继续", 0x06: "惯导跑", 0x07: "纯光电循迹",
       0x08: "惯导跑(同0x06)", 0x09: "推车对图"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM17")
    ap.add_argument("--baud", type=int, default=9600)
    ap.add_argument("--secs", type=float, default=0, help="0 = 一直看")
    ap.add_argument("--send", default=None, help="先发一个命令字节，如 0x09")
    args = ap.parse_args()

    s = serial.Serial(args.port, args.baud, timeout=0.5)
    time.sleep(0.2)
    s.reset_input_buffer()

    if args.send:
        b = int(args.send, 16) & 0xFF
        s.write(bytes([b]))
        print(f"[已发送 0x{b:02X} = {CMD.get(b, '未知')}]\n")
        time.sleep(0.3)
        s.reset_input_buffer()

    print(f"[{args.port} @ {args.baud}]  Ctrl-C 退出\n")
    t0 = time.time()
    try:
        while True:
            if args.secs and time.time() - t0 > args.secs:
                break
            line = s.readline().decode("gbk", errors="replace").strip()
            if line:
                print(f"{time.time() - t0:7.1f}s  {line}")
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
    print("\n[结束]")


if __name__ == "__main__":
    main()
