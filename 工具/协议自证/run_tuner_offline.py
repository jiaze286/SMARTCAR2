# -*- coding: utf-8 -*-
"""让真正的 调参迭代.py 对着编到 PC 上的固件跑一遍（假串口 = sim.exe 的管道）。
证明的是：调参器的 push_all 42 次往返回读、RUN/LAP 收集、决策链，全部能通。"""
import importlib.util
import os
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))   # 仓库根目录
SIM = os.path.join(HERE, 'sim.exe')


class FakeSerial(object):
    """只实现 SerialTransport 用到的那几个方法。"""

    def __init__(self, port=None, baudrate=9600, timeout=2.0, **kw):
        self.timeout = float(timeout)
        self.buf = b''
        self.lock = threading.Lock()
        self.proc = subprocess.Popen([SIM], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     stderr=subprocess.DEVNULL, bufsize=0)
        threading.Thread(target=self._pump, daemon=True).start()
        time.sleep(0.3)

    def _pump(self):
        while True:
            b = self.proc.stdout.read(1)
            if not b:
                return
            with self.lock:
                self.buf += b

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.buf)

    def read(self, n=1):
        t0 = time.time()
        while True:
            with self.lock:
                if len(self.buf) >= n:
                    d, self.buf = self.buf[:n], self.buf[n:]
                    return d
            if time.time() - t0 > self.timeout:
                return b''
            time.sleep(0.001)

    def readline(self):
        t0 = time.time()
        while True:
            with self.lock:
                i = self.buf.find(b'\n')
                if i >= 0:
                    d, self.buf = self.buf[:i + 1], self.buf[i + 1:]
                    return d
            if time.time() - t0 > self.timeout:
                return b''
            time.sleep(0.001)

    def write(self, d):
        self.proc.stdin.write(d)
        self.proc.stdin.flush()

    def flush(self):
        pass

    def reset_input_buffer(self):
        with self.lock:
            self.buf = b''

    def close(self):
        try:
            self.proc.kill()
        except Exception:
            pass


def main():
    spec = importlib.util.spec_from_file_location('tuner', os.path.join(REPO, '工具', '调参迭代.py'))
    tuner = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tuner)

    tuner.serial.Serial = FakeSerial          # 把 pyserial 换成仿真管道

    argv = ["--port", "FAKE", "--phase", "B", "--dry", "--laps", "1",
            "--max-rounds", "8",
            "--b-order", "kp_steer,kd_steer,corner_margin"]
    print("跑：调参迭代.py %s" % " ".join(argv))
    rc = tuner.main(argv)
    print("\n退出码 = %s" % rc)
    return rc


if __name__ == '__main__':
    sys.exit(main())
