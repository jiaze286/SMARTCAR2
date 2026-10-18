# -*- coding: utf-8 -*-
"""离线协议自证：把编译到 PC 上的 User/tune.c 当固件，按调参器的方式说话。"""
import importlib.util
import json
import os
import queue
import subprocess
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))   # 仓库根目录


def load_tuner():
    spec = importlib.util.spec_from_file_location('tuner', os.path.join(REPO, '工具', '调参迭代.py'))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Sim(object):
    def __init__(self):
        self.p = subprocess.Popen(
            [os.path.join(HERE, 'sim.exe')],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            bufsize=0)
        self.lines = queue.Queue()
        self.err = []
        threading.Thread(target=self._pump_out, daemon=True).start()
        threading.Thread(target=self._pump_err, daemon=True).start()
        self._wait_ready()

    def _pump_out(self):
        buf = b''
        while True:
            ch = self.p.stdout.read(1)
            if not ch:
                self.lines.put(None)
                return
            buf += ch
            if ch == b'\n':
                self.lines.put(buf.decode('ascii', 'replace').strip())
                buf = b''

    def _pump_err(self):
        for raw in self.p.stderr:
            self.err.append(raw.decode('utf-8', 'replace').rstrip())

    def _wait_ready(self):
        t0 = time.time()
        while time.time() - t0 < 5:
            if any('ready' in e for e in self.err):
                return
            time.sleep(0.05)
        raise RuntimeError('sim 没就绪: %r' % (self.err,))

    def drain(self, quiet=0.15):
        """模拟调参器每条命令前的 reset_input_buffer()：把已经攒着的行丢掉。"""
        time.sleep(quiet)
        while True:
            try:
                self.lines.get_nowait()
            except queue.Empty:
                return

    def send(self, text):
        self.p.stdin.write((text.strip() + '\n').encode('ascii'))
        self.p.stdin.flush()

    def readline(self, timeout=2.0):
        try:
            line = self.lines.get(timeout=timeout)
        except queue.Empty:
            return None
        return line

    def close(self):
        try:
            self.p.kill()
        except Exception:
            pass


def main():
    tuner = load_tuner()
    params = tuner.load_params(os.path.join(REPO, '工具', '调参参数.json'))
    keys = list(tuner.FIRMWARE_KEYS)

    sim = Sim()
    bad = []

    # ---- 1. GET 21 键 ----
    print('=== 1. GET 21 键 ===')
    got = {}
    for k in keys:
        sim.drain(0.02)
        sim.send('GET %s' % k)
        line = sim.readline()
        if not line or not line.upper().startswith('OK'):
            bad.append('GET %s -> %r' % (k, line))
            continue
        toks = line.split()
        try:
            got[k] = float(toks[1])
        except Exception:
            bad.append('GET %s 返回不是数: %r' % (k, line))
    print('  读到 %d/%d 个键：%s' % (len(got), len(keys), ' '.join('%s=%g' % (k, v) for k, v in list(got.items())[:6]) + ' ...'))

    # 种子要和 JSON 一致（否则第一次 push 就把车改坏了）。
    # autorun 例外：固件默认 1（上电自动开跑），调参器要求 0，靠 SET 关掉。
    for k in keys:
        if k not in got or k == 'autorun':
            continue
        want = float(params[k])
        if abs(got[k] - want) > max(1e-9, abs(want) * 1e-3):
            bad.append('固件默认值 %s=%g 和 JSON 种子 %g 不一致' % (k, got[k], want))
    print('  固件默认值 vs JSON 种子：%s' % ('一致' if not bad else '有不一致'))

    # ---- 2. SET + GET 回读（调参器的判据：相对误差 <= 1e-3）----
    print('\n=== 2. SET + GET 回读（1e-3）===')
    for k in keys:
        if k not in got:
            continue
        base = got[k]
        new = (base * 1.05) if abs(base) > 1e-9 else 0.0
        want = tuner.fmt_val(new)
        sim.drain(0.02)
        sim.send('SET %s %s' % (k, want))
        ack = sim.readline()
        if not ack or not ack.upper().startswith('OK'):
            bad.append('SET %s %s -> %r' % (k, want, ack))
            continue
        sim.drain(0.02)
        sim.send('GET %s' % k)
        line = sim.readline()
        try:
            back = float(line.split()[1])
        except Exception:
            bad.append('SET 后 GET %s -> %r' % (k, line))
            continue
        ok = abs(back - new) <= max(1e-9, abs(new) * 1e-3)
        if not ok:
            bad.append('回读不符 %s: 发 %s 读 %s' % (k, want, back))
        if k in ('kp_steer', 'vmax', 'corr_enc_k', 'straight_deg', 'look_mm'):
            print('  %-14s %s -> 写 %s 读 %s  %s' % (k, base, want, back, 'OK' if ok else '**不符**'))
        # 还原
        sim.drain(0.02)
        sim.send('SET %s %s' % (k, tuner.fmt_val(base)))
        sim.readline()

    # ---- 3. 非法输入 ----
    print('\n=== 3. 非法输入 ===')
    sim.drain(0.02)
    sim.send('SET no_such_key 1')
    line = sim.readline()
    print('  SET no_such_key 1 -> %r' % line)
    if not line or not line.upper().startswith('ERR'):
        bad.append('未知键没回 ERR: %r' % line)

    sim.drain(0.02)
    sim.send('HELLO WORLD')
    line = sim.readline(timeout=0.6)
    print('  HELLO WORLD -> %r（应为 None：噪声不回话）' % line)
    if line is not None:
        bad.append('噪声行回了话: %r' % line)

    # ---- 4. RUN 1 ----
    print('\n=== 4. RUN 1 ===')
    sim.drain(0.02)
    sim.send('RUN 1')
    ack = sim.readline()
    print('  ack = %r' % ack)
    laps = []
    t0 = time.time()
    while time.time() - t0 < 20 and len(laps) < 1:
        line = sim.readline(15.0)
        if line is None:
            break
        rec = tuner.parse_lap_line(line)
        if rec is not None:
            laps.append(rec)
            print('  LAP: %s' % line)
        else:
            print('  (忽略) %r' % line)
    if len(laps) != 1:
        bad.append('RUN 1 只收到 %d 圈' % len(laps))
    else:
        r = laps[0]
        print('  解析：圈=%d 跑完=%d 用时=%.0fms 里程=%.0fmm 线RMS=%.2f P95=%.2f MAX=%.2f 惯导RMS=%.2f 补偿=%.3f 脱线=%d 原因=%s'
              % (r.lap, r.finished, r.time_ms, r.dist_mm, r.line_rms, r.line_p95, r.line_max,
                 r.ins_rms, r.comp_deg_mean, r.lost_count, r.reason))
        if r.finished != 1:
            bad.append('RUN 1 的 finished 不是 1')
        if not (6000 <= r.time_ms <= 12000):
            bad.append('单圈用时不合理: %.0fms（仿真应约 8300ms）' % r.time_ms)
        if not (30000 <= r.dist_mm <= 40000):
            bad.append('单圈里程不合理: %.0fmm' % r.dist_mm)

    # ---- 5. RUN 2（连续两圈 + 跑完自停）----
    print('\n=== 5. RUN 2 ===')
    sim.drain(0.02)
    sim.send('RUN 2')
    ack = sim.readline()
    print('  ack = %r' % ack)
    laps = []
    t0 = time.time()
    while time.time() - t0 < 40 and len(laps) < 2:
        line = sim.readline(15.0)
        if line is None:
            break
        rec = tuner.parse_lap_line(line)
        if rec is not None:
            laps.append(rec)
            print('  LAP: %s' % line)
    if len(laps) != 2:
        bad.append('RUN 2 只收到 %d 圈' % len(laps))
    else:
        print('  两圈用时：%.0fms / %.0fms' % (laps[0].time_ms, laps[1].time_ms))
        if laps[0].lap != 1 or laps[1].lap != 2:
            bad.append('圈号不对: %d %d' % (laps[0].lap, laps[1].lap))

    # 跑完应该自己停车（再等 1 秒不应该再出 LAP）
    extra_laps = 0
    t_end = __import__('time').time() + 1.5
    while __import__('time').time() < t_end:
        line = sim.readline(0.3)
        if line and line.upper().startswith('LAP'):
            extra_laps += 1
    print('  跑完后再等 1.5 秒：多出来的 LAP 行 %d 条（应为 0；遥测行继续发是正常的）' % extra_laps)
    if extra_laps:
        bad.append('跑满 2 圈后还在打 LAP：%d 条' % extra_laps)

    # ---- 6. STOP ----
    print('\n=== 6. STOP ===')
    sim.drain(0.02)
    sim.send('STOP')
    print('  %r' % sim.readline())

    sim.close()

    print('\n================ 结果 ================')
    if bad:
        for b in bad:
            print('  [FAIL] %s' % b)
        return 1
    print('  全部通过（21 键 GET、21 键 SET+回读 1e-3、ERR/噪声、RUN1、RUN2 自停、STOP）')
    return 0


if __name__ == '__main__':
    sys.exit(main())
