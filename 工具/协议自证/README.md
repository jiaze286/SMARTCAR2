# 协议自证 —— 没有实车也能验证 SET/GET/RUN

把 **`User/tune.c` 原样**编译到 PC 上（传感器、电机、里程全用桩），然后用
**真正的 `工具/调参迭代.py`** 对着它跑。目的是在没有车、没有 DAPLink 的时候，
先证明"协议 + LAP 统计 + 判圈"这条链路是对的，把硬件问题隔离在后面。

## 怎么跑

```bash
# 只需要 gcc（MinGW / w64devkit 都行）和 python3 + pyserial
cd 工具/协议自证

gcc -O1 -Wall -o sim.exe harness.c ../../User/tune.c \
    -I stub -I ../../User -pthread

python driver.py              # ① 协议逐条验：21 键 GET / SET+回读 1e-3 / ERR / 噪声 / RUN 1 / RUN 2 自停 / STOP
python run_tuner_offline.py   # ② 让真的调参器跑 A 阶段（或改脚本里的 phase 跑 B）
```

判据：

- `driver.py` 最后打印 **"全部通过"**；
- `run_tuner_offline.py` 不该出现 `PUSHFAIL` / `SHORT`（A 阶段的闸门不过、B 阶段 ROLLBACK 都正常，
  因为桩里的线偏差是人为造的方波）。

## 文件

| 文件 | 作用 |
|---|---|
| `harness.c` | 桩：`USART1_*` 走 stdin/stdout、`GRAY_*`/`ENCODER_*`/`NAV_*` 造假数据、`App_StartRun/Stop` 起停仿真、1ms 一拍的假时间 |
| `stub/*.h` | 让 `tune.c` 能在 PC 上编译的最小头文件（**只是桩，不参与固件**） |
| `driver.py` | 按调参器的规矩逐条发命令，并用 `调参迭代.py` 自己的 `parse_lap_line` 校验 LAP 行 |
| `run_tuner_offline.py` | 把 `serial.Serial` 换成"仿真管道"，直接跑 `调参迭代.py --phase A/B` |

## 它证明什么 / 不证明什么

**证明**：行协议的分帧与回应、21 键的读写与回读精度、`RUN n` 的圈计数与自停、
LAP 13 字段能被调参器解析、`autorun` 关掉后不会自己开跑。

**不证明**：真车上的灰度/陀螺/电机行为、锚点与葫芦圈在实物上是否对得上、
串口在 9600 下会不会因为遥测让路不及时导致回读失败（真机才可能暴露）。

> 实车第一次上手请用 `工具/测协议.py`，那个才是接车的。

## 改固件时

`tune.c` 的键表/协议一改，先跑这两个脚本再烧车 —— 它们比 Flash 快得多。
新增的键记得同时改：`tune.c` 的 `s_name/s_def` + `调参参数.json` + `调参迭代.py` 的 `FIRMWARE_KEYS`。
