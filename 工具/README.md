# 工具 —— 自动调参 / 照搬手册

> 来源：B 车工程 `C:\Users\asus\Desktop\test16\`（同一用户的另一辆车），2026-10-14 搬入本仓库。
> 目的：让 C 车也能用「跑一圈 → 自动决定下一组参数」这套流程，队友直接可用。

---

## 1. 目录内容

| 文件 | 说明 |
|---|---|
| `调参迭代.py` | 主程序（1203 行，**零第三方依赖**，缺 pyserial 自动转离线）。三阶段：A 诊断 → B 压偏差 → C 加速 |
| `调参参数.json` | **唯一的参数来源**（83 个必需键 + 注释键）。代码不写死任何可调数值 |
| `使用手册-调参迭代器-给AI-20261011.md` | 操作手册（给 AI/接手人）：运行协议、不变量、decision 枚举、自证清单 |
| `使用手册-调参迭代器-给人-20261011.md` | 同一份给人类看 |
| `代码策略与手法-给队友-照搬用-20261013.md` | ★ **B 车固件的总纲**：分层架构 / PC 单测 / 串口台架命令 / 分阶段硬判据 / 坑表 / 协作纪律 |

赛道中心线与生成器已在 `导航地图/`（`nav_map.h` 742 点 @50mm、`赛道中心线-mm.json`、`赛道图转惯导地图.py`），**不在这里重复**。

## 2. 快速开始

```bash
python 工具/调参迭代.py --check                  # 环境自检（全 PASS 才算好）
python 工具/调参迭代.py --sim --phase auto --dry  # 离线跑通流程（不接车）
python 工具/调参迭代.py --port COM5 --phase A     # 接实车：先只诊断，确认能拿到 LAP
python 工具/调参迭代.py --port COM5 --phase auto  # 一条龙 A→B→C
```

**已自证**（2026-10-14 搬入后实测）：`--check` 7 项全 PASS；`--sim --phase auto` 从扰动起点收敛到 score ≈ 7.05
（与 B 车文档的回归基线 ≈7.1 一致）。

生成物（`调参日志.csv` / `调参收敛.txt` / `调参参数_best.json`）默认写在脚本同目录，已加 `.gitignore`。

## 3. 三份契约（固件必须实现，细节见手册 §3）

**① 参数契约** —— 只有白名单里的 21 个键可经 `SET` 下发（其余一律拒绝写入，代码里 `RefuseWrite`）：

```
kp_steer kd_steer k_gyro
kp_speed ki_speed kd_speed
vmax vmin a_acc a_brake a_lat
look_mm corr_kp corr_ki corr_kd corr_enc_k
straight_min corner_margin straight_deg
corr_on autorun
```

**② 遥测契约** —— 每圈结束固件打一行，**13 字段、顺序固定**：

```
LAP,<圈号>,<跑完0|1>,<耗时ms>,<总里程mm>,<线RMS>,<线P95>,<线MAX>,<惯导RMS>,<惯导P95>,<补偿角均值deg>,<脱线次数>,<结束原因>
```

数值用 `%.6g` 写出（6 位有效数字）；跑完 `结束原因=OK`，否则 `LOST`/`TIMEOUT`/…。
字段不足 13 个整行丢弃。

**③ 串口契约**

| 命令 | 期望回应 |
|---|---|
| `SET <键> <值>` | `OK`（**必须支持 GET 回读**，否则脚本判失败） |
| `GET <键>` | `OK <值>`（与写入值相对误差 ≤1e-3） |
| `RUN <n>` | `ACK`，随后连打 n 行 LAP（允许少于 n 圈，脚本会处理） |

## 4. ★ C 车适配状态（2026-10-18 更新）

> 原来这里列的 5 条缺口，**固件侧已经全部补上了**（见根目录 `交接-C车-20261018.md` §14）。
> 下面写"现在怎么接"，以及还剩什么。

| 原来的缺口 | 现在 |
|---|---|
| ① 固件侧「4 件套」未做（`SET`/`GET`、每圈 `LAP`、`AUTORUN`、急停） | **已做**：`User/tune.c` / `tune.h` 实现行协议 + 每圈 13 字段 `LAP` + `autorun` 键 + `STOP` 急停 + 30s 单圈超时自停 |
| ② 参数层命名对不上（`g_kp` vs 21 键） | **已做**：`tune.c` 里的键→变量映射表；**其中 9 个键在光电模式下真的有效**，其余是惰性键（表里有标注） |
| ③ 波特率 115200 vs 9600 | **已改 JSON 为 9600**，和固件 `USART1_BAUD_DEFAULT` 一致（DAPLink CDC 的 USB 侧不关心这个数，但 STM32 的 UART 必须对上） |
| ④ 种子参数取自 B 车 | **已全部换成 C 车自己的值和单位**（PWM 0~7199 / mm / 度 / 百分比），取自 `NAV.c` `mycontrol.c` `tune.c` 的当前默认值 |
| ⑤ 控制周期 5ms vs 主循环 ~450µs | `LAP` 统计在**控制拍**里累加、只在圈边界打一行；`RUN` 的圈判定用**里程绕回**（`NAV_LapSeq()`），和时间无关 |

**C 车实测有效的键**（其余键能读写但改了不生效，别浪费轮次）：

```
kp_steer  kd_steer  k_gyro(仅葫芦圈)  vmax  vmin
a_acc  a_brake  corner_margin  corr_enc_k  autorun
```

所以 B 阶段别用默认顺序（默认第一个是 `look_mm`，在光电模式下是惰性键），改成：

```bash
python 工具/调参迭代.py --port COM17 --phase B \
  --b-order kp_steer,kd_steer,corner_margin,a_brake,a_acc,k_gyro
python 工具/调参迭代.py --port COM17 --phase C --c-order a_brake,a_acc
```

`--b-order` / `--c-order` 是这一轮新加的参数（默认值还是 B 车那套，不动它，免得改坏 `--sim` 回归基线）。

**上车前先验协议**（不跑车）：

```bash
python 工具/测协议.py --port COM17            # 21 个键 GET 一遍
python 工具/测协议.py --port COM17 --write    # 再加 SET 回读校验
python 工具/测协议.py --port COM17 --run 1    # 车会真的动：发 RUN 1 等一行 LAP
```

**还剩的坑**：

1. `AUTO_START = 1` 上电 2 秒自动开跑 —— 调参时**先起脚本**（第一条 `SET autorun 0` 会把车按住），或者把车架空。
2. `vmax` 种子 = 7199 = PWM 满量程 = `vmax_ceiling`，所以 C-1 会立刻报"触顶"。想自动找速度上限，先把种子调低（如 6000）。
3. `P95` 分辨率 2mm（直方图桶宽），要更细就加桶数（RAM 代价）。
4. `LAP` 的"惯导横向"取 `NAV_GetCrossTrack()`，光电模式下这一轮才刚开始每 5ms 计算，第一次上车请顺便核对它和 `E:` 符号是否一致。

## 5. ⚠ 串车警告（照搬手册 §9 的纪律）

`代码策略与手法-给队友-照搬用-20261013.md` 里含 **B 车的引脚 / 外设实例 / 参数默认值 / 赛道几何** ——
**这些一律不要照抄进 C 车**（原文 §10 明确列了「按 C 车改」的部分）。C 车的引脚看根目录 `PINOUT.md`。

B 车用的是 **HAL + CubeMX + `src/` 分层**；C 车是 **SPL + Keil `HARDWARE/` `User/`**。
可照搬的是**组织方式和纪律**，不是代码。

## 6. 相关

- C 车引脚（权威）：根目录 `PINOUT.md`
- 导航地图（惯导用）：`导航地图/README.md`
- 固件入口文档（B 车，想抄具体实现看这个）：`test16/文档/交接-固件bcar-20261012.md`
