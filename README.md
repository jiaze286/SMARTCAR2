# SMARTCAR2 · 循迹小车 C 车代码库

西南石油大学校赛智能车（2026-10）—— **三轮差速循迹竞速车**
STM32F103C8T6 + 标准外设库(SPL) + Keil MDK

赛道：**黑底白线**

---

## 硬件

| 部分 | 说明 |
|---|---|
| 主控 | STM32F103C8T6 核心板（插在自制 PCB 上） |
| 驱动 | TB6612FNG ×1 → 2× JGA25-370 编码器电机（减速比 1:21.3，轮径 65mm） |
| 循迹 | 亚博 8 路灰度模块（CD4051 复用，**数字量输出**） |
| 测速 | 正交编码器（左 TIM3，右 TIM2） |
| 姿态 | MPU-6050（I2C1 重映射到 PB8/PB9） |
| 底盘 | 三轮差速：后两轮驱动 + 前万向轮 |
| 通信 | JDY-31 蓝牙（USART1，9600） |
| 显示 | **无 OLED** —— 所有调试信息走蓝牙串口 |

## 引脚分配

```
左PWM   PB6  TIM4_CH1        右PWM   PB7  TIM4_CH2
AIN1/2  PB12/PB13            BIN1/2  PB14/PB15       STBY  PB11
灰度    AD0/AD1/AD2 = PA2/PA3/PA4      OUT = PA5
编码器  左 PA6/PA7 (TIM3)              右 PA0/PA1 (TIM2)
MPU6050 SCL=PB8 SDA=PB9 (I2C1 重映射)
蓝牙    PA9/PA10              LED     PC13
时基    TIM1 1ms 中断 (g_ms_tick)
```

## 代码结构

```
USER/
  main.c          主循环：蓝牙指令 / 遥测 / 模式切换
  mycontrol.c/.h  控制核心：Trace_Run() + Manual_Run() + Control_Tick()
HARDWARE/
  GRAY/           8 路灰度：加权重心算偏差
  IMU/            MPU-6050 驱动（陀螺仪）
  ENCODER/        编码器里程计
  MOTOR/          TB6612 驱动
  TIME/           TIM4 PWM + TIM1 时基
  USART/          蓝牙 + 非阻塞发送
System/ Start/ Library/   标准外设库与启动文件
```

## 架构要点

- **控制跑在主循环里**，一圈约 450µs（灰度采样 400µs 占大头）。
  **不要挪进定时器中断** —— 灰度采样是硬约束，进中断后控制周期只能到 1ms，慢一倍多。
- **`Set_Car_Speed()` 只由 `Control_Tick()` 调用**，别的地方不许碰电机。
- `GRAY_ReadAll()` 里的 `Delay_us()` 是**轮询 SysTick** 实现的 ——
  主循环里不能再有别的 Delay 调用，否则会卡死。

## 控制律

```
err    = 加权重心偏差（已放大 GRAY_ERR_SCALE=10 倍）
slow   = g_kslow * |err|
base   = g_base_speed - slow
corr   = -(g_kp*err + g_kd*Δerr) - g_kgyro*ω     ← ω = 陀螺实测角速度
左轮   = base - corr
右轮   = base + corr
```

## 可调参数（`USER/mycontrol.c` 开头）

| 参数 | 默认 | 说明 |
|---|---|---|
| `g_base_speed` | 3900 | 直道基础速度（PWM 占空比，上限 7199） |
| `g_kp` | 125 | 转向环 P |
| `g_kd` | 400 | 转向环 D（基于误差微分，噪声大） |
| `g_kgyro` | 0 | **陀螺阻尼**增益，0=关闭 |
| `g_kslow` | 32 | 弯道降速系数 |
| `g_manual_fb` | 5000 | 手动前进/后退速度 |

## 调试开关（`USER/main.c`）

| 开关 | 默认 | 说明 |
|---|---|---|
| `AUTO_MOTOR_TEST` | 0 | 上电跑一遍电机自检（前进/后退/左转/右转） |
| `AUTO_TRACE` | 0 | 上电直接进循迹，不等蓝牙指令 |
| `USE_IMU`（在 `IMU.h`） | 1 | 0 = 完全不用陀螺仪 |

## 蓝牙指令

```
0x00 停止     0x01 前进     0x02 后退
0x03 左转     0x04 右转
0x05 循迹 暂停/继续        0x06 启动循迹
```
手动模式下蓝牙失联 500ms 自动停车；循迹模式不检查失联。

## 串口遥测（9600，连上就发）

**数据行（10Hz）**
```
M:2 E:-2 G:00111100 D:123456,123456 V:12,13 W:12 L:0 A:0
M 模式 / E 偏差 / G 8路灰度 / D 里程mm / V 轮速 / W 陀螺 / L 丢线 / A 全压线
```

**状态行（变化时 + 每 3 秒兜底）**
```
S:P125 S32 B3900 D400 K0 I0 ID:104
P=Kp S=kslow B=base D=Kd K=Kgyro I=IMU状态 ID=WHO_AM_I
```

## 踩过的坑

1. **赛道颜色决定极性**：黑底白线 → `GRAY_LINE_VALUE=0`；白底黑线 → `1`。
   换场地必须重新确认
2. **控制不要放定时器中断**（见上）
3. **源码是 GBK 编码**，别存成 UTF-8
4. **转向符号**：`err<0` = 线在车的左边，该左转
5. **烧录后看 Build Output 的 `Load "..."` 路径** —— 烧错工程查半天
6. **`OLED_ShowFloat()` 不能用**：内部整屏刷新，且最大只到 999.99

## 编译 / 烧录

Keil uVision5 打开 `Project.uvprojx`，或命令行：
```
"E:\Keil5\UV4\UV4.exe" -b Project.uvprojx -j0 -o build.log   # 编译
"E:\Keil5\UV4\UV4.exe" -f Project.uvprojx -j0 -o flash.log   # 烧录
```
（把 Keil 路径换成你机器上的）

## 参考

- `桌面\SMARTCAR2`（队友的微缩车工程）—— 惯导为主+光电为辅的思路来源
- `桌面\两轮PID循迹` —— 早期参考
