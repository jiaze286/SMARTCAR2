循迹小车 固件备份
================================================================
备份时间：2026-09-29
状态：赛道上跑通 —— 正常循迹 + 里程触发右转 + 里程计 + 串口遥测

【怎么打开】
  用 Keil uVision5 打开   TraceCar\Project.uvprojx
  工程配置里 "Create HEX File" 已经勾上，编译即出 hex
  编译产物 Objects\Project.hex 就是当时验证通过的固件

【当前参数】在 USER\mycontrol.c 开头
  g_base_speed     = 3000    直道基础速度
  g_kp             = 260     转向环 P（内轮速差 = 2*Kp*|err|）
  g_kd             = 0       转向环 D（没用到）
  g_kslow          = 200     弯道降速：base -= kslow*|err|
  g_turn_dist_mm   = 24621   右转触发里程 mm（0 = 关闭这个动作）
  g_turn_track_mm  = 150     轮距 mm（实测 15cm）
  g_turn_angle_deg = 60      开环转弯角度，剩下的交给巡线 PD

  HARDWARE\ENCODER\ENCODER.h:
    ENCODER_SWAP_L = 0   ENCODER_SWAP_R = 1   （实测定的）
    ENCODER_M_PER_COUNT = 0.0002179f
  HARDWARE\GRAY\GRAY.h:
    GRAY_LINE_VALUE = 0

【硬件】
  主控  STM32F103C8T6        驱动  TB6612FNG
  电机  JGA25-370 编码器 ×2，减速比 1:21.3，轮径 65mm，三轮差速
  循迹  亚博 8 路灰度（CD4051 复用）    赛道【黑底白线】
  蓝牙  JDY-31 9600           显示  0.96" OLED 软件 IIC

【引脚】
  左PWM PB6/ TIM4_CH1    右PWM PB7/ TIM4_CH2
  AIN1/2 PB12/PB13       BIN1/2 PB14/PB15       STBY PB11
  灰度   AD0/1/2 = PA2/PA3/PA4    OUT = PA5
  编码器 左 PA6/PA7 (TIM3)        右 PA0/PA1 (TIM2)
  蓝牙   PA9/PA10        OLED PA11/PA12        LED PC13
  时基   TIM1 1ms 中断（g_ms_tick）

【蓝牙指令】
  0x00 停止   0x01 前进   0x02 后退   0x03 左转   0x04 右转
  0x05 循迹 暂停/继续      0x06 启动循迹
  （手动模式下蓝牙失联 500ms 自动停车；循迹模式不检查失联）

【串口遥测】连上蓝牙后每 100ms 一行
  E:-2 G:00111100 D:123456,123456 V:12,13
  E 偏差 / G 8路灰度 / D 左右轮里程mm / V 左右轮本周期计数差

================================================================
【踩过的坑 —— 改代码前先看这几条】
================================================================
1. 赛道是【黑底白线】，灰度模块出厂按白底黑线设计（它把"黑"当线）
   → GRAY_LINE_VALUE 必须是 0。判据：8 路全压白线时模块的灯全灭

2. 控制跑在【主循环】里，不要挪进定时器中断！
   灰度采样一轮要 400us（8路 x 50us），进 1ms 中断的话控制周期
   只能到 1ms，比主循环（约 450us）慢一倍多。
   实测过：挪进 10ms 中断后车「响应变慢、直道画龙、弯道切线」

3. GRAY_ReadAll() 里的 Delay_us() 是【轮询 SysTick】实现的
   → 主循环里不能再有别的 Delay 调用（比如 LED_Flash），
     否则两边抢 SysTick 会卡死

4. 源码是 GBK 编码。改文件时别存成 UTF-8，否则 Keil 里中文注释乱码

5. 转向符号容易搞反：err < 0 = 线在车的【左边】= 该左转（左轮减速）
   代码里是 corr = -(kp*err + kd*Δerr)，写成正号车会朝反方向打舵

6. 里程计的左右轮方向不对称：
   MOTOR_SWAP_LEFT = 1（左电机反）
   ENCODER_SWAP_R  = 1（右编码器反）
   都是实测定的，别照理论推

7. OLED_ShowFloat() 不能用 —— 它每次调用都整屏刷新（5~8ms），
   而且最大只能显示 999.99。里程显示是手写的
