#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

/* ============================================================
 * 编码器里程计（JGA25-370 自带霍尔编码器）
 *
 * 接线（都是默认映射，不需要重映射）：
 *   左轮 A/B -> PA6/PA7 -> TIM3
 *   右轮 A/B -> PA0/PA1 -> TIM2
 *
 * 参数推导：
 *   编码器 11 PPR，4 倍频 -> 电机每转 44 计数
 *   减速比 1:21.3 -> 输出轴每转 44 * 21.3 = 937.2 计数
 *   轮径 65mm -> 周长 204.2mm
 *   所以 1 个计数 = 204.2 / 937.2 = 0.2179 mm
 *
 * 没配 100nF 滤波电容时的噪声很大，代码里靠 TIM_ICFilter 抑制。
 * ============================================================ */

/* 实测如果里程是倒着走的，把对应的宏改成 1 */
#define ENCODER_SWAP_L   0
#define ENCODER_SWAP_R   1

/* 每个计数对应的距离（米） */
#define ENCODER_M_PER_COUNT   0.0002179f

void    ENCODER_Init(void);

/* 由 Control_Tick 每轮调用：读计数、累加里程。
   在主循环里跑（约 450us 一次），16 位计数器不会溢出。 */
void    ENCODER_Update(void);

/* 里程清零（每次启动循迹时调用） */
void    ENCODER_Reset(void);

/* ---- 给上层用 ---- */
float   ENCODER_GetDistL(void);      /* 左轮累计距离，米 */
float   ENCODER_GetDistR(void);      /* 右轮累计距离，米 */
float   ENCODER_GetDist(void);       /* 两轮平均，米 */

/* 上一轮的计数差（阶段二做速度环时用得上） */
int32_t ENCODER_GetDeltaL(void);
int32_t ENCODER_GetDeltaR(void);

#endif
