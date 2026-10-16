#ifndef __TIM1_TICK_H
#define __TIM1_TICK_H

#include "stm32f10x.h"

/* ============================================================
 * TIM1 系统时基 —— 1ms 递增一次的全局计数器
 *
 * 当前用途：蓝牙失联计时
 * 后续用途：双环 PID 的定时控制周期
 *
 * 中断优先级取最低（抢占3/子3），不会影响 USART1（抢占1/子0）。
 * ============================================================ */

extern volatile uint32_t g_ms_tick;   /* 上电以来经过的毫秒数，溢出周期约 49.7 天 */

void TIM1_TICK_Init(void);

/* 真毫秒时基（DWT 周期计数）：先 TICK_Init() 一次，之后主循环每圈 TICK_Poll() */
void TICK_Init(void);
void TICK_Poll(void);

/* DWT 周期计数器（CMSIS 版本较老，这里直接给地址）
   0xE0001000 = DWT_CTRL，0xE0001004 = DWT_CYCCNT，0xE000EDFC = DEMCR */
#define DWT_CTRL_REG   (*(volatile uint32_t *)0xE0001000u)
#define DWT_CYCCNT     (*(volatile uint32_t *)0xE0001004u)
#define DEMCR_REG      (*(volatile uint32_t *)0xE000EDFCu)

#endif
