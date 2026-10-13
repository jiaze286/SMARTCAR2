#ifndef __TIM4_PWM_H
#define __TIM4_PWM_H

#include "stm32f10x.h"

/* ============================================================
 * TIM4 双通道 PWM —— 驱动 TB6612 的两路调速
 *
 *   PWMA -> PB6 -> TIM4_CH1 -> 左轮
 *   PWMB -> PB7 -> TIM4_CH2 -> 右轮
 *
 * 这两个引脚是 TIM4 的默认映射，不需要任何重映射。
 * ============================================================ */

/* 与 TIM4_PWM_Init 的 arr 实参保持一致，MOTOR.c 用它当速度上限 */
#define TIM4_PWM_ARR   7199

void TIM4_PWM_Init(u16 arr, u16 psc);

#endif
