#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"
#include "TIM4_PWM.h"

/* ============================================================
 * TB6612 电机驱动
 *
 *   PWMA -> PB6  -> TIM4_CH1 -> 左轮
 *   PWMB -> PB7  -> TIM4_CH2 -> 右轮
 *   AIN1 -> PB12   AIN2 -> PB13     左轮方向
 *   BIN1 -> PB14   BIN2 -> PB15     右轮方向
 *   STBY -> PB11                    使能，必须为高
 *
 * 要改接线，只改下面这几个宏。
 * ============================================================ */

#define MOTOR_PORT       GPIOB
#define MOTOR_PIN_STBY   GPIO_Pin_11
#define MOTOR_PIN_AIN1   GPIO_Pin_12
#define MOTOR_PIN_AIN2   GPIO_Pin_13
#define MOTOR_PIN_BIN1   GPIO_Pin_14
#define MOTOR_PIN_BIN2   GPIO_Pin_15

/* 速度范围：-MOTOR_SPEED_MAX ~ +MOTOR_SPEED_MAX，正数前进 */
#define MOTOR_SPEED_MAX  TIM4_PWM_ARR

/* ------------------------------------------------------------
 * 实测时如果某个轮子转反了，把对应的宏改成 1 就行，
 * 不用动接线、也不用改别的代码。
 * ------------------------------------------------------------ */
#define MOTOR_SWAP_LEFT   1
#define MOTOR_SWAP_RIGHT  0

void MOTOR_GPIO_Init(void);

/* Left_speed / Right_speed :
 *     正数 = 前进，负数 = 后退，0 = 刹车
 *     绝对值 = PWM 占空比，上限 MOTOR_SPEED_MAX
 */
void Set_Car_Speed(int Left_speed, int Right_speed);

#endif
