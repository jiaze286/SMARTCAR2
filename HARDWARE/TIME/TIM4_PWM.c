#include "TIM4_PWM.h"

/* ============================================================
 * TIM4 双通道 PWM 初始化
 *
 * 调用示例：TIM4_PWM_Init(7199, 1);
 *   72MHz / (1+1) / (7199+1) = 5kHz
 *
 * 改占空比：TIM_SetCompare1(TIM4, x)  -> PB6 左轮
 *           TIM_SetCompare2(TIM4, x)  -> PB7 右轮
 * ============================================================ */
void TIM4_PWM_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef       TIM_OCInitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);   /* TIM4 挂在 APB1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* PB6 = TIM4_CH1（PWMA，左轮），PB7 = TIM4_CH2（PWMB，右轮） */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;        /* 复用推挽输出 */
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	TIM_TimeBaseStructure.TIM_Period        = arr;
	TIM_TimeBaseStructure.TIM_Prescaler     = psc;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
	/* 必须显式赋值：SPL 会把它写进 CCR，栈上的垃圾值会让电机上电就窜一下 */
	TIM_OCInitStructure.TIM_Pulse       = 0;

	TIM_OC1Init(TIM4, &TIM_OCInitStructure);   /* CH1 -> PB6 左轮 */
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);   /* CH2 -> PB7 右轮 */

	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);

	TIM_Cmd(TIM4, ENABLE);
}
