#include "TIM1_TICK.h"

volatile uint32_t g_ms_tick = 0;

/* ============================================================
 * TIM1 配置成 1ms 周期中断
 *   72MHz / (71+1) = 1MHz，再 /(999+1) = 1kHz -> 每 1ms 一次更新中断
 *
 * 中断里只做一件事：给 g_ms_tick 加一。越短越好。
 * 用途：蓝牙失联计时、OLED 刷新限速。
 *
 * 【车辆控制不在这里】——控制跑在主循环里（见 main.c 的 Control_Tick）。
 * 原因：灰度采样一轮要 8 x 50us = 400us，如果塞进中断，
 *       控制周期最快也只能到 1ms；放在主循环里可以跑到约 450us 一圈，
 *       响应快一倍多。对循迹来说响应速度比"周期严格固定"重要得多。
 *
 * 注意：中断服务函数写在本文件里（模块自包含），
 *       不要在 USER/stm32f10x_it.c 里再写一份，否则链接冲突。
 * ============================================================ */
void TIM1_TICK_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef        NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);   /* TIM1 挂在 APB2 */

	TIM_TimeBaseStructure.TIM_Period        = 999;
	TIM_TimeBaseStructure.TIM_Prescaler     = 71;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

	/* 优先级放到最低，别抢串口 */
	NVIC_InitStructure.NVIC_IRQChannel                   = TIM1_UP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	TIM_Cmd(TIM1, ENABLE);
}

/* TIM1 更新中断：只做加一，越短越好 */
void TIM1_UP_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
		g_ms_tick++;
	}
}
