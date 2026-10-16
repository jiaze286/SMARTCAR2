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
/* ============================================================
 * 【毫秒时基】用 CPU 周期计数器 DWT->CYCCNT 生成，不再依赖 TIM1
 *
 * 为什么：实测 TIM1 配的 1kHz 实际只跑到约 9Hz（慢了 100 多倍），
 * 后果是"5ms 的陀螺节拍"变成 500ms 一拍，航向积分几乎不进账
 * （转 90 度只积出来几度）。DWT 是 CPU 核内的周期计数器，
 * 72MHz 下 72000 个周期就是 1ms，跟外设定时器无关，不会偏。
 *
 * TIM1 的中断仍然保留（多算一点点，不到 1%），当兜底。
 * ============================================================ */
#define TICK_CYC_PER_MS  72000u

void TICK_Init(void)
{
	DEMCR_REG    |= 0x01000000u;       /* TRCENA：允许跟踪 */
	DWT_CYCCNT    = 0;
	DWT_CTRL_REG |= 0x00000001u;       /* CYCCNTENA：开周期计数器 */
}

/* 主循环每圈调一次：把走过的 CPU 周期换算成毫秒累加进 g_ms_tick */
void TICK_Poll(void)
{
	static uint32_t s_last = 0u;
	static uint32_t s_rem  = 0u;
	uint32_t now = DWT_CYCCNT;
	uint32_t d;

	d = now - s_last;                  /* 无符号减法，回绕也正确 */
	s_last = now;

	s_rem += d;
	if (s_rem >= TICK_CYC_PER_MS)
	{
		uint32_t add = s_rem / TICK_CYC_PER_MS;
		s_rem -= add * TICK_CYC_PER_MS;
		g_ms_tick += add;
	}
}

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
