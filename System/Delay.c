#include "stm32f10x.h"

/**
  * @brief  微秒级延时
  * @param  xus 延时时长，范围：0~233015
  * @retval 无
  */
/* ============================================================
 * 【重要】本模块是【轮询 SysTick】实现的，不是中断。
 *
 * 车辆控制跑在 TIM1 的 10ms 中断里，而中断里会调用 Delay_us(50)
 * （灰度模块换通道后必须等 50us）。如果主循环里也调用 Delay_xxx，
 * 两边会互相抢 SysTick：中断把 SysTick 关掉之后，主循环会卡死在
 * `while(!(SysTick->CTRL & 0x00010000))` 这一句上，程序假死。
 *
 * 结论：**主循环里绝对不能调用 Delay_ms / Delay_us / Delay_s**
 *       （包括 LED_Flash()，它内部有 Delay_ms(100)）。
 *       main.c 目前没有调用，改代码时留意。
 * ============================================================ */
void Delay_us(uint32_t xus)
{
	SysTick->LOAD = 72 * xus;				//设置定时器重装值
	SysTick->VAL = 0x00;					//清空当前计数值
	SysTick->CTRL = 0x00000005;				//设置时钟源为HCLK，启动定时器
	while(!(SysTick->CTRL & 0x00010000));	//等待计数到0
	SysTick->CTRL = 0x00000004;				//关闭定时器
}

/**
  * @brief  毫秒级延时
  * @param  xms 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_ms(uint32_t xms)
{
	while(xms--)
	{
		Delay_us(1000);
	}
}
 
/**
  * @brief  秒级延时
  * @param  xs 延时时长，范围：0~4294967295
  * @retval 无
  */
void Delay_s(uint32_t xs)
{
	while(xs--)
	{
		Delay_ms(1000);
	}
} 
