#include "MOTOR.h"
#include "Delay.h"

/* 初始化 TB6612 的 5 个控制脚 */
void MOTOR_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin   = MOTOR_PIN_STBY | MOTOR_PIN_AIN1 | MOTOR_PIN_AIN2
	                              | MOTOR_PIN_BIN1 | MOTOR_PIN_BIN2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(MOTOR_PORT, &GPIO_InitStructure);

	/* STBY 必须拉高。低电平时 TB6612 进待机、输出全高阻，
	   PWM 怎么给电机都不会动 —— 这是最常见的"程序没错但车不动"的原因。 */
	GPIO_SetBits(MOTOR_PORT, MOTOR_PIN_STBY);

	/* 上电先刹车，防止意外窜车 */
	Set_Car_Speed(0, 0);
}

/* 按符号把一对方向脚设成 正转 / 反转 / 刹车 */
static void Motor_SetDir(uint16_t pin_fwd, uint16_t pin_rev, int speed)
{
	if (speed > 0)              /* 正转 */
	{
		GPIO_SetBits(MOTOR_PORT, pin_fwd);
		GPIO_ResetBits(MOTOR_PORT, pin_rev);
	}
	else if (speed < 0)         /* 反转 */
	{
		GPIO_ResetBits(MOTOR_PORT, pin_fwd);
		GPIO_SetBits(MOTOR_PORT, pin_rev);
	}
	else                        /* 停止：两脚同时拉高 = 电机两端短接 = 刹车 */
	{
		GPIO_SetBits(MOTOR_PORT, pin_fwd);
		GPIO_SetBits(MOTOR_PORT, pin_rev);
	}
}

/* 取绝对值并限幅 */
static uint16_t Motor_Abs(int speed)
{
	if (speed >  MOTOR_SPEED_MAX) speed =  MOTOR_SPEED_MAX;
	if (speed < -MOTOR_SPEED_MAX) speed = -MOTOR_SPEED_MAX;
	return (uint16_t)((speed < 0) ? -speed : speed);
}

/* ============================================================
 * 设置左右轮速度
 *   Left_speed / Right_speed : -MOTOR_SPEED_MAX ~ +MOTOR_SPEED_MAX
 *   正数前进，负数后退，0 刹车
 * ============================================================ */
void Set_Car_Speed(int Left_speed, int Right_speed)
{
#if MOTOR_SWAP_LEFT
	Left_speed = -Left_speed;
#endif
#if MOTOR_SWAP_RIGHT
	Right_speed = -Right_speed;
#endif

	/* 左轮：AO1/AO2 由 PWMA 驱动 */
	Motor_SetDir(MOTOR_PIN_AIN1, MOTOR_PIN_AIN2, Left_speed);
	TIM_SetCompare1(TIM4, Motor_Abs(Left_speed));    /* PB6 = TIM4_CH1 */

	/* 右轮：BO1/BO2 由 PWMB 驱动 */
	Motor_SetDir(MOTOR_PIN_BIN1, MOTOR_PIN_BIN2, Right_speed);
	TIM_SetCompare2(TIM4, Motor_Abs(Right_speed));   /* PB7 = TIM4_CH2 */
}
