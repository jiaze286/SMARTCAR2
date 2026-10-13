#include "GRAY.h"
#include "Delay.h"

/* 地址脚：AD0/AD1/AD2 -> PA2/PA3/PA4，AD0 是最低位 */
#define GRAY_ADDR_PORT   GPIOA
#define GRAY_PIN_AD0     GPIO_Pin_2
#define GRAY_PIN_AD1     GPIO_Pin_3
#define GRAY_PIN_AD2     GPIO_Pin_4
#define GRAY_PIN_OUT     GPIO_Pin_5

/* 加权重心权重：**非等距，边缘权重更大**（亚博官方做法）
   这样车在偏离中心时修正得更猛，压中心附近时更柔和，过弯不容易画龙。 */
static const int8_t s_weight[GRAY_CH_NUM] = {-5, -4, -2, -1, 1, 2, 4, 5};

static uint8_t s_state[GRAY_CH_NUM];   /* 每路状态：1 = 压线 */
static int16_t s_error;                /* 加权重心偏差 */
static uint8_t s_lost;                 /* 1 = 丢线 */
static uint8_t s_all_on;               /* 1 = 8 路全压线 */

void GRAY_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* AD0/AD1/AD2：推挽输出，选择通道 */
	GPIO_InitStructure.GPIO_Pin   = GRAY_PIN_AD0 | GRAY_PIN_AD1 | GRAY_PIN_AD2;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GRAY_ADDR_PORT, &GPIO_InitStructure);

	/* OUT：浮空输入。模块输出 5V，PA5 在 F103 上是 FT（5V 容忍）引脚，
	   数字输入接 5V 无风险，不需要分压。 */
	GPIO_InitStructure.GPIO_Pin  = GRAY_PIN_OUT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GRAY_ADDR_PORT, &GPIO_InitStructure);
}

/* 选通道：ch = 0~7，AD0 是最低位 */
static void GRAY_SelectChannel(uint8_t ch)
{
	if (ch & 0x01) GPIO_SetBits  (GRAY_ADDR_PORT, GRAY_PIN_AD0);
	else           GPIO_ResetBits(GRAY_ADDR_PORT, GRAY_PIN_AD0);

	if (ch & 0x02) GPIO_SetBits  (GRAY_ADDR_PORT, GRAY_PIN_AD1);
	else           GPIO_ResetBits(GRAY_ADDR_PORT, GRAY_PIN_AD1);

	if (ch & 0x04) GPIO_SetBits  (GRAY_ADDR_PORT, GRAY_PIN_AD2);
	else           GPIO_ResetBits(GRAY_ADDR_PORT, GRAY_PIN_AD2);
}

/* ============================================================
 * 扫一遍 8 路，并算出加权重心偏差
 *
 *   error = Σ(压线通道的权重) / 压线通道数
 *
 * 取平均而不是直接求和，是为了让偏差量级不随压线通道数变化，
 * 这样 Kp 调好之后在直道（1路压线）和弯道（3路压线）都合用。
 * ============================================================ */
void GRAY_ReadAll(void)
{
	uint8_t i;
	uint8_t cnt = 0;
	int32_t sum = 0;

	for (i = 0; i < GRAY_CH_NUM; i++)
	{
		GRAY_SelectChannel(i);

		/* 官方硬性要求：通道切换后必须等 50us 让模拟开关稳定，不能省。
		   因为权重只跟"压没压线"有关，采样抖动会直接变成转向抖动。 */
		Delay_us(50);

		s_state[i] = (GPIO_ReadInputDataBit(GRAY_ADDR_PORT, GRAY_PIN_OUT) == GRAY_LINE_VALUE) ? 1 : 0;

		if (s_state[i])
		{
			sum += s_weight[i];
			cnt++;
		}
	}

	s_all_on = (cnt == GRAY_CH_NUM) ? 1 : 0;

	if (cnt == 0)
	{
		s_lost  = 1;
		s_error = 0;        /* 丢线时偏差无意义，由上层用"上次方向"接管 */
	}
	else
	{
		s_lost  = 0;
		s_error = (int16_t)(sum * GRAY_ERR_SCALE / (int32_t)cnt);
	}
}

uint8_t GRAY_GetChannel(uint8_t i)
{
	return (i < GRAY_CH_NUM) ? s_state[i] : 0;
}

int16_t GRAY_GetError(void) { return s_error;  }
uint8_t GRAY_IsLost(void)   { return s_lost;   }
uint8_t GRAY_IsAllOn(void)  { return s_all_on; }
