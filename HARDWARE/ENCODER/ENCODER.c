#include "ENCODER.h"

volatile int32_t g_enc_k = 1000;           /* 里程标定 ×1000（见 ENCODER.h） */

static volatile float   s_dist_l = 0.0f;   /* 左轮累计距离（米） */
static volatile float   s_dist_r = 0.0f;   /* 右轮累计距离（米） */
static volatile int32_t s_delta_l = 0;     /* 上一轮计数差 */
static volatile int32_t s_delta_r = 0;

static uint16_t s_last_l = 0;              /* 上次读到的计数值 */
static uint16_t s_last_r = 0;

/* 把一个定时器配成编码器接口模式（TI12：两个通道都计数，4 倍频） */
static void Enc_Init_TIM(TIM_TypeDef* TIMx)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_ICInitTypeDef       TIM_ICInitStructure;

	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler     = 0;
	TIM_TimeBaseStructure.TIM_Period        = 65535;          /* 满量程计数 */
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);

	TIM_EncoderInterfaceConfig(TIMx, TIM_EncoderMode_TI12,
	                           TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

	TIM_ICStructInit(&TIM_ICInitStructure);
	/* 滤波要拉满：电机碳刷噪声很大，不滤会一直多计数 */
	TIM_ICInitStructure.TIM_ICFilter = 15;
	TIM_ICInit(TIMx, &TIM_ICInitStructure);

	TIM_SetCounter(TIMx, 0);
	TIM_Cmd(TIMx, ENABLE);
}

void ENCODER_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 | RCC_APB1Periph_TIM3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	/* 用上拉输入而不是浮空：万一编码器是开漏输出，浮空会读到乱码；
	   推挽输出时上拉也无害。 */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* 左轮 A/B -> PA6/PA7 -> TIM3 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 右轮 A/B -> PA0/PA1 -> TIM2 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	Enc_Init_TIM(TIM3);   /* 左 */
	Enc_Init_TIM(TIM2);   /* 右 */

	ENCODER_Reset();
}

void ENCODER_Update(void)
{
	uint16_t now_l, now_r;
	int16_t  d_l, d_r;

	now_l = TIM_GetCounter(TIM3);
	now_r = TIM_GetCounter(TIM2);

	/* 16 位计数器回绕靠 int16_t 相减自动处理 */
	d_l = (int16_t)(now_l - s_last_l);
	d_r = (int16_t)(now_r - s_last_r);

	s_last_l = now_l;
	s_last_r = now_r;

#if ENCODER_SWAP_L
	d_l = -d_l;
#endif
#if ENCODER_SWAP_R
	d_r = -d_r;
#endif

	s_delta_l = d_l;
	s_delta_r = d_r;

	s_dist_l += (float)d_l * ENCODER_M_PER_COUNT * (float)g_enc_k * 0.001f;
	s_dist_r += (float)d_r * ENCODER_M_PER_COUNT * (float)g_enc_k * 0.001f;
}

void ENCODER_Reset(void)
{
	TIM_SetCounter(TIM3, 0);
	TIM_SetCounter(TIM2, 0);

	s_last_l  = 0;
	s_last_r  = 0;
	s_delta_l = 0;
	s_delta_r = 0;
	s_dist_l  = 0.0f;
	s_dist_r  = 0.0f;
}

float ENCODER_GetDistL(void) { return s_dist_l; }
float ENCODER_GetDistR(void) { return s_dist_r; }
float ENCODER_GetDist(void)  { return (s_dist_l + s_dist_r) * 0.5f; }

int32_t ENCODER_GetDeltaL(void) { return s_delta_l; }
int32_t ENCODER_GetDeltaR(void) { return s_delta_r; }
