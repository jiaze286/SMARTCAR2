#include "USART1.h"
#include "TIM1_TICK.h"

volatile uint8_t  RxData = 0x00;
volatile uint8_t  g_rx_flag = 0;
volatile uint32_t g_last_rx_tick = 0;

/* ---- 发送环形缓冲（单生产者单消费者，主循环写、中断读）---- */
static volatile uint8_t  s_tx_buf[USART1_TX_BUF_SIZE];
static volatile uint16_t s_tx_head = 0;   /* 主循环写指针 */
static volatile uint16_t s_tx_tail = 0;   /* 中断读指针 */

/* ============================================================
 * USART1 初始化
 * ============================================================ */
void USART1_Init(uint32_t bound)
{
	GPIO_InitTypeDef  GPIO_Initstructure;
	NVIC_InitTypeDef  NVIC_InitStruct;
	USART_InitTypeDef USART1_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* USART1_TX  PA9 */
	GPIO_Initstructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_Initstructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_Initstructure);

	/* USART1_RX  PA10 */
	GPIO_Initstructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Initstructure.GPIO_Pin   = GPIO_Pin_10;
	GPIO_Initstructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_Initstructure);

	USART1_InitStructure.USART_BaudRate            = bound;
	USART1_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART1_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART1_InitStructure.USART_Parity              = USART_Parity_No;
	USART1_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART1_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_Init(USART1, &USART1_InitStructure);

	USART_Cmd(USART1, ENABLE);

	/* 串口用抢占 1 / 子 0，比 TIM1 时基（3/3）高，保证指令不丢 */
	NVIC_InitStruct.NVIC_IRQChannel                   = USART1_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelCmd                = ENABLE;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority        = 0;
	NVIC_Init(&NVIC_InitStruct);

	/* 只开接收中断；发送中断等有数据要发时再打开 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ClearFlag(USART1, USART_FLAG_TC);
}

/* ============================================================
 * 接收 + 发送中断
 * ============================================================ */
void USART1_IRQHandler(void)
{
	/* ---- 接收 ---- */
	if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) == SET)
	{
		/* 溢出标志必须处理：读一次 DR 就能清掉。否则 ORE 置位后
		   RXNE 不再更新，串口会彻底收不到东西。 */
		(void)USART_ReceiveData(USART1);
	}

	if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
		RxData = USART_ReceiveData(USART1);   /* 读 DR 会自动清掉 RXNE */
		g_last_rx_tick = g_ms_tick;
		g_rx_flag = 1;
	}

	/* ---- 发送：缓冲里还有就继续发，空了就关掉发送中断 ---- */
	if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
	{
		if (s_tx_tail != s_tx_head)
		{
			USART_SendData(USART1, s_tx_buf[s_tx_tail]);
			s_tx_tail = (uint16_t)((s_tx_tail + 1) % USART1_TX_BUF_SIZE);
		}
		else
		{
			USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
		}
	}
}

/* ============================================================
 * 发送（非阻塞）
 * ============================================================ */
void USART1_SendByte(uint8_t dat)
{
	uint16_t next = (uint16_t)((s_tx_head + 1) % USART1_TX_BUF_SIZE);

	if (next == s_tx_tail) return;     /* 缓冲满了直接丢，绝不阻塞 */

	s_tx_buf[s_tx_head] = dat;
	s_tx_head = next;

	USART_ITConfig(USART1, USART_IT_TXE, ENABLE);
}

void USART1_SendString(const char *str)
{
	while (*str) USART1_SendByte((uint8_t)(*str++));
}

void USART1_SendNum(int32_t num)
{
	char    buf[12];
	int8_t  i = 0;
	uint8_t neg = 0;

	if (num < 0) { neg = 1; num = -num; }

	do
	{
		buf[i++] = (char)('0' + (num % 10));
		num /= 10;
	} while (num != 0 && i < 11);

	if (neg) buf[i++] = '-';

	while (i > 0) USART1_SendByte((uint8_t)buf[--i]);
}

uint16_t USART1_TxPending(void)
{
	return (uint16_t)((s_tx_head + USART1_TX_BUF_SIZE - s_tx_tail) % USART1_TX_BUF_SIZE);
}

/* ============================================================
 * 失联检测
 *   用无符号减法，g_ms_tick 溢出（约 49.7 天）时结果依然正确。
 * ============================================================ */
uint8_t USART1_IsLost(void)
{
	return ((uint32_t)(g_ms_tick - g_last_rx_tick) > USART1_LINK_TIMEOUT_MS) ? 1 : 0;
}
