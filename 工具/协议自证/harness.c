/* 台架仿真：把 User/tune.c 原样编译到 PC 上，用 stdin/stdout 跑那套行协议。
   传感器、电机、里程全部是桩；只验证"协议 + 统计 + 判圈"这条链路。 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pthread.h>

#include "tune.h"
#include "USART1.h"
#include "GRAY.h"
#include "ENCODER.h"
#include "NAV.h"
#include "mycontrol.h"
#include "TIM1_TICK.h"

/* ---------------- 仿真状态 ---------------- */
volatile uint32_t g_ms_tick = 0;

static volatile int     s_running = 0;
static volatile double  s_dist_m  = 0.0;
static volatile int32_t s_lap_seq = 0;
static volatile int32_t s_ct      = 0;
static volatile int32_t s_comp    = 0;
static int32_t          s_noise_i = 0;

#define LAP_MM   36680.0        /* 内置路表 1834 点 x 20mm */
#define MM_PER_MS 4.4           /* 约 4.4 m/s? 不管：一圈 8.3 秒就够 */

/* ---------------- 参数变量（tune.c 会写） ---------------- */
volatile int32_t g_kp = 70000, g_kd = 0, g_kgyro = 3000, g_kslow = 16000;
volatile int32_t g_ramp_up = 18, g_ramp_dn = 25, g_trace_vmin = 300;
volatile int     g_base_speed = 3900;
volatile int     g_nav_vmax = 3200, g_nav_vmin = 900, g_nav_look_mm = 160;
volatile int     g_nav_map_vmax = 7199;
volatile int     g_nav_corr_k = 800, g_nav_corr_on = 1;
volatile int     g_nav_corner_pct = 100, g_nav_straight_min = 200, g_nav_straight_deg10 = 30;
volatile int     g_nav_mm_per_e = 1;
volatile int32_t g_enc_k = 1000;

/* ---------------- 传感器桩 ---------------- */
int16_t GRAY_GetError(void)
{
	/* 带点噪声的"贴线跑"：幅值 8 左右，偶尔 25（好让 MAX/RMS 有内容） */
	static const int16_t seq[12] = { 0, 3, -4, 6, -2, 8, -7, 2, 12, -9, 25, -1 };
	s_noise_i++;
	return seq[(unsigned)s_noise_i % 12u];
}

uint8_t GRAY_IsLost(void) { return 0; }

float ENCODER_GetDist(void) { return (float)s_dist_m; }

int32_t NAV_LapSeq(void)    { return s_lap_seq; }
int32_t NAV_GetCrossTrack(void) { return s_ct; }
int32_t NAV_YawComp10(void) { return s_comp; }

/* ---------------- 开跑/停车（main.c 里的那两个） ---------------- */
void App_StartRun(void)
{
	s_dist_m  = 0.0;
	s_lap_seq = 0;
	s_running = 1;
	fprintf(stderr, "[sim] StartRun\n");
}

void App_Stop(void)
{
	s_running = 0;
	fprintf(stderr, "[sim] Stop\n");
}

/* ---------------- 串口桩 ---------------- */
static void out_byte(uint8_t b) { fputc(b, stdout); fflush(stdout); }

void USART1_SendByte(uint8_t dat) { out_byte(dat); }
void USART1_SendString(const char *str) { while (*str) out_byte((uint8_t)*str++); }

void USART1_SendNum(int32_t num)
{
	char    buf[12];
	int8_t  i = 0;
	uint8_t neg = 0;

	if (num < 0) { neg = 1; num = -num; }
	do { buf[i++] = (char)('0' + (num % 10)); num /= 10; } while (num != 0 && i < 11);
	if (neg) buf[i++] = '-';
	while (i > 0) out_byte((uint8_t)buf[--i]);
}

void USART1_SendMilli(int32_t v)
{
	int32_t f;

	if (v < 0) { out_byte('-'); v = -v; }
	USART1_SendNum(v / 1000);
	out_byte('.');
	f = v % 1000;
	out_byte((uint8_t)('0' + f / 100));
	out_byte((uint8_t)('0' + (f / 10) % 10));
	out_byte((uint8_t)('0' + f % 10));
}

/* 接收环形缓冲：读线程灌，主循环取 */
static volatile uint8_t  rxbuf[8192];
static volatile unsigned rxhead = 0, rxtail = 0;

static void *rx_reader(void *arg)
{
	int c;

	(void)arg;
	while ((c = getchar()) != EOF)
	{
		unsigned next = (rxhead + 1u) % sizeof(rxbuf);
		while (next == rxtail) { /* 满了就等 */ }
		rxbuf[rxhead] = (uint8_t)c;
		rxhead = next;
	}
	return NULL;
}

uint8_t USART1_RxChar(uint8_t *ch)
{
	if (rxtail == rxhead) return 0u;
	*ch = rxbuf[rxtail];
	rxtail = (rxtail + 1u) % sizeof(rxbuf);
	return 1u;
}

/* ---------------- 主循环 ---------------- */
int main(void)
{
	pthread_t th;

	setvbuf(stdout, NULL, _IONBF, 0);

	TUNE_Init();
	fprintf(stderr, "[sim] ready\n");

	pthread_create(&th, NULL, rx_reader, NULL);

	for (;;)
	{
		g_ms_tick += 1u;

		if (s_running)
		{
			/* 编码器里程是单调的（固件里只在开跑时清零），
			   圈数单独按总里程取整算 —— 别让仿真把编码器也绕回去。 */
			int32_t laps;

			s_dist_m += MM_PER_MS / 1000.0;
			laps = (int32_t)(s_dist_m * 1000.0 / LAP_MM);
			if (laps != s_lap_seq) s_lap_seq = laps;

			/* 横向偏移/补偿角给点变化，好让 LAP 有数 */
			s_ct   = (int32_t)((g_ms_tick / 97u) % 21u) - 10;
			s_comp = (int32_t)((g_ms_tick / 53u) % 41u) - 20;
		}

		TUNE_Poll();
		TUNE_Tick();
	}

	(void)th;
	return 0;
}
