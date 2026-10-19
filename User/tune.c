#include "tune.h"
#include "USART1.h"
#include "GRAY.h"
#include "ENCODER.h"
#include "NAV.h"
#include "mycontrol.h"
#include "TIM1_TICK.h"

/* ============================================================
 * 调参协议 + 每圈统计
 *
 * 这个文件只做三件事：
 *   ① 把串口收到的字节拼成行，认 SET / GET / RUN / STOP
 *   ② 把 21 个白名单参数落到控制变量上（车参唯一入口）
 *   ③ 每圈结束打一行 13 字段 LAP
 *
 * 【为什么不直接给 g_kp 这些变量起名当键】键名是调参器定死的英文，
 * 而且有些键在 C 车上没有对应（没有速度环）。这里的表就是"翻译层"：
 * 键 -> 变量，能接上的都接上，接不上的明说（见表里 惰性 注释）。
 * ============================================================ */

/* ---------------- 参数表 ---------------- */

enum
{
	TK_KP_STEER = 0,
	TK_KD_STEER,
	TK_K_GYRO,
	TK_KP_SPEED,
	TK_KI_SPEED,
	TK_KD_SPEED,
	TK_VMAX,
	TK_VMIN,
	TK_A_ACC,
	TK_A_BRAKE,
	TK_A_LAT,
	TK_LOOK_MM,
	TK_CORR_KP,
	TK_CORR_KI,
	TK_CORR_KD,
	TK_CORR_ENC_K,
	TK_STRAIGHT_MIN,
	TK_CORNER_MARGIN,
	TK_STRAIGHT_DEG,
	TK_CORR_ON,
	TK_AUTORUN,
	TK_NUM
};

static const char *const s_name[TK_NUM] =
{
	"kp_steer", "kd_steer", "k_gyro",
	"kp_speed", "ki_speed", "kd_speed",
	"vmax", "vmin", "a_acc", "a_brake", "a_lat",
	"look_mm",
	"corr_kp", "corr_ki", "corr_kd", "corr_enc_k",
	"straight_min", "corner_margin", "straight_deg",
	"corr_on", "autorun"
};

/* 默认值（工程单位 ×1000）—— 和 C 车代码里原来的常数一一对应。
   顺序必须和上面的枚举一致。 */
static const int32_t s_def[TK_NUM] =
{
	70000,        /* kp_steer        -> g_kp        70        */
	0,            /* kd_steer        -> g_kd        0（现在关着） */
	3000,         /* k_gyro          -> g_kgyro    3         */
	3900000,      /* kp_speed        -> g_base_speed 3900     */
	0,            /* ki_speed        —— 惰性：没有速度环      */
	0,            /* kd_speed        —— 惰性：没有速度环      */
	7199000,      /* vmax            -> g_nav_map_vmax 7199（直道满 PWM）*/
	300000,       /* vmin            -> g_trace_vmin 300      */
	18000,        /* a_acc           -> 升速斜率 18/拍        */
	25000,        /* a_brake         -> 降速斜率 25/拍        */
	0,            /* a_lat           —— 惰性：没有横向加速度模型 */
	160000,       /* look_mm         -> g_nav_look_mm 160    */
	800,          /* corr_kp         -> g_nav_corr_k 0.8     */
	0,            /* corr_ki         —— 惰性：修航向是纯比例 */
	0,            /* corr_kd         —— 惰性：修航向是纯比例 */
	1000,         /* corr_enc_k      -> g_enc_k 1.0（里程标定） */
	200000,       /* straight_min    -> g_nav_straight_min 200mm */
	100000,       /* corner_margin   -> g_nav_corner_pct 100%  */
	3000,         /* straight_deg    -> g_nav_straight_deg10 3.0度 */
	1000,         /* corr_on         -> g_nav_corr_on 1      */
	0             /* autorun         -> 上电自动开跑（现在默认关：靠串口发 R）*/
};

static int32_t s_kv[TK_NUM];
static int32_t s_autorun = 1;

/* ---------------- 统计 ---------------- */

/* 128 个桶、每桶 2mm（0~254mm）。P95 靠桶，RMS 靠平方和。
   平方和用 uint32：单点钳到 200mm，200^2*20000 也才 8e8，溢出不了。 */
typedef struct
{
	uint32_t n;        /* 采样拍数 */
	uint32_t sum;      /* Σ v（0.1mm）*/
	uint64_t sq;       /* Σ v*v（0.1mm^2）：用 64 位，200mm 满幅也不会溢出 */
	uint32_t mx;       /* 最大值（0.1mm，不封顶）*/
	uint32_t h[128];   /* 直方图：1mm 一格，0~127mm。用 32 位：一圈 30 秒 x 2.2kHz
	                      同一个桶能落 6 万多拍，uint16 会溢出、P95 直接算飞 */
} TuneStat_t;

static TuneStat_t s_line_st;   /* 光电线偏差 mm */
static TuneStat_t s_ins_st;    /* 惯导横向偏差 mm */

static int32_t  s_comp_sum = 0;   /* 补偿角累加（0.1 度，带符号） */
static uint32_t s_comp_n   = 0;
static uint32_t s_lost_cnt = 0;
static uint8_t  s_lost_prev = 0u;

/* ---------------- 命令行 ---------------- */

static char     s_line[TUNE_LINE_MAX];
static uint8_t  s_len = 0u;
static uint32_t s_line_tick = 0u;
static uint32_t s_last_cmd_tick = 0u;
static uint8_t  s_seen_cmd = 0u;

/* ---------------- 里程打点（TIK / END） ----------------
   陀螺不稳时改用里程当尺子，但得先知道"物理上的相切点/出口"对应多少里程。
   开车过程中每到一个关键位置发一次 TIK 记一笔，跑完发一次 END 全部吐出来。
   同时记两种里程：
     enc = 编码器原始里程（单调，不受锚点影响，用来量真实弧长）
     odo = NAV 内部里程（会被十字 15300 / 直道 20268 锚点重置，葫芦圈判据用的就是它） */
#define TUNE_MARK_MAX   64

typedef struct
{
	uint32_t t;       /* g_ms_tick */
	int32_t  enc;     /* 编码器里程 mm */
	int32_t  odo;     /* NAV 里程 mm */
	int16_t  e;       /* 灰度偏差（×10） */
	uint8_t  mode;
	uint8_t  gray;    /* 8 路，bit7 = 第 0 路 */
} TuneMark_t;

static TuneMark_t s_mark[TUNE_MARK_MAX];
static uint8_t    s_mark_n = 0u;
static uint8_t    s_dump_pos = 0xFFu;   /* 0xFF = 没在吐数据 */
static uint8_t    s_silent = 0u;        /* 1 = 关掉全部遥测（END 之后：只看打点） */

/* ---------------- 跑圈状态 ---------------- */

static uint8_t  s_run_laps = 0u;   /* 0 = 没在跑；否则这一轮要跑几圈 */
static uint8_t  s_run_no   = 0u;   /* 已经跑完几圈 */
static int32_t  s_lap_no   = 0u;   /* 当前圈号（从 1 开始） */
static uint32_t s_lap_t0   = 0u;
static int32_t  s_lap_odo0 = 0;    /* 本圈起点里程(mm) */
static int32_t  s_last_seq = 0;

/* ============================================================
 * 小工具
 * ============================================================ */

/* 解析一个数，返回 ×1000 定点。支持 70 / -1.5 / 68.375 / 4e-06 / 1.2E+3 */
static int32_t Tune_ParseMilli(const char **pp, uint8_t *ok)
{
	const char *p = *pp;
	int32_t sign = 1;
	int32_t mant = 0;
	int32_t frac = 0;
	int32_t exp  = 0;
	uint8_t dot  = 0u;
	uint8_t nd   = 0u;
	int32_t e, v;

	*ok = 0u;

	while (*p == ' ') p++;
	if (*p == '-')      { sign = -1; p++; }
	else if (*p == '+') { p++; }

	while (*p)
	{
		if (*p >= '0' && *p <= '9')
		{
			if (nd < 9u)
			{
				mant = mant * 10 + (int32_t)(*p - '0');
				nd++;
			}
			else if (!dot)
			{
				mant = 999999999;          /* 整数部分超长：饱和 */
			}
			if (dot) frac++;
			p++;
		}
		else if (*p == '.' && !dot)
		{
			dot = 1u;
			p++;
		}
		else
		{
			break;
		}
	}

	if (nd == 0u) return 0;               /* 一个数字都没有 */

	if (*p == 'e' || *p == 'E')
	{
		int32_t es = 1;

		p++;
		if (*p == '-')      { es = -1; p++; }
		else if (*p == '+') { p++; }

		while (*p >= '0' && *p <= '9')
		{
			if (exp < 100) exp = exp * 10 + (int32_t)(*p - '0');
			p++;
		}
		exp *= es;
	}

	/* 总值 = mant × 10^(exp - frac) ；再换成 ×1000 -> 指数 +3 */
	e = exp - frac + 3;

	if (e >= 0)
	{
		while (e-- > 0)
		{
			if (mant > 100000000) { mant = 1000000000; break; }
			mant *= 10;
		}
		v = mant;
	}
	else
	{
		e = -e;
		while (e-- > 0)
		{
			if (mant == 0) break;
			mant = (mant + 5) / 10;       /* 四舍五入 */
		}
		v = mant;
	}

	*ok = 1u;
	*pp = p;
	return (sign < 0) ? -v : v;
}

static uint8_t Tune_Pre(const char *s, const char *pre)
{
	while (*pre)
	{
		char a = *s++;

		if (a >= 'a' && a <= 'z') a = (char)(a - 32);
		if (a != *pre) return 0u;
		pre++;
	}
	return 1u;
}

/* 第一个词是不是已知命令（很多串口工具默认不补换行，靠它判断"这行已经完整"）*/
static uint8_t Tune_HeadKnown(const char *s, uint8_t n)
{
	static const char *const cmd[17] =
	{
		"R", "STOP", "TIK", "END", "CLR", "RES", "GLD", "PING",
		"MOVE", "WID", "SEL", "IN", "WIN", "SLW", "SET", "GET", "RUN"
	};
	uint8_t k, j, m = 0u;

	while (m < n && s[m] != ' ' && s[m] != '\t') m++;   /* 第一个词的长度 */

	for (k = 0u; k < 17u; k++)
	{
		const char *c = cmd[k];

		for (j = 0u; j < m; j++)
		{
			char a = s[j];

			if (a >= 'a' && a <= 'z') a = (char)(a - 32);
			if (a != c[j]) break;
		}
		if (j == m && c[m] == 0) return 1u;
	}
	return 0u;
}

static int8_t Tune_Find(const char *key, uint8_t n)
{
	uint8_t i, j;

	for (i = 0u; i < (uint8_t)TK_NUM; i++)
	{
		const char *nm = s_name[i];

		for (j = 0u; j < n; j++)
		{
			char c = key[j];

			if (c >= 'A' && c <= 'Z') c = (char)(c + 32);
			if (c != nm[j]) break;
		}
		if (j == n && nm[n] == 0) return (int8_t)i;
	}
	return -1;
}

/* ---------------- 统计工具 ---------------- */

static void Tune_StatReset(TuneStat_t *s)
{
	uint8_t i;

	s->n = 0u; s->sum = 0u; s->sq = 0u; s->mx = 0u;
	for (i = 0u; i < 128u; i++) s->h[i] = 0u;
}

static void Tune_StatAdd(TuneStat_t *s, int32_t v10)
{
	uint32_t b;

	if (v10 < 0) v10 = -v10;
	if ((uint32_t)v10 > s->mx) s->mx = (uint32_t)v10;

	if (v10 > 2000) v10 = 2000;           /* 200mm 封顶（直方图只到 127mm，RMS 也别被极值带飞）*/

	s->sum += (uint32_t)v10;
	s->sq  += (uint64_t)((uint32_t)v10 * (uint32_t)v10);

	b = (uint32_t)v10 / 10u;              /* 1mm 一格 */
	if (b > 127u) b = 127u;
	s->h[b]++;
	s->n++;
}

static int32_t Tune_Isqrt(uint32_t v)
{
	uint32_t x, y;

	if (v == 0u) return 0;

	x = v;
	y = (x + 1u) / 2u;
	while (y < x)
	{
		x = y;
		y = (x + v / x) / 2u;
	}
	return (int32_t)x;
}

/* RMS，单位 0.1mm（sq/n 最大 2000*2000 = 4e6，还是 32 位量级） */
static int32_t Tune_StatRms(const TuneStat_t *s)
{
	if (s->n == 0u) return 0;
	return Tune_Isqrt((uint32_t)(s->sq / s->n));
}

/* P95，单位 0.1mm。
   1mm 的桶 + 桶内按均匀分布线性插值 —— 这样 4mm 和 4.4mm 分得开，
   调参器按 5% 步长改参数时才看得见差别（都用整数 mm 的话永远"没变化"）。 */
static int32_t Tune_StatP95(const TuneStat_t *s)
{
	uint32_t i, cum = 0u, target, before, inbin, frac;

	if (s->n == 0u) return 0;

	target = (s->n * 95u + 99u) / 100u;
	for (i = 0u; i < 128u; i++)
	{
		if (s->h[i] == 0u) continue;

		before = cum;
		cum   += s->h[i];
		if (cum >= target)
		{
			inbin = cum - before;
			frac  = ((target - before) * 10u) / inbin;   /* 0~10 = 桶内 0.1mm */

			/* 插值只是个估计：P95 不可能超过实际最大值，超了就压回 MAX */
			if ((uint32_t)(i * 10u + frac) > s->mx) return (int32_t)s->mx;
			return (int32_t)(i * 10u + frac);
		}
	}
	return (int32_t)s->mx;
}

/* ============================================================
 * 参数落地：键 -> 真正的控制变量
 * ============================================================ */
static void Tune_Apply(void)
{
	g_kp            = s_kv[TK_KP_STEER];        /* 千分之一 */
	g_kd            = s_kv[TK_KD_STEER];
	g_kgyro         = s_kv[TK_K_GYRO];

	/* kslow（弯道降速）不在 21 键白名单里 —— 契约里没有它，
	   但控制里还要用，所以这里固定成原值 16（×1000 定点）。 */
	g_kslow         = 16000;

	g_base_speed    = s_kv[TK_KP_SPEED] / 1000;
	g_ramp_up       = s_kv[TK_A_ACC] / 1000;
	g_ramp_dn       = s_kv[TK_A_BRAKE] / 1000;

	/* 【vmax】C 车跑的是"背图变速"（NAV_SpeedBase），直道速度 = 这张表的上限
	   g_nav_map_vmax。惯导跑用的 g_nav_vmax 只是 NAV 模式的直道速度，
	   调参跑的是光电模式，所以 vmax 落到 map_vmax 才真的有作用。 */
	g_nav_map_vmax  = s_kv[TK_VMAX] / 1000;

	/* 【vmin】光电循迹的弯道最低速度（原来写死 300）*/
	g_trace_vmin    = s_kv[TK_VMIN] / 1000;
	g_nav_look_mm   = s_kv[TK_LOOK_MM] / 1000;
	g_nav_corr_k    = s_kv[TK_CORR_KP];         /* 千分之一 */
	g_nav_corr_on   = s_kv[TK_CORR_ON] / 1000;
	g_nav_corner_pct = s_kv[TK_CORNER_MARGIN] / 1000;
	g_nav_straight_min   = s_kv[TK_STRAIGHT_MIN] / 1000;
	g_nav_straight_deg10 = s_kv[TK_STRAIGHT_DEG] / 100;

	g_enc_k         = s_kv[TK_CORR_ENC_K];      /* 千分之一 */
	s_autorun       = s_kv[TK_AUTORUN] / 1000;
}

/* ============================================================
 * LAP 行
 * ============================================================ */
static void Tune_LapLine(uint8_t fin, const char *why)
{
	if (s_silent) return;       /* END 之后连 LAP 也不打 */

	int32_t  ms = (int32_t)(g_ms_tick - s_lap_t0);
	int32_t  mm = (int32_t)(ENCODER_GetDist() * 1000.0f) - s_lap_odo0;
	int32_t  comp = 0;

	if (s_comp_n > 0u)
	{
		/* 0.1 度 -> 千分之一度：sum*100/n */
		comp = (int32_t)((s_comp_sum * 100) / (int32_t)s_comp_n);
	}

	USART1_SendString("LAP,");
	USART1_SendNum(s_lap_no);
	USART1_SendByte(',');
	USART1_SendNum(fin ? 1 : 0);
	USART1_SendByte(',');
	USART1_SendNum(ms);
	USART1_SendByte(',');
	USART1_SendNum(mm);
	USART1_SendByte(',');

	/* 统计量都是 0.1mm，乘 100 变成"千分之一毫米"再按 mm 打成 3 位小数 */
	USART1_SendMilli(Tune_StatRms(&s_line_st) * 100);
	USART1_SendByte(',');
	USART1_SendMilli(Tune_StatP95(&s_line_st) * 100);
	USART1_SendByte(',');
	USART1_SendMilli((int32_t)s_line_st.mx * 100);
	USART1_SendByte(',');

	USART1_SendMilli(Tune_StatRms(&s_ins_st) * 100);
	USART1_SendByte(',');
	USART1_SendMilli(Tune_StatP95(&s_ins_st) * 100);
	USART1_SendByte(',');
	USART1_SendMilli(comp);
	USART1_SendByte(',');
	USART1_SendNum((int32_t)s_lost_cnt);
	USART1_SendByte(',');
	USART1_SendString(why);
	USART1_SendString("\r\n");
}

static void Tune_LapReset(void)
{
	Tune_StatReset(&s_line_st);
	Tune_StatReset(&s_ins_st);
	s_comp_sum  = 0;
	s_comp_n    = 0u;
	s_lost_cnt  = 0u;
	s_lost_prev = 0u;
	s_lap_t0    = g_ms_tick;
	s_lap_odo0  = (int32_t)(ENCODER_GetDist() * 1000.0f);
}

static void Tune_LapDone(uint8_t fin, const char *why)
{
	s_lap_no++;
	Tune_LapLine(fin, why);

	if (!fin)
	{
		/* 这一圈没跑完（超时/停车）：收工，别让车接着跑 */
		s_run_laps = 0u;
		App_Stop();
		return;
	}

	s_run_no++;
	Tune_LapReset();

	if (s_run_no >= s_run_laps)
	{
		s_run_laps = 0u;
		App_Stop();
	}
}

/* ============================================================
 * 命令
 * ============================================================ */
static void Tune_CmdSet(const char *p)
{
	char    key[16];
	uint8_t n = 0u;
	uint8_t ok = 0u;
	int8_t  idx;
	int32_t mi;

	while (*p == ' ') p++;
	while (*p && *p != ' ' && *p != '\t' && n < 15u) key[n++] = *p++;
	key[n] = 0;

	mi  = Tune_ParseMilli(&p, &ok);
	idx = Tune_Find(key, n);

	if (idx < 0 || !ok)
	{
		USART1_SendString("ERR\r\n");
		return;
	}

	s_kv[(uint8_t)idx] = mi;
	Tune_Apply();

	/* 关掉自动开跑 = 把车交给上位机，顺手停一下：
	   否则上电倒计时可能已经把车放出去了，调参器一接上先看到一辆在跑的车。 */
	if (idx == TK_AUTORUN && mi == 0 && s_run_laps == 0u)
	{
		App_Stop();
	}

	USART1_SendString("OK\r\n");
}

static void Tune_CmdGet(const char *p)
{
	char    key[16];
	uint8_t n = 0u;
	int8_t  idx;

	while (*p == ' ') p++;
	while (*p && *p != ' ' && *p != '\t' && n < 15u) key[n++] = *p++;
	key[n] = 0;

	idx = Tune_Find(key, n);
	if (idx < 0)
	{
		USART1_SendString("ERR\r\n");
		return;
	}

	USART1_SendString("OK ");
	USART1_SendMilli(s_kv[(uint8_t)idx]);
	USART1_SendString("\r\n");
}

static void Tune_CmdTik(void)
{
	TuneMark_t *m;
	uint8_t g = 0u, i;

	if (s_mark_n >= (uint8_t)TUNE_MARK_MAX)
	{
		USART1_SendString("ERR FULL\r\n");
		return;
	}

	m = &s_mark[s_mark_n];
	m->t    = g_ms_tick;
	m->enc  = (int32_t)(ENCODER_GetDist() * 1000.0f);
	m->odo  = NAV_OdoMm();
	m->e    = GRAY_GetError();
	m->mode = g_mode;

	for (i = 0u; i < 8u; i++)
	{
		g = (uint8_t)((g << 1) | (GRAY_GetChannel(i) ? 1u : 0u));
	}
	m->gray = g;

	s_mark_n++;

	USART1_SendString("OK ");
	USART1_SendNum((int32_t)s_mark_n);
	USART1_SendString("\r\n");
}

/* END：① 立刻锁车  ② 关掉所有遥测  ③ 只把打点吐出来
   —— 车跑完一圈人在旁边按一下，串口上就只剩里程数据，不会跟遥测混在一起。
   回复先给一行"有多少笔"，数据交给 Tune_DumpPump 慢慢吐（一次全塞进
   256 字节的发送环会被丢）。 */
static void Tune_CmdEnd(void)
{
	/* 锁车：清掉跑圈状态并进 MODE_STOP，Control_Tick 每拍都会把 PWM 写 0 */
	s_run_laps = 0u;
	App_Stop();

	/* 静默：数据行 / 状态行 / LAP 全部停发（命令回应不受影响） */
	s_silent = 1u;

	s_dump_pos = 0u;

	USART1_SendString("OK ");
	USART1_SendNum((int32_t)s_mark_n);
	USART1_SendString("\r\n");
}

/* 等发送缓冲有地方了再吐一行，主循环每圈调一次 */
static void Tune_DumpPump(void)
{
	const TuneMark_t *m;
	int8_t b;

	if (s_dump_pos == 0xFFu) return;
	if (s_dump_pos >= s_mark_n) { s_dump_pos = 0xFFu; return; }
	if (USART1_TxPending() > 160u) return;

	m = &s_mark[s_dump_pos];

	USART1_SendString("TIK,");
	USART1_SendNum((int32_t)s_dump_pos + 1);
	USART1_SendByte(',');
	USART1_SendNum((int32_t)m->t);
	USART1_SendByte(',');
	USART1_SendNum(m->enc);
	USART1_SendByte(',');
	USART1_SendNum(m->odo);
	USART1_SendByte(',');
	USART1_SendNum((int32_t)m->mode);
	USART1_SendByte(',');

	for (b = 7; b >= 0; b--)
	{
		USART1_SendByte((m->gray & (uint8_t)(1u << b)) ? '1' : '0');
	}

	USART1_SendByte(',');
	USART1_SendNum((int32_t)m->e);
	USART1_SendString("\r\n");

	s_dump_pos++;
}

/* 回一行当前窗口：OK <序号> <绝对中心mm> <全宽mm> <动作位> */
static void Tune_SendWin(void)
{
	uint8_t i = NAV_WinSel();

	USART1_SendString("OK ");
	USART1_SendNum((int32_t)i + 1);
	USART1_SendByte(' ');
	USART1_SendNum(NAV_WinAbs(i));
	USART1_SendByte(' ');
	USART1_SendNum(NAV_WinW(i));
	USART1_SendByte(' ');
	USART1_SendNum(NAV_WinAct(i));
	USART1_SendByte(' ');
	USART1_SendNum(NAV_GourdSlowPct());
	USART1_SendString("\r\n");
}

/* SLW [pct]：窗口期速度百分比（不带参数 = 只读） */
static void Tune_CmdSlw(const char *p)
{
	uint8_t ok = 0u;
	int32_t v = Tune_ParseMilli(&p, &ok) / 1000;

	if (ok) NAV_GourdSetSlowPct(v);

	USART1_SendString("OK ");
	USART1_SendNum(NAV_GourdSlowPct());
	USART1_SendString("\r\n");
}

/* MOVE <mm>：当前窗口整体平移。-30 = 前移 30mm（更早触发），+30 = 后移 */
static void Tune_CmdMove(const char *p)
{
	uint8_t ok = 0u;
	int32_t v = Tune_ParseMilli(&p, &ok) / 1000;

	if (!ok) { USART1_SendString("ERR\r\n"); return; }

	NAV_WinMove(v);
	Tune_SendWin();
}

/* WID <mm>：当前窗口全宽 */
static void Tune_CmdWid(const char *p)
{
	uint8_t ok = 0u;
	int32_t v = Tune_ParseMilli(&p, &ok) / 1000;

	if (!ok) { USART1_SendString("ERR\r\n"); return; }

	NAV_WinSetWidth(v);
	Tune_SendWin();
}

/* SEL <n>：选第 n 个窗口（1 起）作为"当前窗口" */
static void Tune_CmdSel(const char *p)
{
	uint8_t ok = 0u;
	int32_t v = Tune_ParseMilli(&p, &ok) / 1000;

	if (!ok) { USART1_SendString("ERR\r\n"); return; }

	NAV_WinSelect((uint8_t)((v > 0) ? (v - 1) : 0));
	Tune_SendWin();
}

/* IN [mm]：不带参数 = 读进圈里程；带参数 = 改（窗口是相对的，会一起走）*/
static void Tune_CmdIn(const char *p)
{
	uint8_t ok = 0u;
	int32_t v = Tune_ParseMilli(&p, &ok) / 1000;

	if (ok) NAV_GourdSetEntry(v);

	USART1_SendString("OK ");
	USART1_SendNum(NAV_GourdEntry());
	USART1_SendString("\r\n");
}

/* WIN：把窗口表全打出来 */
static void Tune_CmdWinDump(void)
{
	uint8_t i, n = NAV_WinCount();

	for (i = 0u; i < n; i++)
	{
		USART1_SendString("WIN,");
		USART1_SendNum((int32_t)i + 1);
		USART1_SendByte(',');
		USART1_SendNum(NAV_WinAbs(i));
		USART1_SendByte(',');
		USART1_SendNum(NAV_WinW(i));
		USART1_SendByte(',');
		USART1_SendNum(NAV_WinAct(i));
		USART1_SendByte(',');
		USART1_SendNum(NAV_GourdSlowPct());
		USART1_SendString("\r\n");
	}

	USART1_SendString("OK ");
	USART1_SendNum((int32_t)n);
	USART1_SendString("\r\n");
}

/* RES：恢复遥测（END 之后想继续看数据就发这个） */
static void Tune_CmdRes(void)
{
	s_silent = 0u;
	USART1_SendString("OK\r\n");
}

/* GLD [0|1]：不带参数 = 只读当前状态 */
static void Tune_CmdGld(const char *p)
{
	uint8_t ok = 0u;
	int32_t v;

	while (*p == ' ') p++;
	v = Tune_ParseMilli(&p, &ok) / 1000;

	if (ok) g_gourd_en = (v != 0) ? 1 : 0;

	USART1_SendString("OK ");
	USART1_SendNum((int32_t)g_gourd_en);
	USART1_SendString("\r\n");
}

static void Tune_CmdRun(const char *p)
{
	uint8_t ok = 0u;
	int32_t n  = Tune_ParseMilli(&p, &ok) / 1000;

	if (!ok)
	{
		USART1_SendString("ERR\r\n");
		return;
	}
	if (n < 1)  n = 1;
	if (n > 20) n = 20;

	/* 先把 ACK 塞进发送缓冲：下面的开跑要标定陀螺（约 0.6~1.8 秒），
	   标定期间主循环是卡住的，不能等它跑完再回话。 */
	USART1_SendString("ACK\r\n");

	/* RUN 期间必须关掉自动开跑，否则 RUN 和上电倒计时会各跑一次 */
	s_kv[TK_AUTORUN] = 0;
	Tune_Apply();

	s_run_laps = (uint8_t)n;
	s_run_no   = 0u;
	s_lap_no   = 0;

	App_StartRun();          /* 会清编码器/惯导里程，所以统计基准放在它后面 */

	s_last_seq = NAV_LapSeq();   /* 必须在开跑之后取：开跑会把圈序号清零 */
	Tune_LapReset();
}

static void Tune_DoLine(char *line)
{
	char *p = line;

	while (*p == ' ') p++;
	if (*p == 0) return;

	s_last_cmd_tick = g_ms_tick;
	s_seen_cmd = 1u;

	if (Tune_Pre(p, "SET ")) { Tune_CmdSet(p + 4); return; }
	if (Tune_Pre(p, "GET ")) { Tune_CmdGet(p + 4); return; }
	if (Tune_Pre(p, "RUN ")) { Tune_CmdRun(p + 4); return; }

	/* 单字符 R = 开跑（和蓝牙 0x06 / RUN 走同一条路）。
	   串口现在就是遥控器：连上、发个 R、车就跑起来。
	   【必须限定"整行就是一个 R"】否则 RES 也会被当成发车（踩过）。 */
	if (Tune_Pre(p, "R") && p[1] == 0)
	{
		App_StartRun();
		USART1_SendString("OK\r\n");
		return;
	}

	if (Tune_Pre(p, "TIK")) { Tune_CmdTik(); return; }
	if (Tune_Pre(p, "END")) { Tune_CmdEnd(); return; }

	if (Tune_Pre(p, "CLR"))
	{
		s_mark_n = 0u;
		s_dump_pos = 0xFFu;
		USART1_SendString("OK\r\n");
		return;
	}

	if (Tune_Pre(p, "MOVE")) { Tune_CmdMove(p + 4); return; }
	if (Tune_Pre(p, "WID"))  { Tune_CmdWid(p + 3); return; }
	if (Tune_Pre(p, "SEL"))  { Tune_CmdSel(p + 3); return; }
	if (Tune_Pre(p, "IN"))   { Tune_CmdIn(p + 2); return; }
	if (Tune_Pre(p, "WIN"))  { Tune_CmdWinDump(); return; }
	if (Tune_Pre(p, "SLW"))  { Tune_CmdSlw(p + 3); return; }

	if (Tune_Pre(p, "RES")) { Tune_CmdRes(); return; }
	if (Tune_Pre(p, "GLD")) { Tune_CmdGld(p + 3); return; }

	if (Tune_Pre(p, "STOP"))
	{
		s_run_laps = 0u;
		App_Stop();
		USART1_SendString("OK\r\n");
		return;
	}
	if (Tune_Pre(p, "PING"))
	{
		USART1_SendString("OK\r\n");
		return;
	}

	/* 其它一律不理：调参器只发上面三种，噪声不该换回任何字节 */
}

/* ============================================================
 * 对外
 * ============================================================ */
void TUNE_Init(void)
{
	uint8_t i;

	for (i = 0u; i < (uint8_t)TK_NUM; i++) s_kv[i] = s_def[i];

	Tune_Apply();

	/* 默认静默：串口上只有"窗口触发"和命令回应，不再有周期遥测 */
	s_silent = 1u;

	s_len = 0u;
	s_line_tick = g_ms_tick;
	s_last_cmd_tick = g_ms_tick;
	s_seen_cmd = 0u;

	s_run_laps = 0u;
	s_run_no = 0u;
	s_lap_no = 0u;
	Tune_LapReset();
}

void TUNE_Poll(void)
{
	uint8_t ch;

	while (USART1_RxChar(&ch))
	{
		if (ch == '\r') continue;

		if (ch == '\n')
		{
			if (s_len > 0u)
			{
				s_line[s_len] = 0;
				Tune_DoLine(s_line);
			}
			s_len = 0u;
			continue;
		}

		/* 不可打印字节 = 蓝牙遥控那套单字节指令或线上噪声：
		   丢掉半行重新同步，别把 0x06 之类拼进命令里 */
		if (ch < 0x20u || ch > 0x7Eu)
		{
			s_len = 0u;
			continue;
		}

		if (s_len >= (uint8_t)(TUNE_LINE_MAX - 1u)) s_len = 0u;

		s_line[s_len++] = (char)ch;
		s_line_tick = g_ms_tick;
	}

	/* 打点数据的续吐（非阻塞，等发送缓冲有地方） */
	Tune_DumpPump();

	/* 【免换行】很多串口上位机（包括野火这个）默认不在命令后面补换行符：
	   内容已经是完整的短命令（R/STOP/TIK/END/CLR/RES/GLD/PING）、
	   并且 100ms 没有后续字节，就当成一条命令执行。
	   SET/GET/RUN 这种带参数的不在此列（必须有换行，否则无法判断参数结束）。 */
	if (s_len >= 1u && (uint32_t)(g_ms_tick - s_line_tick) > 100u &&
	    Tune_HeadKnown(s_line, s_len))
	{
		s_line[s_len] = 0;
		Tune_DoLine(s_line);
		s_len = 0u;
	}
	/* 半行卡住 >1 秒：波特率不对 / 线松了，丢掉重新同步 */
	else if (s_len > 0u && (uint32_t)(g_ms_tick - s_line_tick) > 1000u)
	{
		s_len = 0u;
	}
}

void TUNE_Tick(void)
{
	int32_t v;

	if (s_run_laps == 0u) return;

	/* 线偏差：GRAY_GetError() 是放大 10 倍的光电偏差（±50），
	   乘 g_nav_mm_per_e 换成毫米。1 点 = 1mm 时满偏 50mm。 */
	v = GRAY_GetError();
	if (v < 0) v = -v;
	v = v * (g_nav_mm_per_e > 0 ? g_nav_mm_per_e : 1) * 10;   /* -> 0.1mm */
	Tune_StatAdd(&s_line_st, v);

	/* 惯导横向：车相对路表中心线的偏移(mm) -> 0.1mm */
	v = NAV_GetCrossTrack();
	if (v < 0) v = -v;
	Tune_StatAdd(&s_ins_st, v * 10);

	/* 补偿角（0.1 度，带符号）：均值是有意义的，别取绝对值 */
	s_comp_sum += NAV_YawComp10();
	s_comp_n++;

	/* 脱线次数：由"在线"跳成"丢线"算一次 */
	{
		uint8_t lost = GRAY_IsLost() ? 1u : 0u;

		if (lost && !s_lost_prev) s_lost_cnt++;
		s_lost_prev = lost;
	}

	/* 圈到了没：NAV 的里程跑满一圈会绕回 0，并让 NAV_LapSeq() 加一。
	   这样第二圈起锚点/葫芦圈窗口都还在原来的里程上（多圈跑得通）。 */
	{
		int32_t seq = NAV_LapSeq();

		if (seq != s_last_seq)
		{
			s_last_seq = seq;
			Tune_LapDone(1u, "OK");
		}
	}

	/* 圈超时：被保命刹停 / 掉下赛道 / 卡住 —— 停车，交一圈"没跑完" */
	if (s_run_laps != 0u && (uint32_t)(g_ms_tick - s_lap_t0) > TUNE_LAP_TIMEOUT_MS)
	{
		Tune_LapDone(0u, "TIMEOUT");
	}
}

uint8_t TUNE_Quiet(void)
{
	if (s_run_laps != 0u) return 1u;
	if (s_seen_cmd && (uint32_t)(g_ms_tick - s_last_cmd_tick) < TUNE_QUIET_MS) return 1u;
	return 0u;
}

uint8_t TUNE_RunActive(void) { return (s_run_laps != 0u) ? 1u : 0u; }

uint8_t TUNE_Silent(void) { return s_silent; }

int32_t TUNE_Autorun(void) { return s_autorun; }
