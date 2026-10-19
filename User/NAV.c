#include "NAV.h"
#include "mycontrol.h"
#include "ENCODER.h"
#include "IMU.h"
#include "GRAY.h"
#include "MOTOR.h"
#include "TIM1_TICK.h"
#include "stm32f10x_flash.h"
#include "USART1.h"

/* ============================================================
 * 惯性导航 —— 建图 / 惯导跑
 *
 * 这个文件的思路只有一句话：
 *   把【已经走过的路】记下来，下一次按里程照着这张表开。
 * 所以相切点、十字、直角弯全都不需要判断 —— 表里本来就是对的路径。
 * ============================================================ */

/* ---- 可调参数 ---- */
/* 【葫芦圈总开关】0 = 不用陀螺角度接管（螺旋段交给光电）。
   默认 0：陀螺零偏还没标稳，先用里程/光电跑。串口 GLD 1 可临时打开对比。 */
int g_gourd_en = 0;

volatile int g_nav_vmax     = 3200;   /* 惯导跑：直道速度（PWM）。
                                          第一次试跑先用 2200（慢）；惯导跑没有光电
                                          位置反馈，表没对上时快了救不回来。跑通再加。 */
volatile int g_nav_vmin     = 900;    /* 惯导跑：最急弯的速度 */
volatile int g_nav_look_mm  = 160;    /* 前瞻距离(mm)：大了稳，小了跟得紧 */
volatile int32_t g_nav_kgyro = 2000;      /* 陀螺阻尼增益 */
/* 【2026-10-18】修航向从"除以 corr_div"改成"乘 corr_k"（×1000 定点）：
   比例项改用乘法，调参器按 5% 步长改才不会被整数除法吃光。
   默认 800 和原来的 16/20 完全等价。 */
volatile int g_nav_corr_k   = 800;    /* 修航向比例增益 ×1000 */
volatile int g_nav_corr_on  = 1;      /* 1 = 允许直道光电修航向（调参键 corr_on）*/
volatile int g_nav_straight_min   = 200;  /* 直道最短长度(mm)：修航向只在这种段生效 */
volatile int g_nav_straight_deg10 = 30;   /* 直道角度门限 0.1 度（30 = 3.0 度）*/

volatile int g_nav_corr_div = 20;     /* 直道光电修航向强度：越大越慢
                                         （20 -> 偏差 10mm 给 0.5 度）*/
volatile int g_nav_mm_per_e = 1;      /* 光电 E: 1 点约等于多少 mm（现场标定）*/
volatile int g_nav_start_off_mm = 0;  /* 发车点相对内置表 0 点的偏置(mm)，见 NAV.h */
volatile int g_nav_map_vmax  = 7199;  /* 背图变速：上限(直道速度 PWM)。 */

volatile int g_nav_corner_pct = 100;  /* 圈外弯道加深比例%: 100=完全按表(弯里约2800，等于老光电版本)；
                                        越大弯道越慢。直道(表值255)不受影响。 */

/* ---- 内部控制常量 ---- */
#define NAV_INT_MS       5      /* NAV_Update 的调用周期(ms) */
#define NAV_GYRO_DB      2      /* 陀螺死区(度/秒)：小于它当作 0，治静止漂移 */
#define NAV_ERR_FILT_N   8      /* 转向误差低通，和光电模式同一套 */
#define NAV_LOOK_MIN     2      /* 前瞻至少几个点 */
#define NAV_CORR_MAX16   288    /* 补偿角上限 18 度（x16 定点）*/
#define NAV_JUMP_LIMIT   2000   /* 单拍里程跳变限幅(0.1mm)：超过当异常丢掉 */

/* ---- 路表（RAM：建图时写，跑图时读；上电从 Flash 拷回来）---- */
static int16_t  s_map_x[NAV_N_MAX];
static int16_t  s_map_y[NAV_N_MAX];
static uint8_t  s_map_kind[NAV_N_MAX];   /* 0=普通 1=直道两端 2=直道中段(可修航向) */
static uint16_t s_map_n = 0;

/* ---- 航位推算状态 ---- */
static int32_t s_x10 = 0;           /* 位置，0.1mm */
static int32_t s_y10 = 0;
static int32_t s_yaw_acc = 0;       /* 航向积分累加器，单位 0.001 度。
                                       为什么不用 0.1 度直接加：慢速转弯时
                                       w*5/100 会被整数除法截断成 0，转一整圈
                                       也积不上，实测踩过。 */
static int32_t s_upd_cnt = 0;       /* NAV_Update 被调用次数（上电起算，诊断用）*/
static uint32_t s_yaw_tick = 0;     /* 上次航向积分的时间戳(ms) */
static uint32_t s_lost_tick = 0;    /* 开始丢线的时刻(g_ms_tick)；0=没丢线 */
static int32_t  s_ct_mm    = 0;     /* 车相对路表的横向偏移 mm（左正），遥测用 */
static uint8_t  s_lost_brake = 0;   /* 1 = 保命刹停触发过（诊断：是不是它把车按住的）*/
static int32_t  s_pwm_avg    = 0;   /* 这一拍下发的平均速度（诊断：到底给没给速度）*/
static int32_t  s_spd_lim    = 0;   /* 当前点速度上限(0~255)，遥测用 */
static uint8_t  s_in_gourd   = 0;   /* 1 = 当前在葫芦圈里（照表开）*/
static int32_t  s_gturn      = 0;   /* 葫芦圈内累计【绝对】转角(0.001度)：切点/出口都按它触发，
                                      用陀螺而不是里程，路面不平/打滑都不影响 */
static const int32_t s_flip_deg[3] = { 180000, 360000, 540000 };  /* 半圆 = 180 度 */
/* ---- 蛇形识别（不依赖里程的进圈判定）----
   蛇形：绝对转角一路涨、带符号几乎不涨（左右半圈抵消）-> 比值小
   普通弯/大圆：一直朝一个方向转 -> 带符号≈绝对 -> 比值≈1（不会误触发）
   蛇形弯(R200)那段也是蛇形，用"里程粗门"排除（只在里程 > 18m 后启用）。 */
#define NAV_USE_SNAKE_ENTRY 0        /* 1 = 额外用"蛇形特征"判定进圈（当前关，按里程）*/
#define NAV_SNAKE_ARM_MM   18000     /* 粗里程门(mm)：这之后才认蛇形 */
#define NAV_SNAKE_ABS_DEG  450       /* 绝对转角超过这么多度 */
static int32_t  s_trn_abs = 0;      /* 自上次直道以来累计【绝对】转角 0.001度 */
static int32_t  s_trn_sgn = 0;      /* 自上次直道以来累计【带符号】转角 0.001度 */
static uint16_t s_str_cnt = 0;      /* 连续直道拍数（5ms 一拍）*/

#define NAV_EXIT_DEG_TURN   670000   /* 出口：4个半圆=720度。现场定的 670 */
#define NAV_GOURD_VMAX  4000        /* 葫芦圈内的速度上限(PWM)：那段要强制转向，
                                       压住速度才拐得过来（表里给的是 ~3300）*/

/* 葫芦圈 3 个相切点的里程：入口 + k × 半圆弧长(πR)。
   入口由实测给出；R=0.30125m 是用户量的葫芦圈半径。
   转向序列 左/右/左 是队友在同一赛道实测的。到点强制打 200ms 方向，
   把车从"继续绕本圆"扳到"换下一个圆"。 */
#define NAV_FLIP_MS     100           /* 强制转向持续(ms)：只轻推一下，200ms 会转过头(实测能转 40~80 度) */
#define NAV_VERIFY_MS   320           /* 扳完以后看光电多久(ms)：线有没有跟回来 */
#define NAV_FIX_MS      120           /* 判定扳错时，反向纠正多久(ms) */
#define NAV_TAN_SPACING_MM 946        /* 半圆弧长 = pi * R，R=0.30125m -> 946mm */
#define NAV_GOURD_S0_MM   (NAV_GOURD_ENTRY_MM - 200)   /* 限速区间起点 */
/* 限速区间终点：4 个半圆 = 4 x 946 = 3784mm，再留点余量。
   之前只到 +3000mm，结果车还在最后一个圆里限速就被放开、全速冲出去。 */
#define NAV_GOURD_S1_MM   (NAV_GOURD_ENTRY_MM + 5500)

static const int8_t  s_flip_dir[3] = {  1,    -1,     1   };   /* +1 = 左转 */
static uint8_t  s_flip_done  = 0;
static uint32_t s_flip_until = 0;
static int8_t   s_flip_now   = 0;
static uint32_t s_verify_until = 0;  /* 验证窗口结束时刻 */
static int8_t   s_fix_now    = 0;    /* 反向纠正的方向 */
static uint8_t  s_flip_state = 0;    /* 0空闲 1正在强制 2验证中 3正在反向纠正 */
static uint16_t s_verify_bad = 0;    /* 验证窗口里"没跟回来"的拍数 */

/* ============================================================
 * 【2026-10-19 新】葫芦圈【里程窗口】状态机（纯里程，不碰陀螺）
 *
 * 思路：里程一过"进圈里程"就进状态机；在若干"里程窗口"里改转向策略。
 *   现在只定义第一个窗口：第一个相切点【屏蔽右转】——
 *   窗口内 e>0 一律置 0，等于"这个点不许往右拐"。
 *   （相切点那两条对称分支，灰度质心选不出来，与其让它随机选，不如禁掉一边。）
 *
 * 窗口表运行时用串口改，方便一点一点试，不用反复重烧：
 *   MOVE -30   当前窗口整体前移 30mm（+30 = 后移）
 *   WID 300    当前窗口全宽改成 300mm
 *   SEL 2      选第 2 个窗口来改（触发过的窗口会自动变成"当前窗口"）
 *   IN 24219   进圈里程改成 24219mm（窗口是相对它的，会一起走）
 *   WIN        把窗口表打出来
 *
 * 实测（5 组 TIK1/TIK2，原始编码器里程）：
 *   进圈 ≈ 24219（±136）
 *   第一个切点 ≈ 进圈 + 1067（±65）  ← 理论 πR=942，实测偏长 13%
 * ============================================================ */
#define GW_MASK_R   0x01u   /* 屏蔽右转：e > 0 -> 0 */
#define GW_MASK_L   0x02u   /* 屏蔽左转：e < 0 -> 0 */
#define GW_SLOW     0x04u   /* 窗口期减速：目标速度 * slow_pct/100 */
#define GW_FORCE    0x08u   /* 窗口期【强制转向】：误差直接写成 g_gourd_fix_err（负=左）*/
#define GW_WIN_MAX  8u

typedef struct
{
	const char *name;
	int32_t at_mm;      /* 相对进圈里程的窗口中心(mm) */
	int32_t w_mm;       /* 窗口全宽(mm) */
	int32_t spct;       /* 本窗口的减速百分比（100 = 不额外减速）*/
	int32_t fix;        /* 本窗口强制转向的差速百分比（负=左 正=右 0=不打）*/
	uint8_t act;        /* 动作位 */
} GourdWin_t;

/* 窗口表：动作位 bit0=禁右转 bit1=禁左转 bit2=减速 bit3=强制转向
   spct = 这个窗口自己的速度百分比（100 = 不额外减速） */
static GourdWin_t s_win[GW_WIN_MAX] =
{
	/* T1 切点 1：150mm 一小段，强制左转（力度 = g_gourd_fix_err，负=左），
	   不额外减速 —— 强打本来就是靠差速，再压速度左轮就没速度了。 */
	{ "T1", 1492, 150, 100, -25, GW_FORCE },

	/* T2 出口：实测 TIK 在 28457（相对进圈 +4238）。
	   现场要求：更慢 + 直接右转。30% 慢速 + 30% 差速右打。 */
	/* 出口：30% 慢速 + 硬右转。差速 100 = 右轮停住、左轮翻倍（绕右轮原地转），
	   嫌不够就 SEL 2 → FIX 130（右轮反转）。 */
	{ "T2", 4238, 400,  30, +100, GW_FORCE | GW_SLOW }
};

static uint8_t s_win_n    = 2u;
static uint8_t s_win_sel  = 0u;      /* 当前被 MOVE/WID 编辑的窗口 */
static uint8_t s_win_mask = 0u;      /* 本拍生效的动作位 */
static int32_t s_win_spct = 100;     /* 本拍生效的窗口减速百分比（取最慢） */
static int32_t s_win_fix  = 0;       /* 本拍生效的强制转向差速百分比 */
/* 减速百分比已经改成【每个窗口自己的】（见 GourdWin_t.spct），
   这个全局量留作兼容，不再影响控制。 */
int g_gourd_seg_pct  = 60;           /* 整个葫芦圈段的速度百分比（串口 GSL 可改）*/

/* 【固定打角】借另一队的做法：整段葫芦圈不看灰度，直接把转向误差写成固定值，
   靠圆环几何自己走（他们 15 路模拟量，写的是 -25/±35 ≈ 满偏 71%）。
   我们 8 路加权误差满偏 ±50，等价力度约 -35。0 = 关闭（用光电/窗口逻辑）。 */
/* 强制转向的差速百分比已经改成【每个窗口各自的】（GourdWin_t.fix），
   这个全局量留作兼容，不再影响控制。 */

int g_gourd_in_mm = 24219;           /* 进圈里程（原始编码器 mm）*/

static uint8_t s_gst       = 0u;     /* 0=没进圈 1=圈内 */
static uint8_t s_win_armed = 0u;     /* 本圈哪些窗口还没触发过（位图）*/

/* 【窗口统计】跑完用串口 WST 读 —— "参数到底有没有起作用"的硬证据 */
static uint32_t s_cnt_in       = 0u;   /* 进圈次数 */
static uint32_t s_cnt_trig     = 0u;   /* 窗口触发次数 */
static uint32_t s_cnt_win_tick = 0u;   /* 窗口生效的控制拍数 */
static int32_t  s_win_mm_min   =  2147483647;
static int32_t  s_win_mm_max   = -2147483648;

/* 每 5ms 由 NAV_Update 调一次 */
static void NAV_GourdOdo(void)
{
	int32_t enc = (int32_t)(ENCODER_GetDist() * 1000.0f);
	uint8_t i;

	if (s_gst == 0u)
	{
		/* 进圈：里程一过入口就进状态机（+6m 以内才认，防止倒车/推车误触）*/
		if (enc >= g_gourd_in_mm && enc < g_gourd_in_mm + 6000)
		{
			s_gst = 1u;
			s_win_armed = 0u;
			for (i = 0u; i < s_win_n; i++) s_win_armed |= (uint8_t)(1u << i);

			s_cnt_in++;

			USART1_SendString("NOW IN\r\n");
		}
	}
	else
	{
		/* 跑出去（或退回去）就当离开这一段 */
		/* 段长 5.5m：4 个半圆 ≈ 4 x πR ≈ 3.8m（R=300）再加余量。
		   固定打角只在这一段里生效，过了就交回光电，不然会一直往左绕。 */
		if (enc < g_gourd_in_mm - 300 || enc > g_gourd_in_mm + 5500) s_gst = 0u;
	}

	s_win_mask = 0u;
	s_win_spct = 100;
	s_win_fix  = 0;

	if (s_gst == 1u)
	{
		for (i = 0u; i < s_win_n; i++)
		{
			int32_t c = g_gourd_in_mm + s_win[i].at_mm;
			int32_t h = s_win[i].w_mm / 2;

			if (enc >= c - h && enc < c + h)
			{
				s_win_mask |= s_win[i].act;

				/* 减速位生效就用这个窗口自己的百分比，多窗口重叠取最慢 */
				if ((s_win[i].act & GW_SLOW) && s_win[i].spct < s_win_spct)
				{
					s_win_spct = s_win[i].spct;
				}

				/* 强制转向：用这个窗口自己的差速量 */
				if (s_win[i].act & GW_FORCE)
				{
					s_win_fix = s_win[i].fix;
				}
				s_win_sel   = i;          /* 刚触发过的窗口 = "当前窗口" */

				if (s_win_armed & (uint8_t)(1u << i))
				{
					s_win_armed &= (uint8_t)~(1u << i);

					/* 唯一允许主动上报的一行 */
					s_cnt_trig++;

					USART1_SendString("NOW MOVE ");
					USART1_SendNum((int32_t)i + 1);
					USART1_SendString(" ");
					USART1_SendNum(enc);
					USART1_SendString("\r\n");
				}
			}
		}
	}

	/* 统计：这一拍窗口是否生效（放在这里，NAV_GourdMask 每拍会被读好几次）*/
	if (s_win_mask != 0u)
	{
		s_cnt_win_tick++;
		if (enc < s_win_mm_min) s_win_mm_min = enc;
		if (enc > s_win_mm_max) s_win_mm_max = enc;
	}

	/* 【注意】窗口期不再连发串口：TXE 中断会和控制环抢时间，
	   窗口的实时指示交给 PC13 蓝灯（main.c 里常亮），串口只留
	   一次性标记（NOW IN / NOW MOVE）。
	   代价是串口上看不到窗口内的实时里程，需要时再用 TIK 打点。 */
}

/* ---- 对外 ---- */
uint8_t NAV_GourdMask(void) { return s_win_mask; }

/* 窗口里把目标速度压到 g_gourd_slow_pct%；窗口外原样返回 */
/* 目标速度按百分比压下来：
     窗口内   -> 用窗口自己的 slow_pct（默认 40）
     圈内其它 -> 用整段全局的 seg_pct（默认 70）
     圈外     -> 原样返回
   两个都是"乘在目标速度上"，再经斜率限制，所以是平滑减速。 */
int32_t NAV_GourdSlowTgt(int32_t tgt)
{
	int32_t pct;

	if (s_win_mask != 0u && s_win_spct < 100) pct = s_win_spct;
	else if (s_gst == 1u)                     pct = g_gourd_seg_pct;
	else                                      return tgt;

	if (pct < 10)  pct = 10;
	if (pct > 100) pct = 100;

	return tgt * pct / 100;
}

int32_t NAV_GourdSlowPct(void) { return (s_win_sel < s_win_n) ? s_win[s_win_sel].spct : 100; }
int32_t NAV_GourdSegPct(void)  { return g_gourd_seg_pct; }
uint8_t NAV_GourdSeg(void)     { return s_gst; }

/* ---- 窗口统计（WST 用）---- */
int32_t NAV_GourdFixErr(void) { return s_win_fix; }

void NAV_GourdSetFixErr(int32_t e)
{
	if (s_win_sel >= s_win_n) return;
	if (e >  150) e =  150;     /* 100 = 内轮停转（绕内轮原地转）；150 = 内轮反转 */
	if (e < -150) e = -150;
	s_win[s_win_sel].fix = e;
}

uint32_t NAV_WinStatIn(void)     { return s_cnt_in; }
uint32_t NAV_WinStatTrig(void)   { return s_cnt_trig; }
uint32_t NAV_WinStatTicks(void)  { return s_cnt_win_tick; }
int32_t  NAV_WinStatMinMm(void)  { return (s_win_mm_min == 2147483647) ? 0 : s_win_mm_min; }
int32_t  NAV_WinStatMaxMm(void)  { return (s_win_mm_max == -2147483648) ? 0 : s_win_mm_max; }

void NAV_WinStatClear(void)
{
	s_cnt_in = 0u;
	s_cnt_trig = 0u;
	s_cnt_win_tick = 0u;
	s_win_mm_min =  2147483647;
	s_win_mm_max = -2147483648;
}

void NAV_GourdSetSegPct(int32_t pct)
{
	if (pct < 10)  pct = 10;
	if (pct > 100) pct = 100;
	g_gourd_seg_pct = pct;
}

void NAV_GourdSetSlowPct(int32_t pct)
{
	if (s_win_sel >= s_win_n) return;
	if (pct < 5)   pct = 5;
	if (pct > 100) pct = 100;
	s_win[s_win_sel].spct = pct;
}
int32_t NAV_GourdSt(void)    { return (int32_t)s_gst; }
int32_t NAV_GourdEntry(void) { return g_gourd_in_mm; }

void NAV_GourdSetEntry(int32_t mm)
{
	g_gourd_in_mm = mm;
	s_gst = 0u;
}

uint8_t NAV_WinCount(void) { return s_win_n; }
uint8_t NAV_WinSel(void)   { return s_win_sel; }
void    NAV_WinSelect(uint8_t i) { if (i < s_win_n) s_win_sel = i; }
int32_t NAV_WinAbs(uint8_t i) { return (i < s_win_n) ? (g_gourd_in_mm + s_win[i].at_mm) : 0; }
int32_t NAV_WinW(uint8_t i)   { return (i < s_win_n) ? s_win[i].w_mm : 0; }
int32_t NAV_WinSlow(uint8_t i){ return (i < s_win_n) ? s_win[i].spct : 100; }
int32_t NAV_WinFix(uint8_t i) { return (i < s_win_n) ? s_win[i].fix : 0; }
int32_t NAV_WinAct(uint8_t i) { return (i < s_win_n) ? (int32_t)s_win[i].act : 0; }

void NAV_WinMove(int32_t dmm)
{
	if (s_win_sel < s_win_n) s_win[s_win_sel].at_mm += dmm;
}

/* 改当前窗口的动作位（0~7）：bit0 禁右转 / bit1 禁左转 / bit2 减速 */
void NAV_WinSetAct(int32_t act)
{
	if (s_win_sel >= s_win_n) return;
	if (act < 0)  act = 0;
	if (act > 15) act = 15;
	s_win[s_win_sel].act = (uint8_t)act;
}

void NAV_WinSetWidth(int32_t mm)
{
	if (s_win_sel >= s_win_n) return;
	if (mm < 20)     mm = 20;
	if (mm > 3000)   mm = 3000;
	s_win[s_win_sel].w_mm = mm;
}

/* ---- 出口右转（第 4 个动作）---- */
#define NAV_GOURD_EXIT_MM  3850       /* 出口位置：放车/入口起算。
                                         实测在读数 3985 处转正好，但读数滞后动作约 72mm，
                                         再留一点冗余 -> 3850。 */
#define NAV_EXIT_DEG       62         /* 出口要转的角度：85->80->70->62（本车实测老转多）*/
#define NAV_EXIT_PWM       2600       /* 出口转向轮速 */
#define NAV_EXIT_TIMEOUT   2000u      /* 超时保护(ms) */
static uint8_t  s_exit_done = 0;
static uint8_t  s_exiting   = 0;
static int32_t  s_exit_yaw0 = 0;
static uint32_t s_exit_tick = 0;
static uint8_t  s_exit_ongood = 0;   /* 转向中"看到线且基本对正"的连续拍数 */

/* 葫芦圈（4 个相切圆=蛇形）在赛道上的里程区间，从地图量出来的 */
static int32_t s_yawc16 = 0;        /* 光电修出来的航向补偿角，x16 定点 */
static int32_t s_odo10 = 0;           /* 累计里程，0.1mm */
static int32_t s_lap_seq = 0;         /* 里程绕回 0 的次数 = 跑完的圈数 */
static int32_t s_odo_raw = 0;       /* 上一次编码器的原始累计(0.1mm)，用来算增量 */
/* ---- 十字路口锚点 ----
   车过十字（8 路全亮 + 车基本直着走）时记下里程 XC:；
   若 g_cross_nom_mm 不为 0，就把里程【重置】成它 —— 每圈经过十字都把里程
   拨回同一个值，进葫芦圈的时机就永远一致（不受漂移影响）。
   0 = 只记录、不重置（第一次测量用）。 */
volatile int g_cross_nom_mm = 15300;   /* 实测两趟 15306/15227，取 15300 */
static int32_t s_cross_odo = 0;
static int32_t s_entry_odo = 0;
static uint8_t s_cross_armed = 1u;     /* 1 = 本圈还没锚过（每圈只锚第一个十字）*/
static uint8_t s_gourd_done  = 0u;     /* 1 = 本圈葫芦圈已走完，不许再进圈。
                                        （出圈后里程仍落在进圈窗口里，会被当成又进一次圈，
                                         速度被 4000 压住、直道上累计转角不涨 -> 永远解不掉）*/
/* ---- 直道终点锚点（葫芦圈前那段长直道的终点）----
   判定：长直道(>1.5m)之后【刚开始转弯】的那一刻。
   它离葫芦圈入口只有 4~5 米，比十字近得多 -> 漂移更小 -> 进圈更准。 */
volatile int g_straight_nom_mm = 20268;   /* 直道终点里程(实测 19998+20539 取平均) */
static int32_t  s_str_odo  = 0;           /* 直道起点里程（转弯时不断刷新）*/
static int32_t  s_corner_odo = 0;
static uint8_t  s_corner_armed = 0u;
#define NAV_DS_CLAMP   80            /* 5ms 内允许的最大里程增量(0.1mm)=8mm=1.6m/s。
                                        路面颠一下轮子离地，编码器会瞬间多计几十个计数，
                                        超过物理上限的一律当噪声挡掉。 */
static int32_t s_odo_lst = 0;       /* 上一拍的里程 */
static int32_t s_rec_lst = 0;       /* 上一次记点时的里程 */
static uint8_t s_mapping = 0;

/* ---- 转向环内部 ---- */
static int32_t s_err_f16 = 0;
static int16_t s_last_err = 0;
static int32_t s_idx = 0;
static int32_t s_last_e = 0;

/* ---- sin 表：Q15，0~360 度分 256 份，多存一格 360 度方便插值 ---- */
static const int16_t s_sin_tab[257] = {
	     0,   804,  1608,  2410,  3212,  4011,  4808,  5602,  6393,  7179,  7962,  8739,  9512,
	 10278, 11039, 11793, 12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530, 18204, 18868,
	 19519, 20159, 20787, 21403, 22005, 22594, 23170, 23731, 24279, 24811, 25329, 25832, 26319,
	 26790, 27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956, 30273, 30571, 30852, 31113,
	 31356, 31580, 31785, 31971, 32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757, 32767,
	 32757, 32728, 32678, 32609, 32521, 32412, 32285, 32137, 31971, 31785, 31580, 31356, 31113,
	 30852, 30571, 30273, 29956, 29621, 29268, 28898, 28510, 28105, 27683, 27245, 26790, 26319,
	 25832, 25329, 24811, 24279, 23731, 23170, 22594, 22005, 21403, 20787, 20159, 19519, 18868,
	 18204, 17530, 16846, 16151, 15446, 14732, 14010, 13279, 12539, 11793, 11039, 10278,  9512,
	  8739,  7962,  7179,  6393,  5602,  4808,  4011,  3212,  2410,  1608,   804,     0,  -804,
	 -1608, -2410, -3212, -4011, -4808, -5602, -6393, -7179, -7962, -8739, -9512,-10278,-11039,
	-11793,-12539,-13279,-14010,-14732,-15446,-16151,-16846,-17530,-18204,-18868,-19519,-20159,
	-20787,-21403,-22005,-22594,-23170,-23731,-24279,-24811,-25329,-25832,-26319,-26790,-27245,
	-27683,-28105,-28510,-28898,-29268,-29621,-29956,-30273,-30571,-30852,-31113,-31356,-31580,
	-31785,-31971,-32137,-32285,-32412,-32521,-32609,-32678,-32728,-32757,-32767,-32757,-32728,
	-32678,-32609,-32521,-32412,-32285,-32137,-31971,-31785,-31580,-31356,-31113,-30852,-30571,
	-30273,-29956,-29621,-29268,-28898,-28510,-28105,-27683,-27245,-26790,-26319,-25832,-25329,
	-24811,-24279,-23731,-23170,-22594,-22005,-21403,-20787,-20159,-19519,-18868,-18204,-17530,
	-16846,-16151,-15446,-14732,-14010,-13279,-12539,-11793,-11039,-10278, -9512, -8739, -7962,
	 -7179, -6393, -5602, -4808, -4011, -3212, -2410, -1608,  -804,     0,
};

/* ------------------------------------------------------------
 * 取 0.1 度对应的 sin，Q15（-32768 ~ 32767）。查表 + 线性插值。
 * 比调用库函数 sinf 省得多，而且耗时确定（软浮点 sinf 要几百个周期）。
 * ------------------------------------------------------------ */
static int32_t Nav_Sin(int32_t a10)
{
	int32_t q, i, f;

	a10 %= 3600;
	if (a10 < 0) a10 += 3600;

	q = a10 * 256;                 /* 0 ~ 921600 */
	i = q / 3600;                  /* 0 ~ 255 */
	f = q - i * 3600;              /* 0 ~ 3599 */

	return (int32_t)s_sin_tab[i]
	     + ((int32_t)s_sin_tab[i + 1] - (int32_t)s_sin_tab[i]) * f / 3600;
}

static int32_t Nav_Cos(int32_t a10)
{
	return Nav_Sin(a10 + 900);
}

/* 整数开方（牛顿迭代） */
int32_t Nav_Sqrt(int32_t v)
{
	int32_t x, y;

	if (v <= 0) return 0;
	x = v;
	y = (x + 1) / 2;
	while (y < x)
	{
		x = y;
		y = (x + v / x) / 2;
	}
	return x;
}

/* ------------------------------------------------------------
 * 两段弦夹角的正弦，单位 1/1000（-1000 ~ 1000）。
 *   i: 起点下标；k1: 第一段弦跨几个点；k2: 第二段弦跨几个点
 * 用正弦代替角度，省掉 atan2；判断"弯急不急"完全够用。
 * 弦长直接用 k*DS 近似（点是等距采的），省掉开方。
 * ------------------------------------------------------------ */
static int32_t Nav_AngSin(int32_t i, int32_t k1, int32_t k2)
{
	int32_t n = (int32_t)s_map_n;
	int32_t i1, i2, i3;
	int32_t ax, ay, bx, by, cr, L1, L2;

	if (n < 8) return 0;

	i1 = i;                while (i1 >= n) i1 -= n;   while (i1 < 0) i1 += n;
	i2 = i1 + k1;          while (i2 >= n) i2 -= n;
	i3 = i2 + k2;          while (i3 >= n) i3 -= n;

	ax = (int32_t)s_map_x[i2] - (int32_t)s_map_x[i1];
	ay = (int32_t)s_map_y[i2] - (int32_t)s_map_y[i1];
	bx = (int32_t)s_map_x[i3] - (int32_t)s_map_x[i2];
	by = (int32_t)s_map_y[i3] - (int32_t)s_map_y[i2];

	cr = ax * by - ay * bx;                 /* 叉积，mm*mm 量级 */
	L1 = k1 * NAV_DS_MM;
	L2 = k2 * NAV_DS_MM;
	if (L1 == 0 || L2 == 0) return 0;

	return (cr * 1000) / (L1 * L2);         /* 千分之几 */
}

/* ------------------------------------------------------------
 * 每 5ms：航向积分 + 位置积分 + 建图记点
 * ------------------------------------------------------------ */
/* 按【编码器里程 + 发车偏置】取路表下标（整圈循环） */
static int32_t NAV_MapIdx(void)
{
	int32_t idx, n = (int32_t)s_map_n;

	if (n < 1) return 0;
	idx = (s_odo10 + (int32_t)g_nav_start_off_mm * 10) / (NAV_DS_MM * 10);
	while (idx >= n) idx -= n;
	while (idx < 0)  idx += n;
	return idx;
}

/* 【背图变速】这一拍的基础速度。没表返回 0（调用方用 g_base_speed）。 */
int32_t NAV_SpeedBase(void)
{
	int32_t idx;

	if (s_map_n < 8) return 0;

	idx = NAV_MapIdx();
	s_spd_lim = (int32_t)g_nav_map_spd[idx];

	/* 表认为"该降多少"(255-表值) 按 g_nav_corner_pct 放大：
	   直道表值=255 -> 该降=0 -> 怎么放大都不动；
	   弯道表值低 -> 降得更多 -> 圈外弯道整体变慢。 */
	{
		int32_t d  = 255 - s_spd_lim;
		int32_t k  = (g_nav_corner_pct > 50) ? g_nav_corner_pct : 50;
		int32_t fr = 255 - d * k / 100;

		if (fr < 40) fr = 40;
		return (int32_t)g_nav_map_vmax * fr / 255;
	}
}

int32_t NAV_GetSpeedLimit(void) { return s_spd_lim; }

/* ------------------------------------------------------------
 * 葫芦圈自动接管
 *   进圈瞬间：把【位置】用路表重新对齐（车一直在线上，实物≈表），
 *             航向继续用陀螺积分。不重对齐的话，陀螺漂移在前面 23 米里
 *             积出来的位置偏差可能有好几米，照表开反而更歪。
 * ------------------------------------------------------------ */
uint8_t NAV_InGourd(void)
{
	if (!g_gourd_en) return 0u;

	/* 返回【状态】而不是当场按里程算：进圈时置位、出口转完才清。
	   这样即使中途编码器被路面颠出错，限速也不会中途放开。 */
	return s_in_gourd;
}

/* 葫芦圈切点强制转向（每拍调用） */
int8_t NAV_GourdFlip(int16_t gray_err, uint8_t lost)
{
	uint8_t i;
	int32_t e;

	if (!g_gourd_en) return 0;

	if (s_flip_state == 0u)
	{
		/* 空闲：到切点就强制打一把 */
		for (i = 0u; i < 3u; i++)
		{
			if (!(s_flip_done & (uint8_t)(1u << i)) &&
			    s_gturn >= s_flip_deg[i])
			{
				s_flip_done  |= (uint8_t)(1u << i);
				s_flip_now    = s_flip_dir[i];
				s_flip_until  = g_ms_tick + NAV_FLIP_MS;
				s_flip_state  = 1u;
			}
		}
	}
	else if (s_flip_state == 1u)
	{
		/* 强制转向结束 -> 放开手，让光电自己去找线，开始验证 */
		if ((int32_t)(s_flip_until - g_ms_tick) <= 0)
		{
			s_flip_state   = 2u;
			s_verify_bad   = 0u;
			s_verify_until = g_ms_tick + NAV_VERIFY_MS;
		}
	}
	else if (s_flip_state == 2u)
	{
		/* 验证：看光电有没有把线跟回来 */
		e = (int32_t)gray_err;
		if (e < 0) e = -e;
		if (lost || e > 45) s_verify_bad++;

		if ((int32_t)(s_verify_until - g_ms_tick) <= 0)
		{
			/* 超过 60% 的拍都"没跟回来" -> 这次扳错了（比如方向反了）*/
			if (s_verify_bad > (uint16_t)((NAV_VERIFY_MS / 5) * 3 / 5))
			{
				s_fix_now    = (int8_t)(-s_flip_now);   /* 反向拉回线上 */
				s_flip_until = g_ms_tick + NAV_FIX_MS;
				s_flip_state = 3u;
			}
			else
			{
				s_flip_state = 0u;                      /* 扳对了，收工 */
			}
		}
	}
	else /* 3：反向纠正中 */
	{
		if ((int32_t)(s_flip_until - g_ms_tick) <= 0) s_flip_state = 0u;
	}

	if (s_flip_state == 1u) return s_flip_now;
	if (s_flip_state == 3u) return s_fix_now;
	return 0;
}

/* 强制转向状态（遥测）：0空闲 1正在强制 2验证中 3正在反向纠正 */
int32_t NAV_GourdFlipState(void) { return (int32_t)s_flip_state; }

/* 【诊断用】本拍有没有切点"角度已经到点、但还没扳"——
   也就是强推正被"8 路全亮"挡着下不了手的那几拍。
   纯读状态，不改任何行为；只给遥测 FA: 用。 */
int8_t NAV_GourdPending(void)
{
	uint8_t i;

	if (!g_gourd_en) return 0;
	if (!s_in_gourd) return 0;
	if (s_flip_state != 0u) return 0;          /* 正在扳/验证中，不算 */

	for (i = 0u; i < 3u; i++)
	{
		if (!(s_flip_done & (uint8_t)(1u << i)) && s_gturn >= s_flip_deg[i]) return 1;
	}
	return 0;
}
int32_t  NAV_GourdTurn(void)     { return s_gturn / 1000; }
int32_t  NAV_SnakeAbs(void)      { return s_trn_abs / 1000; }

int32_t  NAV_CrossOdo(void)       { return s_cross_odo; }
int32_t  NAV_EntryOdo(void)       { return s_entry_odo; }
int32_t  NAV_CornerOdo(void)      { return s_corner_odo; }

/* 光电那边检测到"十字"时调这个（8 路全亮 + 陀螺几乎不转）。
   只在合理里程窗口内认，并做 300mm 去抖。 */
void NAV_CrossDetected(void)
{
	int32_t mm = s_odo10 / 10;

	if (!s_cross_armed) return;                 /* 本圈已经锚过一次，后面十字全忽略 */
	if (mm < 15000 || mm > 21000) return;       /* 里程窗口。17~20 太窄错过十字了，放宽到 15~21 */

	s_gourd_done  = 0u;                         /* 过了十字算新一圈，重新允许进圈 */
	s_corner_armed = 0u;                        /* 直道终点锚点也重新武装 */
	s_cross_armed = 0u;                         /* 一圈只锚一次：
	                                               两个十字挨得近，都锚会把它们之间的距离抹掉 */
	s_cross_odo   = mm;

	if (g_cross_nom_mm > 0)                  /* 有定标值才真重置 */
	{
		s_odo10     = (int32_t)g_cross_nom_mm * 10;
		s_rec_lst   = s_odo10;
		s_entry_odo = 0;
	}
}   /* 自上次直道以来的绝对转角(度) */   /* 累计转角(度)，遥测用 */

/* ------------------------------------------------------------
 * 出口右转
 *   到出口里程（入口+3858mm）就开始右转 —— 那里实物线是"绕着最后一个圆"
 *   还是"拐出去"两条路，光电同样分不出来（跟切点一样），必须主动拐。
 *   转的角度用陀螺闭环（不靠时间），转够就交回光电。
 * ------------------------------------------------------------ */
uint8_t NAV_GourdExit(int32_t *l, int32_t *r, int16_t gray_err, uint8_t lost)
{
	int32_t turned;

	if (!g_gourd_en) return 0u;

	if (!s_exit_done && s_gturn >= NAV_EXIT_DEG_TURN)
	{
		s_exit_done = 1u;
		s_exiting   = 1u;
		s_exit_yaw0 = s_yaw_acc;
		s_exit_tick = g_ms_tick;
		s_exit_ongood = 0;
	}

	if (!s_exiting) return 0u;

	turned = s_yaw_acc - s_exit_yaw0;          /* 右转会让它变负（左转为正）*/

	/* 【看到线就停】转向途中如果光电探到线、而且基本对正（|E| 小），
	   说明车已经压在出圈线上 -> 立刻结束，不用死磕固定角度。
	   （转弯中车跟线是斜的，|E| 会很大，所以"|E| 小"基本只可能是
	     已经对上出圈线了。） */
	{
		int32_t e = (int32_t)gray_err;
		if (e < 0) e = -e;
		if (!lost && e <= 25)
		{
			if (s_exit_ongood < 250u) s_exit_ongood++;
		}
		else
		{
			s_exit_ongood = 0;
		}
	}

	/* 【必须转过 40% 之后才允许"看到线就停"】
	   刚进转向时车本来就压在圆上的线上(|E|很小)，不设这个门槛会立刻
	   判定"已经对上了"、9ms 就取消转向 —— 现象就是"到位置了却不转"。 */
	if ((s_exit_ongood >= 20u && turned <= -(int32_t)NAV_EXIT_DEG * 750) ||
	    turned <= -(int32_t)NAV_EXIT_DEG * 1000 ||
	    (uint32_t)(g_ms_tick - s_exit_tick) > NAV_EXIT_TIMEOUT)
	{
		s_exiting = 0u;                        /* 转完，交回光电 */
		/* 出圈状态先不清：等 1.5 秒（NAV_Update 里清），把出圈后那个直角弯也走完 */
		return 0u;
	}

	/* 快转完的最后 25 度把力度收到 40% —— 直接满舵松手会让车一顿，
	   收一点再交给光电，衔接顺得多。 */
	{
		int32_t rem = (int32_t)NAV_EXIT_DEG * 1000 - (-turned);   /* 还差多少(0.001度) */

		if (rem < 8000)
		{
			int32_t p = NAV_EXIT_PWM * rem / 8000;

			if (p < 2000) p = 2000;
			*l = p;
			*r = 0;                        /* 画弧：右轮停住，边走边转 */
			return 1u;
		}
	}

	/* 【必须"边走边转"，不能原地打转】
	   原来写成左轮正转+右轮反转 = 绕自身中心转，转完车还在原来那个圆的线上，
	   光电接着就把它绕回去 -> 一直打圈。现在右轮停住：车一边往前一边转，
	   转完会离开那个圆、落到出圈线上。 */
	*l =  NAV_EXIT_PWM;
	*r =  0;
	return 1u;
}

void NAV_Update(int16_t w_dps)
{
	int32_t ds10, yaw;

	if (s_upd_cnt < 100000000) s_upd_cnt++;      /* 诊断：这一拍到底有没有跑 */

	/* 这一拍实际过了多少毫秒 —— 【不能假设 5ms】，主循环慢了就得按真实
	   时间积分，否则积分会等比例少掉（转 90 度只积出来 2 度）。 */
	{
		uint32_t now = g_ms_tick;
		int32_t  dt  = (int32_t)(now - s_yaw_tick);

		s_yaw_tick = now;
		if (dt <= 0 || dt > 200) dt = NAV_INT_MS;    /* 异常就退回标称 5ms */

		/* 死区按角速度砍（单位还是 度/秒） */
		if      (w_dps >  NAV_GYRO_DB) w_dps -= NAV_GYRO_DB;
		else if (w_dps < -NAV_GYRO_DB) w_dps += NAV_GYRO_DB;
		else                           w_dps = 0;

		/* ω(度/秒) × 毫秒 = 0.001 度，直接累加，不做除法就不会被截断 */
		s_yaw_acc += (int32_t)w_dps * dt;

		/* 葫芦圈进度：累计绝对转角（0.001度）。用陀螺不用里程 ——
		   路面不平/轮子打滑会让里程忽多忽少，而陀螺量的是车实际转了多少。 */
		if (s_in_gourd)
		{
			/* 【死区补偿】把死区加回来 1 度/秒。
			   死区(±2度/秒)在长转弯里会吃掉约 4% 的转角 —— 4 个半圆累计只到
			   约 690 度，出口(720度)就永远触发不了。 */
			int32_t w2 = (int32_t)w_dps;

			if      (w2 > 0) w2 += 1;
			else if (w2 < 0) w2 -= 1;
			s_gturn += (w2 < 0) ? (-w2 * dt) : (w2 * dt);
		}

		/* 蛇形识别用的两个累加：转弯就累，连续直道 300ms 清零 */
		if (w_dps != 0)
		{
			int32_t d10 = (int32_t)w_dps * dt;
			int32_t mmy = s_odo10 / 10;

			/* 【直道终点】长直道(>1.5m)之后刚开始转弯的那一刻 */
			if (!s_corner_armed && (mmy - s_str_odo) > 1500 &&
			    mmy > 18000 && mmy < 23000)
			{
				s_corner_armed = 1u;
				s_corner_odo   = mmy;
				if (g_straight_nom_mm > 0)          /* 有定标值就重置里程 */
				{
					s_odo10   = (int32_t)g_straight_nom_mm * 10;
					s_rec_lst = s_odo10;
				}
			}
			s_str_odo = mmy;                        /* 转弯中不断刷新 = 直道起点 */

			s_trn_sgn += d10;
			s_trn_abs += (d10 < 0) ? -d10 : d10;
			s_str_cnt  = 0;
		}
		else
		{
			if (s_str_cnt < 60000u) s_str_cnt++;
			if (s_str_cnt >= 60u)
			{
				/* 连续直道 300ms -> 全部清零。
				   葫芦圈的【累计转角】也在这里清零 —— 这样 0 点永远是"蛇形真正
				   的起点"，跟"进圈判定早一点/晚一点"无关（否则进圈前那段弯的
				   转角会混进来，出口就提前触发了）。 */
				s_trn_abs = 0; s_trn_sgn = 0;
				if (!s_exit_done) s_gturn = 0;
			}
		}
	}

	/* ---- 1. 里程（编码器给的是米，换成 0.1mm）---- */
	/* ---- 0. 葫芦圈进出：进圈后一切按累计转角走，不再看里程 ---- */
	if (g_gourd_en && !s_in_gourd && !s_gourd_done)
	{
		int32_t mm = s_odo10 / 10 + (int32_t)g_nav_start_off_mm;

		{
			int32_t ad = s_trn_abs / 1000;          /* 度 */
			int32_t sd = s_trn_sgn / 1000;

			if (sd < 0) sd = -sd;

			/* 条件一：里程门（原来那套，最准）；
			   条件二：蛇形特征（绝对转角大、带符号却很小 -> 来回绕 = 蛇形） */
			if ((mm >= NAV_GOURD_S0_MM && mm <= NAV_GOURD_S1_MM) ||
			    (NAV_USE_SNAKE_ENTRY && mm > NAV_SNAKE_ARM_MM && ad > NAV_SNAKE_ABS_DEG && sd * 2 < ad))
			{
			s_in_gourd  = 1u;
			s_gturn     = 0;
			s_entry_odo = s_odo10 / 10;      /* 记下进圈里程（遥测 XE:）*/
			s_flip_done = 0u; s_flip_state = 0u;
			s_exit_done = 0u; s_exiting    = 0u;
			s_trn_abs   = 0;  s_trn_sgn    = 0;  s_str_cnt = 0;
			}
		}
	}

	/* 出口转完 1.5 秒后才真正退出葫芦圈（限速解除）。
	   出圈紧跟着就是那个直角弯（顶点约 D:3981），一放速就会全速冲进弯里。 */
	if (s_in_gourd && s_exit_done &&
	    (uint32_t)(g_ms_tick - s_exit_tick) > 1500u)
	{
		s_in_gourd   = 0u;
		s_gourd_done = 1u;                      /* 本圈走完了，别再进圈 */
		s_cross_armed = 1u;                     /* 下一圈重新武装，可以再锚一次 */
	}

	/* 【自己累加限幅后的增量】，不再直接用编码器的累计值：
	   否则路面颠出来的假计数会一次性灌进里程，越跑越偏。 */
	{
		int32_t raw = (int32_t)(ENCODER_GetDist() * 10000.0f);
		int32_t d10 = raw - s_odo_raw;

		s_odo_raw = raw;
		if      (d10 >  NAV_DS_CLAMP) d10 =  NAV_DS_CLAMP;
		else if (d10 < -NAV_DS_CLAMP) d10 = -NAV_DS_CLAMP;
		s_odo10 += d10;
	}
	ds10 = s_odo10 - s_odo_lst;
	s_odo_lst = s_odo10;

	/* ---- 跑满一整圈：里程绕回 0 ----
	   为什么必须绕：十字锚点、直道终点锚点、葫芦圈入口窗口全是"里程落在
	   某一个区间里"的判据。里程只涨不减的话，第二圈起这些窗口就永远
	   匹配不上（锚点不再触发、葫芦圈不再接管）。绕回 0 以后每一圈都从
	   同一个数出发，多圈跑得通，调参器要的"每圈一行 LAP"也才有圈可数。 */
	if (!s_mapping && s_map_n > 0u)
	{
		int32_t len10 = (int32_t)s_map_n * NAV_DS_MM * 10;

		if (len10 > 0 && s_odo10 >= len10)
		{
			s_odo10   -= len10;
			s_odo_lst -= len10;
			if (s_rec_lst > len10) s_rec_lst -= len10;
			s_lap_seq++;
		}
	}

	/* ---- 2. 航向积分已在上面按真实 dt 做完了 ---- */

	/* ---- 3. 位置积分：这一小段走了多远、朝哪个方向 ---- */
	if (ds10 != 0 && ds10 < NAV_JUMP_LIMIT && ds10 > -NAV_JUMP_LIMIT)
	{
		int32_t vx, vy;

		yaw = s_yaw_acc / 100 + s_yawc16 / 16;   /* 积分用的是【补偿后】的航向 */

		/* 四舍五入：cos/sin 是 Q15（cos0 = 32767 不是 32768），
		   直接 /32768 会让【每一拍都少算 0.1mm】，累计能差 9%（实测里程
		   1197mm 时位置只走到 1093mm）。正负分开处理才对称。 */
		vx = ds10 * Nav_Cos(yaw);
		vy = ds10 * Nav_Sin(yaw);
		s_x10 += (vx >= 0) ? ((vx + 16384) / 32768) : -(((-vx) + 16384) / 32768);
		s_y10 += (vy >= 0) ? ((vy + 16384) / 32768) : -(((-vy) + 16384) / 32768);
	}

	/* ---- 3.5 诊断量：车【自己】相对路表的横向偏移（左正，mm）----
	   原来看只在惯导跑里算，光电模式下 CT: 永远是 0，而调参的
	   "惯导横向偏差"要用它 —— 改成每 5ms 都算（光电/建图/惯导三种模式都有）。
	   顺便也是自检：车真压在线上（E≈0）时这个数也该≈0。 */
	{
		int32_t n2 = (int32_t)s_map_n;

		if (n2 >= 8)
		{
			int32_t idx2 = NAV_MapIdx();
			int32_t j2 = idx2 + 2;
			int32_t ax, ay, vx, vy, den;

			while (j2 >= n2) j2 -= n2;
			ax = (int32_t)s_map_x[idx2] * 10;
			ay = (int32_t)s_map_y[idx2] * 10;
			vx = (int32_t)s_map_x[j2] * 10 - ax;
			vy = (int32_t)s_map_y[j2] * 10 - ay;
			den = Nav_Sqrt(vx * vx + vy * vy);
			if (den != 0)
				s_ct_mm = ((s_x10 - ax) * (-vy) + (s_y10 - ay) * vx) / den / 10;
		}
	}

	/* ---- 3.8 葫芦圈里程窗口状态机 ---- */
	NAV_GourdOdo();

	/* ---- 4. 建图：每走 NAV_DS_MM 记一个点 ---- */
	if (s_mapping)
	{
		while (s_odo10 - s_rec_lst >= (int32_t)(NAV_DS_MM * 10))
		{
			s_rec_lst += NAV_DS_MM * 10;
			if (s_map_n < NAV_N_MAX)
			{
				s_map_x[s_map_n] = (int16_t)(s_x10 / 10);
				s_map_y[s_map_n] = (int16_t)(s_y10 / 10);
				s_map_n++;
			}
		}
	}
}

/* ------------------------------------------------------------
 * 直道分类（建图结束时算一次）
 *   只有 kind=2 的段允许光电修航向 —— 弯里光电量的是车头前方那一小段，
 *   跟惯导的横向对不齐，修了反而更糟。直道两端各留一段当过渡，不修。
 * ------------------------------------------------------------ */
static void NAV_ClassifyStraights(void)
{
	uint16_t n = s_map_n;
	uint32_t i, st, len, k;
	int32_t min_pts, trim_pts;
	int32_t sin_th;

	for (i = 0; i < n; i++) s_map_kind[i] = 0;
	if (n < 24) return;

	/* 【可调】直道角度门限用 Nav_Sin 现算：30（=3.0 度）-> 约 53/1000 */
	sin_th = (Nav_Sin(g_nav_straight_deg10) * 1000) / 32768;
	if (sin_th < 5) sin_th = 5;

	min_pts  = g_nav_straight_min / NAV_DS_MM;   /* 至少这么长才算直道 */
	trim_pts = 90 / NAV_DS_MM;      /* 两端各裁 90mm */
	if (min_pts < 4) min_pts = 4;
	if (trim_pts < 1) trim_pts = 1;

	/* 第一遍：把"近直"的点先标 1 */
	for (i = 0; i + 8 < (uint32_t)n; i++)
	{
		int32_t s = Nav_AngSin((int32_t)i, 2, 2);   /* 32mm 弦 + 32mm 弦 */
		if (s < 0) s = -s;
		if (s < sin_th) s_map_kind[i] = 1;
	}

	/* 第二遍：够长的连续段，中段标 2 */
	i = 0;
	while (i < (uint32_t)n)
	{
		if (s_map_kind[i] != 1) { i++; continue; }
		st = i;
		while (i < (uint32_t)n && s_map_kind[i] == 1) i++;
		len = i - st;
		if (len >= (uint32_t)min_pts)
		{
			for (k = st; k < i; k++)
			{
				if ((k - st) < (uint32_t)trim_pts ||
				    ((i - 1) - k) < (uint32_t)trim_pts)
					s_map_kind[k] = 1;
				else
					s_map_kind[k] = 2;
			}
		}
	}
}

/* ------------------------------------------------------------
 * 光电为辅：只在直道中段，把光电量到的实际横向和惯导算的横向一比，
 * 拧一下陀螺的航向漂移（比例，不积分，不会积累出错）。
 * 弯里和丢线时都不动它（保持上一次的值，带过弯）。
 * ------------------------------------------------------------ */
static void NAV_YawFix(int32_t idx)
{
	int32_t n = (int32_t)s_map_n;
	int32_t j, ax, ay, vx, vy, den, ct, ct_true, e_lat, target16;

	if (!g_nav_corr_on)       return;       /* 调参键 corr_on = 0 时整段停用 */
	if (s_map_kind[idx] != 2) return;       /* 只在直道中段 */
	if (GRAY_IsLost())        return;       /* 丢线冻结 */

	j = idx + 2;                            /* 用 32mm 弦算横向 */
	while (j >= n) j -= n;

	ax = (int32_t)s_map_x[idx] * 10;
	ay = (int32_t)s_map_y[idx] * 10;
	vx = (int32_t)s_map_x[j] * 10 - ax;
	vy = (int32_t)s_map_y[j] * 10 - ay;

	den = Nav_Sqrt(vx * vx + vy * vy);
	if (den == 0) return;

	/* 惯导横向：当前估计位置到这一段路表的垂直距离（左正）*/
	ct = ((s_x10 - ax) * (-vy) + (s_y10 - ay) * vx) / den;

	/* 真横向：光电 E: 换成 mm。E: 正 = 线在车右边 = 车偏左 -> 横向为正 */
	ct_true = (int32_t)GRAY_GetError() * g_nav_mm_per_e * 10;

	e_lat = ct - ct_true;                   /* 惯导比真车多偏了多少 */

	/* 目标补偿角：惯导偏左(e_lat>0) = 积分把航向拧左了 -> 往右拧 */
	target16 = -(e_lat * g_nav_corr_k) / 1000;   /* ×1000 定点：800 等价于原来的 16/20 */
	if (target16 >  NAV_CORR_MAX16) target16 =  NAV_CORR_MAX16;
	if (target16 < -NAV_CORR_MAX16) target16 = -NAV_CORR_MAX16;

	/* 限速靠近，别突变 */
	if      (target16 > s_yawc16 + 2) s_yawc16 += 2;
	else if (target16 < s_yawc16 - 2) s_yawc16 -= 2;
	else                              s_yawc16  = target16;
}

/* ------------------------------------------------------------
 * 惯导跑的控制拍
 * ------------------------------------------------------------ */
void NAV_Run(void)
{
	int32_t n, idx = 0, j, k, dx, dy, sx, cx, lat10, err, base, corr, turn, turn_near, l, r, w;
	int32_t per;

	if (s_map_n < 8) { s_pwm_avg = 0; Set_Car_Speed(0, 0); return; }   /* 没表：停住 */

	/* 【保命】连续丢线超过 0.4 秒 = 车已经不在赛道上了 —— 直接停住。
	   惯导跑是"照着表开"，它自己不知道车掉下去了，不停就会一直开出去。 */
	/* 【保命】连续丢线超过 0.4 秒 = 车已经不在赛道上了 -> 停住。
	   【必须用真实时间】—— 之前写成"每拍加 5ms"，可 NAV_Run 是每 0.45ms
	   跑一次，等于丢了 36ms 就刹停，稍微一闪就停车。 */
	if (GRAY_IsLost())
	{
		if (s_lost_tick == 0u) s_lost_tick = g_ms_tick;
		if (s_odo10 > 5000 && (uint32_t)(g_ms_tick - s_lost_tick) > 400u)
		{
			s_lost_brake = 1;
			s_pwm_avg = 0;
			Set_Car_Speed(0, 0);
			return;
		}
	}
	else
	{
		s_lost_tick = 0u;
	}

	n = (int32_t)s_map_n;

	/* ---- 1. 进度下标：按编码器里程取，整圈循环 ---- */
	idx = (s_odo10 + (int32_t)g_nav_start_off_mm * 10) / (NAV_DS_MM * 10);
	while (idx >= n) idx -= n;
	while (idx < 0)  idx += n;
	s_idx = idx;

	/* ---- 2. 前瞻点（沿路表往前数，不是沿车头伸出去）
	       【弯道里前瞻要缩短】：前瞻太长会抄近路切弯（纯跟踪的固有偏差
	       约 L^2/(2R)，L=160mm 在 R=50mm 的弯上偏差 250mm，弯直接抄没了）。
	       直道用 160mm，急弯缩到 60mm。 */
	/* 【两个预判分开】
	   转向的前瞻缩短：看近处(96mm)，够用就行 —— 看太远会"提早转"；
	   减速的前瞻：看远处(160mm)，提前把速度降下来，否则冲进弯里。 */
	turn_near = Nav_AngSin(idx, 2, 4);      /* 32mm 弦 与 96mm 弦 */
	turn      = Nav_AngSin(idx, 2, 8);      /* 32mm 弦 与 160mm 弦（减速用）*/
	if (turn_near < 0) turn_near = -turn_near;
	if (turn_near > 1000) turn_near = 1000;
	if (turn < 0) turn = -turn;
	if (turn > 1000) turn = 1000;

	k = g_nav_look_mm / NAV_DS_MM;
	k -= turn_near / 300;                   /* 缩短幅度也收着点，别一转就贴脸 */
	if (k < NAV_LOOK_MIN) k = NAV_LOOK_MIN;
	j = idx + k;   while (j >= n) j -= n;

	dx = (int32_t)s_map_x[j] * 10 - s_x10;
	dy = (int32_t)s_map_y[j] * 10 - s_y10;

	/* ---- 3. 车体坐标下的横向误差（左正，0.1mm）---- */
	sx = Nav_Sin(s_yaw_acc / 100 + s_yawc16 / 16);
	cx = Nav_Cos(s_yaw_acc / 100 + s_yawc16 / 16);
	lat10 = (-sx * dx + cx * dy) / 32768;

	/* ---- 4. 换成 E: 的量纲（E: 正 = 线在车的右边）。
	        目标在车左边(lat>0) 就是"线在左边"，所以取负号。 ---- */
	per = (g_nav_mm_per_e > 0) ? g_nav_mm_per_e : 1;
	err = -((lat10 / 10) / per);
	if (err >  50) err =  50;
	if (err < -50) err = -50;
	s_last_e = err;

	/* ---- 4.5 光电辅助（直道段）----
	   直道的"形状"就是把线走直，惯导没有额外信息；而光电是直接量车相对
	   实物线的横向偏差，最准。所以【直道段让光电占 2/3】，把 CAD 与实物线
	   的偏差、以及任何横漂都拉回来。弯道段不用光电（弯里它量的是车头前方
	   一小段，跟路表对不齐，反而添乱）。 */
	if (s_map_kind[idx] == 2 && !GRAY_IsLost())
	{
		int32_t ge = (int32_t)GRAY_GetError();   /* 正 = 线在车的右边，与 err 同号 */

		if (ge >  25) ge =  25;                  /* 限幅：一瞬间的闪动别打飞方向 */
		if (ge < -25) ge = -25;
		err = (err + 2 * ge) / 3;
	}

	/* ---- 5. 误差低通 ---- */
	s_err_f16 += (err * 16 - s_err_f16) / NAV_ERR_FILT_N;
	err = (int32_t)(s_err_f16 / 16);

	/* ---- 6. 速度：弯道自动减速（turn 上面已经算好）---- */
	base = g_nav_vmax - (g_nav_vmax - g_nav_vmin) * turn / 700;
	if (base < g_nav_vmin) base = g_nav_vmin;

	/* 偏得越多越慢（保险）*/
	base -= ((g_kslow * ((err < 0) ? -err : err)) / 1000) / 3;

	/* 葫芦圈里先限速：那一段是相切圆，跟不住就丢线，先确认能跟住 */
	if (s_in_gourd && base > NAV_GOURD_VMAX) base = NAV_GOURD_VMAX;
	if (base > g_nav_vmax) base = g_nav_vmax;
	if (base < 300) base = 300;

	/* ---- 7. 转向 PD + 陀螺阻尼 ---- */
	corr = -((g_kp * err + g_kd * (err - s_last_err)) / 1000);   /* ×1000 定点 */
	s_last_err = (int16_t)err;

	w = (int32_t)IMU_GetGyroZDps();
	corr -= (g_nav_kgyro * w) / 1000;

	l = base - corr;
	r = base + corr;
	if (l > MOTOR_SPEED_MAX) l = MOTOR_SPEED_MAX;
	if (r > MOTOR_SPEED_MAX) r = MOTOR_SPEED_MAX;
	if (l < 0) l = 0;
	if (r < 0) r = 0;

	s_pwm_avg = (l + r) / 2;
	Set_Car_Speed(l, r);

	/* ---- 8. 光电为辅：直道上修航向 ---- */
	NAV_YawFix(idx);
}

/* ------------------------------------------------------------
 * 建图 / 跑图 的起停
 * ------------------------------------------------------------ */
void NAV_StartMap(void)
{
	ENCODER_Reset();
	s_odo10 = 0;  s_odo_lst = 0;  s_cross_armed = 1u;  s_gourd_done = 0u;  s_corner_armed = 0u;  s_str_odo = 0;  s_odo_raw = 0;  s_rec_lst = 0;
	s_x10 = 0;    s_y10 = 0;      s_yaw_acc = 0; s_yawc16 = 0; s_yaw_tick = g_ms_tick;
	s_map_n = 0;
	s_mapping = 1;
	s_lap_seq = 0;
}

void NAV_StopMap(void)
{
	s_mapping = 0;
	NAV_ClassifyStraights();
}

uint8_t NAV_IsMapping(void) { return s_mapping; }

void NAV_StartRun(void)
{
	int32_t i0;

	ENCODER_Reset();
	s_odo10 = 0;  s_odo_lst = 0;  s_cross_armed = 1u;  s_gourd_done = 0u;  s_corner_armed = 0u;  s_str_odo = 0;  s_odo_raw = 0;

	/* 位置初值跟"发车点偏置"对齐：偏置不为 0 时，车不是从表的第 0 点出发的 */
	i0 = (int32_t)g_nav_start_off_mm * 10 / (NAV_DS_MM * 10);
	if (i0 < 0) i0 = 0;
	if (s_map_n > 0 && i0 >= (int32_t)s_map_n) i0 = (int32_t)s_map_n - 1;
	if (s_map_n > 0)
	{
		s_x10 = (int32_t)s_map_x[i0] * 10;
		s_y10 = (int32_t)s_map_y[i0] * 10;
	}
	else
	{
		s_x10 = 0;  s_y10 = 0;
	}
	s_yaw_acc = 0; s_yawc16 = 0; s_yaw_tick = g_ms_tick;
	s_err_f16 = 0; s_last_err = 0; s_idx = 0; s_last_e = 0;
	s_lost_tick = 0; s_lost_brake = 0;
	s_lap_seq = 0;

	/* 【必须一起清】葫芦圈的所有状态 —— 否则用 0x04 单测过一次之后，
	   这个标志一直是 1，接着按 0x05 跑整圈时会【全程被当成在葫芦圈里】，
	   速度一直被 4000 压住（= 满速的 55%），直道弯道都明显变慢。 */
	s_in_gourd  = 0u;
	s_gturn     = 0;
	s_flip_done = 0u; s_flip_state = 0u;
	s_exit_done = 0u; s_exiting    = 0u;
	s_trn_abs   = 0;  s_trn_sgn    = 0;  s_str_cnt = 0;
	s_in_gourd = 0;
	s_flip_done = 0; s_flip_until = 0; s_flip_now = 0;
}

/* ------------------------------------------------------------
 * 存 Flash / 上电读回来
 *
 * 存在片内 Flash 48KB 处（0x0800C000）：代码只占前面 10KB，撞不上。
 * 【存之前必须停车】—— 擦一页要几十毫秒，这段时间控制是停的。
 * ------------------------------------------------------------ */
#define NAV_MAGIC       0x4E41u      /* 'AN' */
#define NAV_VER         0x0101u
#define NAV_HDR_HW      6u           /* 头占 6 个半字 */
#define NAV_FLASH_PAGES 16u          /* 预留 16 页 = 16KB */

static uint32_t NAV_HWCount(uint16_t n)
{
	return NAV_HDR_HW + (uint32_t)n * 2u + ((uint32_t)n + 3u) / 4u;
}

static uint32_t NAV_PagesUsed(uint16_t n)
{
	uint32_t pages = (NAV_HWCount(n) * 2u + 1023u) / 1024u;
	if (pages > NAV_FLASH_PAGES) pages = NAV_FLASH_PAGES;
	return pages;
}

/* 第 i 个半字该写什么（chk 只在写校验和那一格时用）*/
static uint16_t NAV_HW(uint32_t i, uint16_t n, uint16_t chk)
{
	uint32_t j, v;

	if (i == 0) return NAV_MAGIC;
	if (i == 1) return NAV_VER;
	if (i == 2) return n;
	if (i == 3) return (uint16_t)NAV_DS_MM;
	if (i == 4) return (uint16_t)NAV_PagesUsed(n);
	if (i == 5) return chk;

	i -= NAV_HDR_HW;
	if (i < n) return (uint16_t)s_map_x[i];
	i -= n;
	if (i < n) return (uint16_t)s_map_y[i];
	i -= n;

	v = 0;
	for (j = 0; j < 4u; j++)
	{
		uint32_t k = i * 4u + j;
		if (k < n) v |= ((uint32_t)s_map_kind[k] & 3u) << (j * 2u);
	}
	return (uint16_t)v;
}

uint16_t NAV_Restore(void)
{
	const uint16_t *p = (const uint16_t *)NAV_FLASH_ADDR;
	uint32_t i, total, sum = 0;
	uint16_t n;

	if (p[0] != NAV_MAGIC) return 0;
	if (p[1] != NAV_VER)   return 0;

	n = p[2];
	if (n < 8u || n > NAV_N_MAX) return 0;

	total = NAV_HWCount(n);
	for (i = 0; i < total; i++)
	{
		if (i != 5u) sum += (uint32_t)p[i];
		sum &= 0xFFFFu;
	}
	if ((uint16_t)sum != p[5]) return 0;

	for (i = 0; i < n; i++)
	{
		s_map_x[i] = (int16_t)p[NAV_HDR_HW + i];
		s_map_y[i] = (int16_t)p[NAV_HDR_HW + n + i];
	}
	for (i = 0; i < n; i++)
	{
		s_map_kind[i] = (uint8_t)((p[NAV_HDR_HW + 2u * n + i / 4u] >> ((i % 4u) * 2u)) & 3u);
	}

	s_map_n = n;
	return n;
}

uint8_t NAV_Store(void)
{
	uint32_t i, total, pages, sum = 0;
	FLASH_Status st;
	uint16_t n = s_map_n;
	uint16_t chk;

	if (n < 8 || n > NAV_N_MAX) return 0;

	total = NAV_HWCount(n);
	pages = NAV_PagesUsed(n);

	/* 先算校验和（那一格先当 0）*/
	for (i = 0; i < total; i++)
	{
		sum += (uint32_t)NAV_HW(i, n, 0u);
		sum &= 0xFFFFu;
	}
	chk = (uint16_t)sum;

	FLASH_Unlock();

	for (i = 0; i < pages; i++)
	{
		st = FLASH_ErasePage(NAV_FLASH_ADDR + i * 1024u);
		if (st != FLASH_COMPLETE) { FLASH_Lock(); return 0; }
	}

	for (i = 0; i < total; i++)
	{
		st = FLASH_ProgramHalfWord(NAV_FLASH_ADDR + i * 2u, NAV_HW(i, n, chk));
		if (st != FLASH_COMPLETE) { FLASH_Lock(); return 0; }
	}

	FLASH_Lock();

	/* 写完全部读回来验一遍，不放心就当没存成功 */
	return (NAV_Restore() == n) ? 1u : 0u;
}

/* ------------------------------------------------------------ */
void NAV_Init(void)
{
	s_map_n = 0;
	s_mapping = 0;
	NAV_StartRun();
}

/* 上电装表：优先用 Flash 里【自己推】的那圈；没有就用【内置的 CAD 中心线】。
   内置表不用推车；自己推的表更贴实物线，推过就用推的。 */
uint8_t NAV_LoadMap(void)
{
	uint16_t i, n;

	if (NAV_Restore() != 0u) return 1u;      /* Flash 里有表，直接用 */

	n = g_nav_map_n;
	if (n > NAV_N_MAX) n = NAV_N_MAX;

	for (i = 0; i < n; i++)
	{
		s_map_x[i] = g_nav_map_x[i];
		s_map_y[i] = g_nav_map_y[i];
	}
	s_map_n = n;
	NAV_ClassifyStraights();                 /* 直道分类：光电只在这几段修航向 */
	return 2u;                               /* 2 = 用的是内置表 */
}

/* ---- 遥测 ---- */
int32_t  NAV_Xmm(void)       { return s_x10 / 10; }
int32_t  NAV_Ymm(void)       { return s_y10 / 10; }
int32_t  NAV_Yaw10(void)     { return s_yaw_acc / 100; }
int32_t  NAV_UpdCnt(void)    { return s_upd_cnt; }
int32_t  NAV_GetCrossTrack(void) { return s_ct_mm; }
int32_t  NAV_GetLostBrake(void)  { return (int32_t)s_lost_brake; }
int32_t  NAV_GetPwm(void)        { return s_pwm_avg; }
int32_t  NAV_YawComp10(void) { return s_yawc16 / 16; }
int32_t  NAV_Idx(void)       { return s_idx; }
int32_t  NAV_LastErr(void)   { return s_last_e; }
int32_t  NAV_LapSeq(void)    { return s_lap_seq; }
int32_t  NAV_OdoMm(void)     { return s_odo10 / 10; }
uint16_t NAV_Count(void)     { return s_map_n; }
