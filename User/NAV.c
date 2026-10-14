#include "NAV.h"
#include "mycontrol.h"
#include "ENCODER.h"
#include "IMU.h"
#include "GRAY.h"
#include "MOTOR.h"
#include "TIM1_TICK.h"
#include "nav_map_fw.h"     /* 离线生成好的路表（const，占 Flash 不占 RAM）*/
#include "nav_sin_tab.h"    /* Q15 sin 表 */

/* ============================================================
 * 惯性导航 —— 地图直接输入版
 *
 * 一句话：路表是【别人量好、离线生成、编译进来】的，车照着表开。
 * 所以相切点、十字、直角弯全都不需要判断 —— 表里本来就是对的路径。
 *
 * 和 ins-nav 那版的差别就一处：不再"推车建图 + 存 Flash"，
 * 改成 #include nav_map_fw.h。附带好处是省掉约 11KB RAM 和 16KB Flash 预留
 * （C8T6 只有 20KB RAM / 64KB Flash，这一条很关键）。
 * ============================================================ */

/* ---- 可调参数 ---- */
volatile int g_nav_vmax     = 3900;   /* 惯导跑：直道速度（PWM） */
volatile int g_nav_vmin     = 900;    /* 惯导跑：最急弯的速度 */
volatile int g_nav_look_mm  = 150;    /* 前瞻距离(mm)。地图点距是 50mm，
                                         所以 150 = 3 个点。大了稳，小了跟得紧 */
volatile int g_nav_kgyro    = 2;      /* 陀螺阻尼增益 */
volatile int g_nav_corr_div = 20;     /* 直道光电修航向强度：越大越慢
                                         （20 -> 偏差 10mm 给 0.5 度）*/
volatile int g_nav_mm_per_e = 1;      /* 光电 E: 1 点约等于多少 mm（现场标定）*/

/* ---- 内部控制常量 ---- */
#define NAV_INT_MS       5      /* NAV_Update 的调用周期(ms) */
#define NAV_GYRO_DB      2      /* 陀螺死区(度/秒)：小于它当作 0，治静止漂移 */
#define NAV_ERR_FILT_N   8      /* 转向误差低通，和纯光电模式同一套 */
#define NAV_LOOK_MIN     2      /* 前瞻至少几个点（2 x 50mm = 100mm）*/
#define NAV_CORR_MAX16   288    /* 补偿角上限 18 度（x16 定点）*/
#define NAV_JUMP_LIMIT   2000   /* 单拍里程跳变限幅(0.1mm)：超过当异常丢掉 */

/* ---- 航位推算状态 ---- */
static int32_t s_x10 = 0;           /* 位置，0.1mm，场地坐标 */
static int32_t s_y10 = 0;
static int32_t s_yaw_acc = 0;       /* 航向积分累加器，单位 0.001 度。
                                       为什么不用 0.1 度直接加：慢速转弯时
                                       w*5/100 会被整数除法截断成 0，转一整圈
                                       也积不上，实测踩过。 */
static int32_t s_upd_cnt = 0;       /* NAV_Update 被调用次数（诊断用）*/
static uint32_t s_yaw_tick = 0;     /* 上次航向积分的时间戳(ms) */
static int32_t s_yawc16 = 0;        /* 光电修出来的航向补偿角，x16 定点 */
static int32_t s_odo10 = 0;         /* 累计里程，0.1mm */
static int32_t s_odo_lst = 0;       /* 上一拍的里程 */
static int32_t s_fx = 0;            /* 位置积分余数累加器（x），见 NAV_Update */
static int32_t s_fy = 0;            /* 位置积分余数累加器（y） */

/* ---- 转向环内部 ---- */
static int32_t s_err_f16 = 0;
static int16_t s_last_err = 0;
static int32_t s_idx = 0;
static int32_t s_last_e = 0;
static int32_t s_ydot = 0;          /* 最近一拍的带符号角速度(度/秒)，见 Nav_YDot */

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
static int32_t Nav_Sqrt(int32_t v)
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
	int32_t n = NAV_MAP_N;
	int32_t i1, i2, i3;
	int32_t ax, ay, bx, by, cr, L1, L2;

	if (n < 8) return 0;

	i1 = i;                while (i1 >= n) i1 -= n;   while (i1 < 0) i1 += n;
	i2 = i1 + k1;          while (i2 >= n) i2 -= n;
	i3 = i2 + k2;          while (i3 >= n) i3 -= n;

	ax = (int32_t)NAV_MAP[i2].x - (int32_t)NAV_MAP[i1].x;
	ay = (int32_t)NAV_MAP[i2].y - (int32_t)NAV_MAP[i1].y;
	bx = (int32_t)NAV_MAP[i3].x - (int32_t)NAV_MAP[i2].x;
	by = (int32_t)NAV_MAP[i3].y - (int32_t)NAV_MAP[i2].y;

	cr = ax * by - ay * bx;                 /* 叉积，mm*mm 量级 */
	L1 = k1 * NAV_MAP_DS_MM;
	L2 = k2 * NAV_MAP_DS_MM;
	if (L1 == 0 || L2 == 0) return 0;

	return (cr * 1000) / (L1 * L2);         /* 千分之几 */
}

/* 补偿后的航向，0.1 度 */
static int32_t Nav_Yaw10(void)
{
	return s_yaw_acc / 100 + s_yawc16 / 16;
}

/* ------------------------------------------------------------
 * 带符号的偏航角速度（度/秒），【地图坐标系】约定：右转为正。
 *
 * 整条符号链（推过一遍，别改错）：
 *   1. 地图 yaw：0 度 = +X，yaw 增大 = 朝 +Y 拐。位置积分写成
 *      x += ds*cos(yaw)、y += ds*sin(yaw)，与这个约定自洽。
 *   2. 差速：左轮快 -> 车右转 -> yaw 增大。代码里 l = base-corr、
 *      r = base+corr，所以 corr < 0 = 右转 = yaw 增大。
 *   3. 于是"车正右转"对应这里的 yaw_dot > 0，阻尼要把 corr 往正里推
 *      （corr 正 = 左转 = 把右转拉回来），所以 NAV_Run 里是 corr += k*ydot。
 *   4. yaw_dot 的符号由 NAV_YAW_SIGN 定，必须台架标定（见 NAV.h）。
 * ------------------------------------------------------------ */
static int32_t Nav_YDot(void)
{
	return s_ydot;
}

/* ------------------------------------------------------------
 * 每 5ms：航向积分 + 位置积分
 * ------------------------------------------------------------ */
void NAV_Update(int16_t w_dps)
{
	int32_t ds10, yaw;
	int32_t w;

	if (s_upd_cnt < 100000000) s_upd_cnt++;      /* 诊断：这一拍到底有没有跑 */

	/* ---- 1. 航向积分 ----
	   这一拍实际过了多少毫秒 —— 【不能假设 5ms】，主循环慢了就得按真实
	   时间积分，否则积分会等比例少掉（转 90 度只积出来 2 度）。 */
	{
		uint32_t now = g_ms_tick;
		int32_t  dt  = (int32_t)(now - s_yaw_tick);

		s_yaw_tick = now;
		if (dt <= 0 || dt > 200) dt = NAV_INT_MS;    /* 异常就退回标称 5ms */

		w = (int32_t)w_dps * NAV_YAW_SIGN;      /* 转成"右转为正" */

		/* 死区按角速度砍（单位还是 度/秒） */
		if      (w >  NAV_GYRO_DB) w -= NAV_GYRO_DB;
		else if (w < -NAV_GYRO_DB) w += NAV_GYRO_DB;
		else                       w = 0;

		s_ydot = w;      /* 阻尼用的是同一个带符号值，别再乘一次 NAV_YAW_SIGN */

		/* ω(度/秒) × 毫秒 = 0.001 度，直接累加，不做除法就不会被截断 */
		s_yaw_acc += w * dt;
	}

	/* ---- 2. 里程（编码器给的是米，换成 0.1mm）---- */
	s_odo10 = (int32_t)(ENCODER_GetDist() * 10000.0f);
	ds10 = s_odo10 - s_odo_lst;
	s_odo_lst = s_odo10;

	/* ---- 3. 位置积分：这一小段走了多远、朝哪个方向 ----
	   里程跳变保护：单拍跳变超过限幅就当异常丢掉（编码器噪声/回绕）。 */
	if (ds10 != 0 && ds10 < NAV_JUMP_LIMIT && ds10 > -NAV_JUMP_LIMIT)
	{
		yaw = Nav_Yaw10();                       /* 积分用【补偿后】的航向 */

		/* 【余数不能丢 —— 这里踩过坑】
		   以前写成 s_x10 += ds10*cos/32768，每次整除都截掉不到 1 个单位
		   （0.1mm）。NAV_Update 是 5ms 一拍 = 每秒 200 拍，慢慢推的时候
		   一拍才走几个单位，于是每秒最多丢 20mm。
		   实测：直线推 384mm，里程计说 553mm，位置只积出 384mm（丢 30%）
		   —— 丢多少跟推的速度有关，所以路表下标会持续落后，车跟不上地图。
		   现在把余数攒到下一拍，累积起来不再丢。 */
		s_fx += ds10 * Nav_Cos(yaw);
		s_x10 += s_fx / 32768;
		s_fx  = s_fx % 32768;

		s_fy += ds10 * Nav_Sin(yaw);
		s_y10 += s_fy / 32768;
		s_fy  = s_fy % 32768;
	}
}

/* ------------------------------------------------------------
 * 光电为辅：只在直道中段（kind == 2），把光电量到的实际横向和惯导算的
 * 横向一比，拧一下陀螺的航向漂移（比例，不积分，不会积累出错）。
 * 弯里和丢线时都不动它（保持上一次的值，带过弯）。
 *
 * kind 是离线算好写进路表的（弦角法），比固件里现算 R 可靠 ——
 * 原始 radius 列在笔直的线上也报过 R=110mm，噪声很大。
 * ------------------------------------------------------------ */
static void NAV_YawFix(int32_t idx)
{
	int32_t n = NAV_MAP_N;
	int32_t j, ax, ay, vx, vy, den, ct, ct_true, e_lat, target16;

	if (NAV_MAP[idx].kind != 2) return;     /* 只在直道中段 */
	if (GRAY_IsLost())          return;     /* 丢线冻结 */

	j = idx + 2;                            /* 用 100mm 弦算横向 */
	while (j >= n) j -= n;

	ax = (int32_t)NAV_MAP[idx].x * 10;
	ay = (int32_t)NAV_MAP[idx].y * 10;
	vx = (int32_t)NAV_MAP[j].x * 10 - ax;
	vy = (int32_t)NAV_MAP[j].y * 10 - ay;

	den = Nav_Sqrt(vx * vx + vy * vy);
	if (den == 0) return;

	/* 惯导横向：当前估计位置到这一段路表的垂直距离（左正）*/
	ct = ((s_x10 - ax) * (-vy) + (s_y10 - ay) * vx) / den;

	/* 真横向，同样以"车偏在线的右边为正"：
	   光电 E: 正 = 线在车的右边 = 车偏在线的【左】边 -> 车偏右为【负】，
	   所以要取负号。ct 和它同号，相减才有意义。 */
	ct_true = -(int32_t)GRAY_GetError() * g_nav_mm_per_e * 10;

	e_lat = ct - ct_true;                   /* 惯导比真车多偏了多少 */

	/* 目标补偿角：惯导偏左(e_lat>0) = 积分把航向拧左了 -> 往右拧 */
	target16 = -(e_lat * 16) / (g_nav_corr_div > 0 ? g_nav_corr_div : 20);
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
	int32_t n, idx, j, k, dx, dy, sx, cx, lat10, err, base, corr, turn, l, r;
	int32_t per;
	int32_t yaw;

	n = NAV_MAP_N;

	/* ---- 1. 进度下标：按编码器里程取，整圈循环 ----
	   闭环地图，跑完一圈自动回到 idx=0，不需要判圈也能一直跑。 */
	idx = s_odo10 / (NAV_MAP_DS_MM * 10);
	while (idx >= n) idx -= n;
	while (idx < 0)  idx += n;
	s_idx = idx;

	/* ---- 2. 前瞻点（沿路表往前数，不是沿车头伸出去）---- */
	k = g_nav_look_mm / NAV_MAP_DS_MM;
	if (k < NAV_LOOK_MIN) k = NAV_LOOK_MIN;
	j = idx + k;   while (j >= n) j -= n;

	/* ---- 3. 车体坐标下的横向误差（左正，0.1mm）---- */
	dx = (int32_t)NAV_MAP[j].x * 10 - s_x10;
	dy = (int32_t)NAV_MAP[j].y * 10 - s_y10;

	yaw = Nav_Yaw10();
	sx = Nav_Sin(yaw);
	cx = Nav_Cos(yaw);
	lat10 = (-sx * dx + cx * dy) / 32768;

	/* ---- 4. 换成 E: 的量纲 ----
	   lat10 的几何含义：前瞻点相对车头、沿【车的右侧】的分量
	   （推导：lat10 = (-sin, cos)·(dx,dy)，而 (-sin, cos) 正是车头方向
	    再转 +90 度，也就是车的右侧）。

	   所以 lat10 > 0 = 目标在车的右边 = 车偏在线的左边 = 线在右边 -> 该右转。
	   光电模式里 E: 正 也是"线在右边"、也是右转，两者同号 ——
	   这一步【不能再取负】，取了负就变成朝背离赛道的一侧打死。
	   （ins-nav 那一版这里是 -()，符号是反的。） ---- */
	per = (g_nav_mm_per_e > 0) ? g_nav_mm_per_e : 1;
	err = ((lat10 / 10) / per);
	if (err >  50) err =  50;
	if (err < -50) err = -50;
	s_last_e = err;

	/* ---- 5. 误差低通 ---- */
	s_err_f16 += ((int32_t)err * 16 - s_err_f16) / NAV_ERR_FILT_N;
	err = (int32_t)(s_err_f16 / 16);

	/* ---- 6. 速度：按表上前方两段弦的夹角规划，弯道自动减速 ----
	   50mm 点距下，2 点 = 100mm、4 点 = 200mm 的弦。 */
	turn = Nav_AngSin(idx, 2, 4);
	if (turn < 0) turn = -turn;
	if (turn > 1000) turn = 1000;
	base = g_nav_vmax - (g_nav_vmax - g_nav_vmin) * turn / 1000;

	/* 偏得越多越慢（保险）*/
	base -= (g_kslow * ((err < 0) ? -err : err)) / 3;
	if (base > g_nav_vmax) base = g_nav_vmax;
	if (base < 300) base = 300;

	/* ---- 7. 转向 PD + 陀螺阻尼 ----
	   陀螺只用来【阻尼】，不用来【定角】—— 角度靠上面的积分。 */
	corr = -(g_kp * err + g_kd * (err - s_last_err));
	s_last_err = (int16_t)err;

	/* 阻尼：车正在右转(yaw_dot>0) 时要把 corr 往正推（corr 正 = 左转），
	   才能把转过头拉回来 —— 写成减号就变成正反馈，越转越猛。 */
	corr += g_nav_kgyro * Nav_YDot();

	l = base - corr;
	r = base + corr;
	if (l > MOTOR_SPEED_MAX) l = MOTOR_SPEED_MAX;
	if (r > MOTOR_SPEED_MAX) r = MOTOR_SPEED_MAX;
	if (l < 0) l = 0;
	if (r < 0) r = 0;

	Set_Car_Speed(l, r);

	/* ---- 8. 光电为辅：直道上修航向 ---- */
	NAV_YawFix(idx);
}

/* ============================================================
 * 开始惯导跑：把位姿复位到地图上的发车点
 * ============================================================ */
void NAV_StartRun(void)
{
	ENCODER_Reset();

	s_odo10 = 0;  s_odo_lst = 0;  s_fx = 0;  s_fy = 0;

	/* 【关键】位置和航向都要对齐到地图的发车点。
	   nav_map_fw.h 里的 START_YAW_MDEG 是【从地图几何算出来的真实行进方向】，
	   不是 nav_map.h 里那列平滑航向 —— 那一列在 s=0 处有闭环伪影
	   （首段 500mm 明明沿 +X 走，它却写 -45 度），直接用会一开始就偏 45 度。 */
	s_x10 = (int32_t)NAV_MAP_START_X_MM * 10;
	s_y10 = (int32_t)NAV_MAP_START_Y_MM * 10;
	s_yaw_acc = NAV_MAP_START_YAW_MDEG;     /* 单位 0.001 度 */
	s_yawc16  = 0;
	s_yaw_tick = g_ms_tick;

	s_err_f16 = 0; s_last_err = 0; s_idx = 0; s_last_e = 0;
}

void NAV_Init(void)
{
	NAV_StartRun();
}

/* ---- 遥测 / 诊断 ---- */
int32_t NAV_Xmm(void)         { return s_x10 / 10; }
int32_t NAV_Ymm(void)         { return s_y10 / 10; }
int32_t NAV_Yaw10(void)       { return Nav_Yaw10(); }
int32_t NAV_YawComp10(void)   { return s_yawc16 / 16; }
int32_t NAV_Idx(void)         { return s_idx; }
int32_t NAV_LastErr(void)     { return s_last_e; }
int32_t NAV_OdoMm(void)       { return s_odo10 / 10; }
int32_t NAV_LapMm(void)       { return (int32_t)NAV_MAP_LAP_MM; }
int32_t NAV_UpdCnt(void)      { return s_upd_cnt; }
int32_t NAV_StraightNow(void) { return (s_idx >= 0 && s_idx < NAV_MAP_N) ? NAV_MAP[s_idx].kind : 0; }
