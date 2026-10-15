#include "NAV.h"
#include "mycontrol.h"
#include "ENCODER.h"
#include "IMU.h"
#include "GRAY.h"
#include "MOTOR.h"
#include "TIM1_TICK.h"
#include "stm32f10x_flash.h"

/* ============================================================
 * 惯性导航 —— 建图 / 惯导跑
 *
 * 这个文件的思路只有一句话：
 *   把【已经走过的路】记下来，下一次按里程照着这张表开。
 * 所以相切点、十字、直角弯全都不需要判断 —— 表里本来就是对的路径。
 * ============================================================ */

/* ---- 可调参数 ---- */
volatile int g_nav_vmax     = 2200;   /* 惯导跑：直道速度（PWM）。
                                          第一次试跑先用 2200（慢）；惯导跑没有光电
                                          位置反馈，表没对上时快了救不回来。跑通再加。 */
volatile int g_nav_vmin     = 900;    /* 惯导跑：最急弯的速度 */
volatile int g_nav_look_mm  = 160;    /* 前瞻距离(mm)：大了稳，小了跟得紧 */
volatile int g_nav_kgyro    = 2;      /* 陀螺阻尼增益 */
volatile int g_nav_corr_div = 20;     /* 直道光电修航向强度：越大越慢
                                         （20 -> 偏差 10mm 给 0.5 度）*/
volatile int g_nav_mm_per_e = 1;      /* 光电 E: 1 点约等于多少 mm（现场标定）*/

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
static uint16_t s_lost_ms  = 0;     /* 连续丢线多少毫秒（保命用）*/
static int32_t  s_ct_mm    = 0;     /* 车相对路表的横向偏移 mm（左正），遥测用 */
static uint8_t  s_lost_brake = 0;   /* 1 = 保命刹停触发过（诊断：是不是它把车按住的）*/
static int32_t  s_pwm_avg    = 0;   /* 这一拍下发的平均速度（诊断：到底给没给速度）*/
static int32_t s_yawc16 = 0;        /* 光电修出来的航向补偿角，x16 定点 */
static int32_t s_odo10 = 0;         /* 累计里程，0.1mm */
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
	}

	/* ---- 1. 里程（编码器给的是米，换成 0.1mm）---- */
	s_odo10 = (int32_t)(ENCODER_GetDist() * 10000.0f);
	ds10 = s_odo10 - s_odo_lst;
	s_odo_lst = s_odo10;

	/* ---- 2. 航向积分已在上面按真实 dt 做完了 ---- */

	/* ---- 3. 位置积分：这一小段走了多远、朝哪个方向 ---- */
	if (ds10 != 0 && ds10 < NAV_JUMP_LIMIT && ds10 > -NAV_JUMP_LIMIT)
	{
		yaw = s_yaw_acc / 100 + s_yawc16 / 16;   /* 积分用的是【补偿后】的航向 */
		s_x10 += ds10 * Nav_Cos(yaw) / 32768;
		s_y10 += ds10 * Nav_Sin(yaw) / 32768;
	}

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
	const int32_t sin_th = 52;      /* sin(3 度) = 0.052 -> 52/1000 */

	for (i = 0; i < n; i++) s_map_kind[i] = 0;
	if (n < 24) return;

	min_pts  = 200 / NAV_DS_MM;     /* 至少 200mm 长才算直道 */
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
	int32_t n, idx = 0, j, k, dx, dy, sx, cx, lat10, err, base, corr, turn, l, r, w;
	int32_t per;

	if (s_map_n < 8) { s_pwm_avg = 0; Set_Car_Speed(0, 0); return; }   /* 没表：停住 */

	/* 【保命】连续丢线超过 0.4 秒 = 车已经不在赛道上了 —— 直接停住。
	   惯导跑是"照着表开"，它自己不知道车掉下去了，不停就会一直开出去。 */
	if (GRAY_IsLost())
	{
		if (s_lost_ms < 60000u) s_lost_ms += 5u;
		/* 【发车那半米先不管】发车点在 S、G 两条线中间的空地上，
		   刚起步灰度可能看不到线；0.4 秒就刹停 = "按了开跑没反应"。
		   所以跑出 0.5 米之后这条才生效。 */
		if (s_lost_ms > 400u && s_odo10 > 5000)
		{
			s_lost_brake = 1;
			s_pwm_avg = 0;
			Set_Car_Speed(0, 0);
			return;
		}
	}
	else
	{
		s_lost_ms = 0;
	}

	n = (int32_t)s_map_n;

	/* ---- 1. 进度下标：按编码器里程取，整圈循环 ---- */
	idx = s_odo10 / (NAV_DS_MM * 10);
	while (idx >= n) idx -= n;
	while (idx < 0)  idx += n;
	s_idx = idx;

	/* ---- 0. 诊断量：车【自己】相对路表的横向偏移（左正，mm）----
	   和光电的 E: 比一下：车真在线上时（E≈0）这个也该≈0；
	   两个符号相反就说明"表/坐标系镜像了"。 */
	{
		int32_t j2 = idx + 2, ax, ay, vx, vy, den;

		if (j2 >= n) j2 -= n;
		ax = (int32_t)s_map_x[idx] * 10;
		ay = (int32_t)s_map_y[idx] * 10;
		vx = (int32_t)s_map_x[j2] * 10 - ax;
		vy = (int32_t)s_map_y[j2] * 10 - ay;
		den = Nav_Sqrt(vx * vx + vy * vy);
		if (den != 0)
			s_ct_mm = ((s_x10 - ax) * (-vy) + (s_y10 - ay) * vx) / den / 10;
	}

	/* ---- 2. 前瞻点（沿路表往前数，不是沿车头伸出去）---- */
	k = g_nav_look_mm / NAV_DS_MM;
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

	/* ---- 5. 误差低通 ---- */
	s_err_f16 += (err * 16 - s_err_f16) / NAV_ERR_FILT_N;
	err = (int32_t)(s_err_f16 / 16);

	/* ---- 6. 速度：按表上前方两段弦的夹角规划，弯道自动减速 ---- */
	turn = Nav_AngSin(idx, 2, 4);           /* 32mm 弦 与 96mm 弦 的夹角 */
	if (turn < 0) turn = -turn;
	if (turn > 1000) turn = 1000;
	base = g_nav_vmax - (g_nav_vmax - g_nav_vmin) * turn / 1000;

	/* 偏得越多越慢（保险）*/
	base -= (g_kslow * ((err < 0) ? -err : err)) / 3;
	if (base > g_nav_vmax) base = g_nav_vmax;
	if (base < 300) base = 300;

	/* ---- 7. 转向 PD + 陀螺阻尼 ---- */
	corr = -(g_kp * err + g_kd * (err - s_last_err));
	s_last_err = (int16_t)err;

	w = (int32_t)IMU_GetGyroZDps();
	corr -= g_nav_kgyro * w;

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
	s_odo10 = 0;  s_odo_lst = 0;  s_rec_lst = 0;
	s_x10 = 0;    s_y10 = 0;      s_yaw_acc = 0; s_yawc16 = 0; s_yaw_tick = g_ms_tick;
	s_map_n = 0;
	s_mapping = 1;
}

void NAV_StopMap(void)
{
	s_mapping = 0;
	NAV_ClassifyStraights();
}

uint8_t NAV_IsMapping(void) { return s_mapping; }

void NAV_StartRun(void)
{
	ENCODER_Reset();
	s_odo10 = 0;  s_odo_lst = 0;
	s_x10 = 0;    s_y10 = 0;      s_yaw_acc = 0; s_yawc16 = 0; s_yaw_tick = g_ms_tick;
	s_err_f16 = 0; s_last_err = 0; s_idx = 0; s_last_e = 0;
	s_lost_ms = 0; s_lost_brake = 0;
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
uint16_t NAV_Count(void)     { return s_map_n; }
