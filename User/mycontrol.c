#include "mycontrol.h"
#include "GRAY.h"
#include "MOTOR.h"
#include "ENCODER.h"
#include "TIM1_TICK.h"
#include "IMU.h"
#include "NAV.h"

/* ---- 主循环里各函数共享的状态 ---- */
volatile uint8_t g_mode = MODE_STOP;
volatile uint8_t g_dir  = DIR_NONE;

/* ---- 可调参数 ---- */
volatile int g_base_speed = 3900;   /* 直道基础速度 */
volatile int32_t g_kp     = 70000;       /* 转向环 P。误差已放大 10 倍(GRAY_ERR_SCALE)，
                                        所以这里等价于以前的 260。往上调就能提速 */
volatile int32_t g_kd     = 0;         /* 转向环 D，先关着 */
volatile int g_manual_fb  = 5000;   /* 手动前进/后退速度（占空比 5000/7199 = 69%）。
                                        只影响 0x01/0x02，循迹和转向不受影响。 */
volatile int32_t g_kgyro  = 3000;      /* 【陀螺阻尼】corr -= g_kgyro * ω(度/秒)。
                                         抑制"转过头"形成的极限环（弯里一顶一松的微抖）。
                                         以前用它会甩头，是因为当时陀螺 Z 轴符号是反的
                                         （正反馈）——现在符号已修正，可以正常用了。
                                         太大车会变钝/发飘：3 -> 5 -> 8 逐档试。 */
volatile int32_t g_kslow  = 16000;     /* 弯道降速（误差放大 10 倍后的等价值）。
                                         转弯半径 R = (轮距/2)*(外+内)/(外-内)
                                         ks200 -> R=115mm，ks320 -> R=81mm */

#define MANUAL_SPEED      3000      /* 蓝牙左转/右转的速度（0x03/0x04） */

/* 丢线时用的"假装偏差"，朝最后一次有效方向猛打一把，把车拉回线上 */
#define TRACE_LOST_ERR    50    /* 误差放大 10 倍，所以是 50 不是 5 */
/* 丢线时基础速度降到几分之一 */
#define TRACE_LOST_DIV    2
/* 降速后的速度下限（原来写死 300）—— 调参键 vmin 落到 g_trace_vmin */
volatile int32_t g_trace_vmin = 300;

#if USE_IMU
static uint32_t s_imu_tick = 0;            /* 陀螺读取限速 */

/* 误差低通滤波（定点 x16 保精度）。
   灰度误差是【离散台阶】(-50/-20/0/20/50...)，直接乘 Kp 会让电机指令
   一跳一跳，车头就甩。滤波把它磨成连续值。
   ERR_FILT_N 越大越平滑、越滞后：8 约 4ms，16 约 7ms，32 约 14ms。 */
#define ERR_FILT_N   8     /* 16->8：滞后从 7ms 降到约 4ms，蛇形弯跟得上 */
static int32_t s_err_f16 = 0;        /* 滤波后的误差，放大 16 倍 */
#endif
/* 速度斜坡（原来写死 +18 / -25）—— 调参键 a_acc / a_brake 落到这里 */
volatile int32_t g_ramp_up = 18;
volatile int32_t g_ramp_dn = 25;

/* 窗口屏蔽"真的把 err 削掉"的次数（WST 读；0 = 屏蔽一次都没生效）*/
volatile uint32_t g_mask_used = 0u;

static volatile int8_t  s_last_dir = 0;   /* 最后有效偏差方向：-1 左 / +1 右 / 0 未知 */

/* 诊断：葫芦圈里"切点角度已经到了、但被 8 路全亮挡着扳不出去"的拍数。
   纯计数，不改任何行为。跑一趟看遥测 S: 行末尾的 FA: ——>0 就说明
   相切点确实有一段是全亮的，强推在那里下不了手（要等全亮过去才扳）。 */
volatile uint32_t g_flip_allon_cnt = 0;
#define GOURD_TRACE_VMAX  4000            /* 光电模式下、葫芦圈内的速度上限(PWM) */
#define GOURD_FLIP_PWM    1800            /* 【切点强制转向的差速量(PWM)】
                                             历史：2000/200ms 那次太猛（转过头 40~80 度），
                                             之后一路收到 1200/100ms。这次按现场要求
                                             “每个转向力度大一些”，提到 1800。
                                             估算：100ms 里车头转约 17 度（原来 1200 约 11 度）。
                                             还嫌扳不过去 -> 上 2000；过头了 -> 退回 1400。
                                             想加“冲量”该动的是 NAV.c 的 NAV_FLIP_MS(100ms)，
                                             不是这里。 */
static int32_t s_base_ramp = 0;           /* 基础速度平滑值：背图速度表是一段一段的台阶，
                                             直接跳车会"一卡一卡"，这里做斜率限制 */
static int32_t s_w_acc  = 0;      /* 陀螺角速度累加（做 200ms 平均用） */
static int32_t s_w_cnt  = 0;
static int32_t s_w_avg  = 0;      /* 最近约 200ms 的平均角速度(度/秒，左正) */
static volatile int16_t s_last_err = 0;   /* 上一次的误差，给 D 项用 */

/* ------------------------------------------------------------ */
void Trace_Init(void)
{
	s_last_dir = 0;
	s_last_err = 0;
	GRAY_Init();          /* 硬件初始化只做这一次 */
}

void Trace_Reset(void)
{
	s_last_dir = 0;
	s_last_err = 0;
	s_base_ramp = 0;
	ENCODER_Reset();      /* 编码器里程也清零（暂时没用到，留着给以后的速度环） */
	g_flip_allon_cnt = 0;   /* 每趟重新记 */
}

/* ------------------------------------------------------------
 * 手动模式：按 g_dir 给固定速度
 * ------------------------------------------------------------ */
static void Manual_Run(void)
{
	switch (g_dir)
	{
		/* 前进/后退用 g_manual_fb（可调，试最大安全速度用）；
		   左转/右转仍用固定的 MANUAL_SPEED，保持可控。 */
		case DIR_FWD:   Set_Car_Speed( g_manual_fb,  g_manual_fb); break;
		case DIR_BACK:  Set_Car_Speed(-g_manual_fb, -g_manual_fb); break;
		case DIR_LEFT:  Set_Car_Speed(-MANUAL_SPEED,  MANUAL_SPEED); break;
		case DIR_RIGHT: Set_Car_Speed( MANUAL_SPEED, -MANUAL_SPEED); break;
		default:        Set_Car_Speed(0, 0); break;
	}
}

/* ------------------------------------------------------------
 * 循迹模式
 * ------------------------------------------------------------ */
static void Trace_Run(void)
{
	int err;

	/* 葫芦圈那一段仍然用光电转向（沿线走），只在【相切点】按里程强制打方向 ——
	   见下面 PD 之后的 NAV_GourdFlip()。 */

	/* 【出口右转】完全接管：只发速度，不看线 */
	{
		int32_t el, er;

		/* 【原来这里传的是上面那个还没赋值的 err】
		   未初始化变量的值是不确定的 —— 出口那个〈看到线就停〉的提前结束
		   判定(|E|<=25)就时对时错，是“不稳定”的一个来源。
		   Control_Tick 开头已经采过灰度了，取本拍的 GRAY_GetError() 才是本意。 */
		if (NAV_GourdExit(&el, &er, GRAY_GetError(), GRAY_IsLost()))
		{
			/* 出口转向期间把光电的滤波/微分历史清掉：
			   否则松手那一瞬间它会拿着转弯前的旧误差猛纠一下（就是那一下顿）。 */
			s_err_f16   = 0;
			s_last_err  = 0;
			s_last_dir  = 1;         /* 出口是右转：把"丢线往哪找"的记忆种成右边，
			                            否则转完一旦丢线，会拿葫芦圈里留下的旧记忆
			                            往【左】找（实测就是出圈后那个直角弯往左转）。 */
			/* 注意：速度斜坡 s_base_ramp 故意【不复位】——
			   复位会让松手后从 0 重新加速，就是那一下停顿。 */
			Set_Car_Speed((int)el, (int)er);
			return;
		}
	}
	int base;
	int corr;
	int slow;
	int l;
	int r;

	/* ---- 0. 陀螺最近转向平均（约 200ms）----
	   "宽图案/丢线"时用它判断车【实际在往哪边转】，比上一拍的 PD 修正量可靠。 */
	s_w_acc += (int32_t)IMU_GetGyroZDps();
	s_w_cnt++;
	if (s_w_cnt >= 40)
	{
		s_w_avg = s_w_acc / s_w_cnt;
		s_w_acc = 0; s_w_cnt = 0;
	}

	/* ---- 1. 十字路口 ----
	   8 路同时看到"线的颜色" = 车正压在十字路口上，直行过去就行。
	   注意：这一条对白底黑线、黑底白线都成立 —— 因为在十字路口，
	   所有探头都会同时看到线色（黑十字看到黑、白十字看到白）。 */
	if (GRAY_IsAllOn())
	{
		/* 【8 路全亮】= 十字路口 或 直角弯顶点。
		   保持【陀螺最近 200ms 的实际转向方向】继续打：
		     十字路口   : 车本来就是直着走 -> 陀螺约 0 -> 直行通过
		     直角弯顶点 : 车正在转 -> 陀螺不为 0 -> 把弯带过去
		   顺便判一下"是不是十字"（给里程锚点用）：全亮 + 陀螺几乎不转 = 十字。 */
		int32_t c;
		int32_t b2 = g_base_speed;
		int32_t l2, r2;
		int32_t sb = NAV_SpeedBase();

		if (!NAV_InGourd() && s_w_avg < 25 && s_w_avg > -25)
		{
			NAV_CrossDetected();
		}

		c = s_w_avg * 12;
		if (c >  2000) c =  2000;
		if (c < -2000) c = -2000;

		/* 【关键】相切点那几拍经常是 8 路全亮，走的是这条分支 ——
		   窗口的屏蔽必须在这里也生效，否则"屏蔽右转"在最该起作用的时候空转。
		   这里 c>0 = 左转修正，c<0 = 右转修正。 */
		{
			uint8_t gm = NAV_GourdMask();

			if ((gm & 0x01u) && c < 0) { c = 0; g_mask_used++; }
			if ((gm & 0x02u) && c > 0) { c = 0; g_mask_used++; }
		}

		/* 【诊断·不改行为】角度到点了、但全亮挡着还没扳的那几拍。 */
		if (NAV_GourdPending()) g_flip_allon_cnt++;

		if (sb > 0) b2 = sb;
		b2 = NAV_GourdSlowTgt(b2);
		if (NAV_InGourd() && b2 > GOURD_TRACE_VMAX) b2 = GOURD_TRACE_VMAX;

		l2 = b2 - c;
		r2 = b2 + c;
		if (l2 > MOTOR_SPEED_MAX) l2 = MOTOR_SPEED_MAX;
		if (r2 > MOTOR_SPEED_MAX) r2 = MOTOR_SPEED_MAX;
		if (l2 < 0) l2 = 0;
		if (r2 < 0) r2 = 0;
		Set_Car_Speed((int)l2, (int)r2);
		s_last_err = 0;
		s_err_f16  = 0;
		return;
	}

	/* ---- 2. 取偏差 ----
	   基础速度：有路表就【背图变速】（直道冲快、弯道提前降速），
	   没表就还用固定的 g_base_speed。转向仍然是纯光电。 */
	{
		int32_t tgt = NAV_SpeedBase();

		if (tgt <= 0) tgt = g_base_speed;

		/* 葫芦圈窗口的"减速 buff"：目标速度先按百分比压下来，再交给下面的
		   斜率限制 —— 这样是平滑减速，不是瞬间掉速。 */
		tgt = NAV_GourdSlowTgt(tgt);

		/* 葫芦圈里压住速度：那段要在切点强制转向，快了拐不过来 */
		if (NAV_InGourd() && tgt > GOURD_TRACE_VMAX) tgt = GOURD_TRACE_VMAX;

		/* 斜率限制：升速每拍最多 +12、降速最多 -25（约 27k / 55k PWM 每秒），
		   把速度表的台阶磨成斜坡，车就不一顿一顿了。 */
		if (s_base_ramp == 0) s_base_ramp = tgt;
		if      (tgt > s_base_ramp) { s_base_ramp += g_ramp_up; if (s_base_ramp > tgt) s_base_ramp = tgt; }
		else if (tgt < s_base_ramp) { s_base_ramp -= g_ramp_dn; if (s_base_ramp < tgt) s_base_ramp = tgt; }
	}
	base = (int)s_base_ramp;

	if (GRAY_IsLost())
	{
		/* 丢线（8 路全没看到线 = 车已经跑出赛道）：朝最后一次有效方向
		   继续修正，同时降速。从没压到过线（s_last_dir==0）就停住 ——
		   不知道往哪找。 */
		if (s_last_dir == 0)
		{
			Set_Car_Speed(0, 0);
			return;
		}
		err  = s_last_dir * TRACE_LOST_ERR;
		base = base / TRACE_LOST_DIV;

		/* 把微分历史同步掉，否则丢线瞬间 Δerr 很大，D 项会炸 */
		s_last_err = err;
	}
	else
	{
		err = GRAY_GetError();

		/* 只记方向不记大小 —— 丢线时用方向就够，
		   用大小会让记忆量级随压线通道数乱跳 */
		if      (err < 0) s_last_dir = -1;
		else if (err > 0) s_last_dir =  1;
	}

	/* ---- 2.5 葫芦圈里程窗口：屏蔽某一侧转向 ----
	   窗口里 e>0 一律置 0（= 这个点不许往右拐）。相切点那两条对称分支
	   灰度选不出来，与其让它随机选，不如硬性禁掉错的那一边。 */
	{
		uint8_t gm = NAV_GourdMask();

		if ((gm & 0x01u) && err > 0) { err = 0; g_mask_used++; }
		if ((gm & 0x02u) && err < 0) { err = 0; g_mask_used++; }
	}

	/* ---- 3. 误差低通滤波 ----
	   把灰度的离散台阶磨成连续值，放在 P/D 之前让指令平滑。 */
	s_err_f16 += ((int32_t)err * 16 - s_err_f16) / ERR_FILT_N;
	err = (int)(s_err_f16 / 16);

	/* ---- 4. 弯道降速 ----
	   误差越大说明偏离越多（弯越急），适当降速，让同样的 Kp 能拐过来。
	   直道上 err≈0 不减速；弯道最多降 g_kslow*5。 */
	/* 【误差降速】以前是 base -= kslow*|err|。但弯道里 err 本身就在
	   ±30~50 地来回摆，等于速度被 ±1000 PWM 地抽 -> 车"一卡一卡"。
	   现在速度归【背图速度表】管，这条只保留"跑偏很多"时的保险：
	   误差在 ±25 以内完全不动作，超过才按 (|err|-25) 轻微降速。 */
	if (NAV_GourdSeg())
	{
		/* 葫芦圈段内（新里程状态机）：用带死区的弱降速。
		   老代码用的是 NAV_InGourd()（陀螺那套），那套关掉以后这里会退化成
		   圈外那种"无死区"降速 —— err 在蛇形里摆 ±40，速度被抽 ±640，看着忽快忽慢。 */
		if      (err >  25) slow = (g_kslow * (err - 25)) / 1000;
		else if (err < -25) slow = (g_kslow * (-err - 25)) / 1000;
		else                slow = 0;
	}
	else
	{
		/* 圈外：完全按老光电那一版 —— 误差多少就降多少（无死区）。
		   老版本弯里就是因为这条才贴得紧。 */
		slow = (g_kslow * ((err < 0) ? -err : err)) / 1000;
	}
	base -= slow;
	if (base < g_trace_vmin) base = g_trace_vmin;

	/* ---- 4. 转向环 PD ----
	   外面这个负号是关键：err<0（线在左）时修正量要为正，
	   下面 l = base - corr 才会让左轮减速。写成正号车会朝背离线的方向打舵。 */
	corr = -((g_kp * err + g_kd * (err - s_last_err)) / 1000);   /* ×1000 定点 */
	s_last_err = err;

	/* ---- 葫芦圈切点：强制打一下方向 ----
	   实物在相切点有两条对称分支，灰度质心选不出该跟哪条，所以按里程
	   强制转向 200ms，把车从"继续绕本圆"扳到"换下一个圆"。 */
	{
		int8_t f = NAV_GourdFlip((int16_t)err, GRAY_IsLost());

		if (f != 0)
		{
			corr = (f > 0) ? GOURD_FLIP_PWM : -GOURD_FLIP_PWM;
			s_last_err = 0;
		}
	}

	/* ---- 陀螺阻尼 ----
	   用陀螺测到的实际角速度压在转向指令上，抑制"转过头"。
	   和上面基于【误差微分】的 D 项相比，陀螺信号连续、干净，
	   不受灰度量化台阶的干扰 —— 这就是能让 Kp 推更高的关键。

	   符号：corr 正 = 左转。车正在左转时 ω>0，要减小左转 -> corr 减。
	        写成 corr -= k*ω。
	   g_kgyro = 0 时这一行完全没作用，不影响现有行为。 */
	/* 【陀螺阻尼只在葫芦圈里用】
	   它是"反着压舵"，当初是为了压住葫芦圈里的极限环抖动。
	   圈外用它会让车转不够、往外跑（实测"弯道偏外、不如老光电顺"）。
	   老光电那一版根本没有这一项 —— 圈外就完全不用。 */
	if (g_kgyro != 0 && NAV_InGourd())
	{
		corr -= (int)((g_kgyro * (int32_t)IMU_GetGyroZDps()) / 1000);
	}

	l = base - corr;
	r = base + corr;

	/* 限幅：只保留上限。
	   内轮【允许反转】—— 这样转弯角速度 ω = (v外-v内)/轮距 就不再被
	   "内轮最低为 0"卡住，急弯能转得更紧（这是提速的关键之一）。
	   下限由 MOTOR.c 的 Motor_Abs() 兜底到 ±MOTOR_SPEED_MAX。
	   三轮车有万向轮，内轮反转是安全的；万一出现甩尾就把 Kp 退回去。 */
	/* 限幅：外轮封顶，内轮【不许反转】。
	   为什么不让反转：err=50 时算出来 r = 2300 - 4000 = -1700，
	   内轮反转会给一个很猛的转向力矩。直道急弯确实转得更紧，
	   但【蛇形弯】是连续左右急弯，每次 |err|>35 内轮就反转 ->
	   车猛甩，再加上滤波滞后，直接甩飞丢线。实测踩过。
	   宁可转得"不够紧"，也不要过冲。 */
	if (l > MOTOR_SPEED_MAX) l = MOTOR_SPEED_MAX;
	if (r > MOTOR_SPEED_MAX) r = MOTOR_SPEED_MAX;
	if (l < 0) l = 0;
	if (r < 0) r = 0;

	Set_Car_Speed(l, r);
}

/* ============================================================
 * 控制节拍 —— 由 main 的主循环每圈调用一次
 *
 * 一轮做完三件事：采灰度、跑控制律、设 PWM。
 * 一圈约 450us，其中灰度采样占 400us（8 路 x 50us）。
 *
 * 【为什么不做成定时器中断】
 *   灰度采样一轮要 400us，塞进 1ms 中断的话控制周期最快也只能到 1ms，
 *   比放在主循环里慢一倍多。循迹要的是响应快，不是周期严格固定。
 *
 * 注意：GRAY_ReadAll() 里的 Delay_us() 是【轮询 SysTick】实现的。
 *      现在中断里不再用 Delay，所以主循环里用它没问题；
 *      但以后如果往定时器中断里加 Delay 调用，两边会抢 SysTick，
 *      主循环会卡死在 while 里。改代码时留意。
 * ============================================================ */
void Control_Tick(void)
{
	GRAY_ReadAll();       /* 无论什么模式都采，OLED 在哪种模式下都能看到 8 路状态 */
	ENCODER_Update();     /* 编码器里程（暂时没显示，留着给以后的速度环） */

#if USE_IMU
	/* 读陀螺（限速 5ms）。MPU6050 内部 200Hz，读太快是浪费 I2C 带宽。
	   放在控制律之前，让这一拍用的是最新角速度。 */
	if ((uint32_t)(g_ms_tick - s_imu_tick) >= 5u)
	{
		s_imu_tick = g_ms_tick;
		IMU_Update();

		/* 惯导记账：不管什么模式都在算（建图/跑图/光电模式都用同一份），
		   光电模式下也能从遥测看 X:Y:YW: 有没有道理。 */
		NAV_Update((int16_t)IMU_GetGyroZDps());
	}
#endif

	if      (g_mode == MODE_TRACE)  Trace_Run();
	else if (g_mode == MODE_MANUAL) Manual_Run();
	else if (g_mode == MODE_NAV)    NAV_Run();
	else                            Set_Car_Speed(0, 0);
}
