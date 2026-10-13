#include "mycontrol.h"
#include "GRAY.h"
#include "MOTOR.h"
#include "ENCODER.h"
#include "TIM1_TICK.h"
#include "IMU.h"

/* ---- 主循环里各函数共享的状态 ---- */
volatile uint8_t g_mode = MODE_STOP;
volatile uint8_t g_dir  = DIR_NONE;

/* ---- 可调参数 ---- */
volatile int g_base_speed = 3900;   /* 直道基础速度 */
volatile int g_kp         = 70;       /* 转向环 P。误差已放大 10 倍(GRAY_ERR_SCALE)，
                                        所以这里等价于以前的 260。往上调就能提速 */
volatile int g_kd         = 0;         /* 转向环 D，先关着 */
volatile int g_manual_fb  = 5000;   /* 手动前进/后退速度（占空比 5000/7199 = 69%）。
                                        只影响 0x01/0x02，循迹和转向不受影响。 */
volatile int g_kgyro      = 0;      /* 陀螺阻尼增益。0 = 关闭。
                                        建议从 30 试起，太大会抖。 */
volatile int g_kslow      = 32;     /* 弯道降速（误差放大 10 倍后的等价值）。
                                         转弯半径 R = (轮距/2)*(外+内)/(外-内)
                                         ks200 -> R=115mm，ks320 -> R=81mm */

/* ============================================================
 * 葫芦圈（相切圆环）处理
 *
 * 为什么需要：纯重心法在【相切点】分不清"直行"和"跟着圆转" ——
 * 车会一直绕着一个圆跑，永远出不去（实测现象）。
 *
 * 三步走（队友在同一赛道验证过思路，这里做了改进）：
 *   1. 进圈：陀螺【累计转角】判定。
 *      直道清零，只有持续转弯才累加。
 *      普通弯/直角弯约 90 度，葫芦圈 4 个圆 >1400 度 —— 干净区分。
 *      （队友原版用"短时角速度大"，但直角弯也满足，会误触发，所以改进）
 *   2. 出口：编码器【里程】判定。进圈那一刻记 0 点，跑够阈值就出。
 *      里程是物理量，与"看到什么图案"无关 ——
 *      直角弯的印记只能撑约 60mm，远不到 403mm，天生不会误触发。
 *   3. 出圈：右转固定角度，陀螺角度闭环。
 * ============================================================ */

/* ---- 可调参数 ---- */
volatile int g_gourd_enable    = 1;     /* 0 = 关闭葫芦圈处理 */
volatile int g_gourd_enter_deg = 360;   /* 累计转角阈值(度) -> 判定进圈 */
volatile int g_gourd_exit_mm   = 403;   /* 进圈后里程阈值(mm)。队友同赛道实测 403 */
volatile int g_gourd_turn_deg  = 85;    /* 出圈右转角度(度) */

/* 【出圈改成"从发车线起算的总里程"】
   原来的"进圈后再跑 403mm"不行：进圈判定时刻每次不一样（累计转角到
   360 度时车在圈里哪个位置是随机的），于是右转的点也是随机的，转完
   又咬回同一个圆 -> 无限绕圈。
   现在：车从发车线出发，总里程到这个数就右转一次（每次循迹只转一次）。
   要求：每次都从同一条发车线出发。
   怎么标定：把车放到发车线 -> 发 0x05 开始再立刻 0x05 停（里程清零）
            -> 用手沿赛道推到"该右转出圈"的那一点 -> 看 D: 两个数的平均。 */
volatile int g_gourd_exit_start_mm = 24621;   /* 之前实测在这里右转是对的；队友车 24847 */

#define GOURD_NORMAL      0u
#define GOURD_INSIDE      1u
#define GOURD_EXITING     2u
#define GOURD_DONE        3u   /* 本次循迹已出过圈，不再触发 */

#define GOURD_STRAIGHT_DPS   15     /* |角速度| 小于它算"直道" */
#define GOURD_STRAIGHT_RESET 60     /* 连续这么多拍(每拍5ms=300ms)直道 -> 累计转角清零 */
#define GOURD_TURN_PWM       2500   /* 出圈右转的轮速 */
#define GOURD_EXIT_TIMEOUT   2000u  /* 转向超时保护(ms)：转不够也强行结束 */

#define MANUAL_SPEED      3000      /* 蓝牙左转/右转的速度（0x03/0x04） */

/* 丢线时用的"假装偏差"，朝最后一次有效方向猛打一把，把车拉回线上 */
#define TRACE_LOST_ERR    50    /* 误差放大 10 倍，所以是 50 不是 5 */
/* 丢线时基础速度降到几分之一 */
#define TRACE_LOST_DIV    2
/* 降速后的速度下限，别降到停车 */
#define TRACE_MIN_SPEED   300

#if USE_IMU
static uint32_t s_imu_tick = 0;            /* 陀螺读取限速 */

/* 误差低通滤波（定点 x16 保精度）。
   灰度误差是【离散台阶】(-50/-20/0/20/50...)，直接乘 Kp 会让电机指令
   一跳一跳，车头就甩。滤波把它磨成连续值。
   ERR_FILT_N 越大越平滑、越滞后：8 约 4ms，16 约 7ms，32 约 14ms。 */
#define ERR_FILT_N   8     /* 16->8：滞后从 7ms 降到约 4ms，蛇形弯跟得上 */
static int32_t s_err_f16 = 0;        /* 滤波后的误差，放大 16 倍 */
#endif
volatile int     g_gourd_state_dbg = 0;
volatile int32_t g_gourd_turn_dbg  = 0;
static uint8_t  s_gourd_state = GOURD_NORMAL;
static int32_t  s_turn_acc10  = 0;   /* 累计|转角|，0.1度 —— 判进圈 */
static int32_t  s_yaw10       = 0;   /* 累计带符号航向，0.1度 —— 判出圈转向 */
static uint16_t s_straight_cnt = 0;
static int32_t  s_exit_yaw0   = 0;   /* 开始出圈时的航向 */
static uint32_t s_exit_tick   = 0;

static volatile int8_t  s_last_dir = 0;   /* 最后有效偏差方向：-1 左 / +1 右 / 0 未知 */
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
	s_gourd_state  = GOURD_NORMAL;
	s_turn_acc10   = 0;
	s_yaw10        = 0;
	s_straight_cnt = 0;
	ENCODER_Reset();      /* 编码器里程也清零（暂时没用到，留着给以后的速度环） */
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
 * 葫芦圈状态更新 —— 每 5ms 调一次（跟着陀螺读取走）
 *   角度增量 = ω(度/秒) x 0.005秒，再 x10 存成 0.1 度单位
 * ------------------------------------------------------------ */
static void Gourd_Update(int16_t w_dps)
{
	int32_t d10 = (int32_t)w_dps * 5 / 100;

	s_yaw10 += d10;                       /* 航向积分（带符号）*/

	if (w_dps < GOURD_STRAIGHT_DPS && w_dps > -GOURD_STRAIGHT_DPS)
	{
		/* 直道：连续够久就把累计转角清零 */
		if (s_straight_cnt < 60000u) s_straight_cnt++;
		if (s_straight_cnt >= GOURD_STRAIGHT_RESET) s_turn_acc10 = 0;
	}
	else
	{
		s_straight_cnt = 0;
		s_turn_acc10  += (d10 < 0) ? -d10 : d10;   /* 累加绝对值 */
	}

	/* 给遥测看的镜像量 */
	g_gourd_state_dbg = (int)s_gourd_state;
	g_gourd_turn_dbg  = s_turn_acc10;
}

/* ------------------------------------------------------------
 * 出圈右转 —— 陀螺角度闭环
 *   转够 g_gourd_turn_deg 就回正常循迹。右转 = 左轮前进 + 右轮后退。
 * ------------------------------------------------------------ */
static void Gourd_ExitRun(void)
{
	int32_t turned = s_yaw10 - s_exit_yaw0;

	if (turned < 0) turned = -turned;

	if (turned >= (int32_t)g_gourd_turn_deg * 10 ||
	    (uint32_t)(g_ms_tick - s_exit_tick) > GOURD_EXIT_TIMEOUT)
	{
		s_gourd_state = GOURD_DONE;
		g_gourd_state_dbg = 3;
		s_turn_acc10   = 0;               /* 清掉，免得立刻又判定进圈 */
		g_gourd_turn_dbg = 0;
		s_straight_cnt = 0;
		Set_Car_Speed(0, 0);
		return;
	}

	Set_Car_Speed(GOURD_TURN_PWM, -GOURD_TURN_PWM);   /* 左前右后 = 右转 */
}

/* ------------------------------------------------------------
 * 循迹模式
 * ------------------------------------------------------------ */
static void Trace_Run(void)
{
	int err;
	int base;
	int corr;
	int slow;
	int l;
	int r;

	/* ---- 0. 葫芦圈状态机 ----
	   出圈转向中 -> 完全接管电机；进圈后 -> 正常循迹但盯着里程。 */
	if (g_gourd_enable)
	{
		if (s_gourd_state == GOURD_EXITING)
		{
			Gourd_ExitRun();
			return;
		}

		/* 总里程到了 -> 右转出圈（只转一次）。
		   累计转角 s_turn_acc10 现在只做遥测参考，不再决定出圈。 */
		if (s_gourd_state == GOURD_NORMAL &&
		    (int32_t)(ENCODER_GetDist() * 1000.0f) >= (int32_t)g_gourd_exit_start_mm)
		{
			s_gourd_state = GOURD_EXITING;
			g_gourd_state_dbg = 2;
			s_exit_yaw0   = s_yaw10;
			s_exit_tick   = g_ms_tick;
			Gourd_ExitRun();
			return;
		}
	}

	/* ---- 1. 十字路口 ----
	   8 路同时看到"线的颜色" = 车正压在十字路口上，直行过去就行。
	   注意：这一条对白底黑线、黑底白线都成立 —— 因为在十字路口，
	   所有探头都会同时看到线色（黑十字看到黑、白十字看到白）。 */
	if (GRAY_IsAllOn())
	{
		Set_Car_Speed(g_base_speed, g_base_speed);
		s_last_err = 0;          /* 十字路口不参与微分，免得出来时抖一下 */
		s_err_f16  = 0;          /* 十字路口重置滤波 */
		return;
	}

	/* ---- 2. 取偏差 ---- */
	base = g_base_speed;

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
		base = g_base_speed / TRACE_LOST_DIV;

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

	/* ---- 3. 误差低通滤波 ----
	   把灰度的离散台阶磨成连续值，放在 P/D 之前让指令平滑。 */
	s_err_f16 += ((int32_t)err * 16 - s_err_f16) / ERR_FILT_N;
	err = (int)(s_err_f16 / 16);

	/* ---- 4. 弯道降速 ----
	   误差越大说明偏离越多（弯越急），适当降速，让同样的 Kp 能拐过来。
	   直道上 err≈0 不减速；弯道最多降 g_kslow*5。 */
	slow = g_kslow * ((err < 0) ? -err : err);
	base -= slow;
	if (base < TRACE_MIN_SPEED) base = TRACE_MIN_SPEED;

	/* ---- 4. 转向环 PD ----
	   外面这个负号是关键：err<0（线在左）时修正量要为正，
	   下面 l = base - corr 才会让左轮减速。写成正号车会朝背离线的方向打舵。 */
	corr = -(g_kp * err + g_kd * (err - s_last_err));
	s_last_err = err;

	/* ---- 陀螺阻尼 ----
	   用陀螺测到的实际角速度压在转向指令上，抑制"转过头"。
	   和上面基于【误差微分】的 D 项相比，陀螺信号连续、干净，
	   不受灰度量化台阶的干扰 —— 这就是能让 Kp 推更高的关键。

	   符号：corr 正 = 左转。车正在左转时 ω>0，要减小左转 -> corr 减。
	        写成 corr -= k*ω。
	   g_kgyro = 0 时这一行完全没作用，不影响现有行为。 */
	if (g_kgyro != 0)
	{
		corr -= (int)((float)g_kgyro * IMU_GetGyroZDps());
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
		Gourd_Update((int16_t)IMU_GetGyroZDps());   /* 葫芦圈状态更新，固定 5ms 一拍 */
	}
#endif

	if      (g_mode == MODE_TRACE)  Trace_Run();
	else if (g_mode == MODE_MANUAL) Manual_Run();
	else                            Set_Car_Speed(0, 0);
}
