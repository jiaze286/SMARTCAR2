#include "stm32f10x.h"
#include "Delay.h"
#include "LED.h"
#include "TIM4_PWM.h"
#include "TIM1_TICK.h"
#include "MOTOR.h"
#include "GRAY.h"
#include "USART1.h"
#include "mycontrol.h"
#include "ENCODER.h"
#include "IMU.h"

/* ============================================================
 *  循迹小车主程序
 *
 *  硬件：STM32F103C8T6 + TB6612 + 2×JGA25-370 + 8路灰度 + MPU6050 + JDY-31
 *
 *  【没有 OLED】
 *     所有调试信息走蓝牙串口。好处：控制环不再被刷屏打断（原来整屏
 *     5~8ms、分页 0.64ms），PA11/PA12 两个脚也空出来了，全部资源留给控制。
 *     坏处：车跑起来时串口数据是滚动的，要事后看就得录下来。
 *
 *  【架构】控制跑在主循环里，一圈约 450us（就是灰度采样的 400us）。
 *  电机只由 Control_Tick() 驱动，别的地方不许碰 Set_Car_Speed。
 *
 *  蓝牙指令（与遥控 APP 一致）：
 *      0x00 停止          0x01 前进        0x02 后退
 *      0x03 左转          0x04 右转
 *      0x05 循迹 暂停/继续  0x06 启动循迹
 * ============================================================ */

/* 数据行周期。9600 波特率下每毫秒 1 字节，一行约 60 字节 = 63ms。
   100ms（10Hz）占约 63% 带宽。
   嫌挤就把 TELEM_PERIOD_MS 调大，或者把 D: 那段（里程）删掉。
   想更快就把 JDY-31 和手机 APP 一起改到 115200（速度提 12 倍）。 */
/* ------------------------------------------------------------
 * 【调试开关】上电自动开始循迹
 *   1 = 不用连蓝牙，一上电就进循迹模式（排故用）
 *   0 = 正常（等蓝牙 0x06 指令）
 *
 * 【注意】置 1 时一上电车就会跑，要先把车摆到线上再上电。
 *        正式比赛前必须改回 0。
 * ------------------------------------------------------------ */
/* ------------------------------------------------------------
 * 【调试开关】上电电机自检
 *   1 = 上电后先跑一遍电机测试，再进主循环
 *       顺序：前进 -> 停 -> 后退 -> 停 -> 左转 -> 停 -> 右转 -> 停
 *       LED 会先闪 1/2/3/4 下，告诉你接下来是哪个方向
 *   0 = 正常
 *
 * 【注意】车轮必须架空！自检期间不读灰度、不看蓝牙。
 *        自检模式下把 AUTO_TRACE 设成 0（否则自检完就开始循迹）。
 * ------------------------------------------------------------ */
#define AUTO_MOTOR_TEST   0
/* 自检用的轮速，别用 g_manual_fb（那个 5000 太快，台架上危险） */
#define MOTOR_TEST_SPEED  3000

#define AUTO_TRACE   0

#define TELEM_PERIOD_MS     100

/* 状态行兜底周期：即使参数没变，也每 3 秒重发一次，
   免得刚上电错过那一行就再也看不到 */
#define STATUS_PERIOD_MS    3000

static uint8_t  s_last_cmd    = 0xFF;  /* 0xFF = 上电后还没收到过任何指令 */
static uint32_t s_telem_tick  = 0;
static uint32_t s_status_tick = 0;
static uint8_t  s_imu_status  = 0xFF;  /* 0=OK, 1=I2C错, 2=芯片不对 */
static uint8_t  s_imu_who     = 0;     /* 读到的 WHO_AM_I 原值 */
static uint8_t  s_imu_addr    = 0;     /* 扫描到的 I2C 地址（0=没扫到） */

/* 上次发出去的状态，用来判断有没有变化 */
static uint8_t  s_st_mode = 0xFF, s_st_imu = 0xFF;
static int      s_st_p = -1, s_st_s = -1, s_st_b = -1, s_st_d = -1, s_st_k = -1;

/* ============================================================
 * 数据行（每 TELEM_PERIOD_MS 一行）
 *
 *   M:2 E:-2 G:00111100 D:123456,123456 V:12,13 W:12 L:0 A:0
 *
 *   M  当前模式    0=停止 1=手动 2=循迹
 *   E  加权重心偏差（已放大 GRAY_ERR_SCALE 倍）
 *   G  8 路灰度状态，左到右
 *   D  左右轮累计里程，单位 mm
 *   V  左右轮本周期计数差（就是轮速）
 *   W  陀螺 Z 轴角速度，°/s
 *   L  1 = 丢线
 *   A  1 = 8 路全压线（十字路口）
 * ============================================================ */
static void Send_DataLine(void)
{
	uint8_t i;

	USART1_SendString("M:");
	USART1_SendNum((int32_t)g_mode);

	USART1_SendString(" E:");
	USART1_SendNum(GRAY_GetError());

	USART1_SendString(" G:");
	for (i = 0; i < GRAY_CH_NUM; i++)
	{
		USART1_SendByte(GRAY_GetChannel(i) ? '1' : '0');
	}

	USART1_SendString(" D:");
	USART1_SendNum((int32_t)(ENCODER_GetDistL() * 1000.0f));
	USART1_SendByte(',');
	USART1_SendNum((int32_t)(ENCODER_GetDistR() * 1000.0f));

	USART1_SendString(" V:");
	USART1_SendNum(ENCODER_GetDeltaL());
	USART1_SendByte(',');
	USART1_SendNum(ENCODER_GetDeltaR());

	USART1_SendString(" W:");
	USART1_SendNum((int32_t)IMU_GetGyroZDps());

	USART1_SendString(" L:");
	USART1_SendNum(GRAY_IsLost() ? 1 : 0);

	USART1_SendString(" A:");
	USART1_SendNum(GRAY_IsAllOn() ? 1 : 0);

	USART1_SendString("\r\n");
}

/* ============================================================
 * 状态行（参数/模式/IMU 状态一变就发；另外每 STATUS_PERIOD_MS 兜底一次）
 *
 *   S:P125 S32 B3900 D400 K0 I0 ID:104
 *     ID = 读到的 WHO_AM_I 原值（104 = 0x68 才是真 MPU-6050）
 *     AD = 扫描到的 I2C 地址（104=0x68 / 105=0x69 / 0=一个都没应答）
 *
 *   P=kp  S=kslow  B=base_speed  D=kd  K=kgyro  I=IMU状态
 *   （M=模式在数据行的开头）
 * ============================================================ */
static void Send_StatusLine(void)
{
	USART1_SendString("S:P"); USART1_SendNum(g_kp);
	USART1_SendString(" S");  USART1_SendNum(g_kslow);
	USART1_SendString(" B");  USART1_SendNum(g_base_speed);
	USART1_SendString(" D");  USART1_SendNum(g_kd);
	USART1_SendString(" K");  USART1_SendNum(g_kgyro);
	USART1_SendString(" I");  USART1_SendNum((int32_t)s_imu_status);
	USART1_SendString(" ID:"); USART1_SendNum((int32_t)s_imu_who);
	USART1_SendString(" AD:"); USART1_SendNum((int32_t)s_imu_addr);
	USART1_SendString("\r\n");

	s_st_mode = g_mode;   s_st_imu = s_imu_status;
	s_st_p = g_kp;  s_st_s = g_kslow;  s_st_b = g_base_speed;
	s_st_d = g_kd;  s_st_k = g_kgyro;
}

/* 参数/模式有变化就发状态行 */
static void Status_Changed(void)
{
	if (g_mode != s_st_mode || s_imu_status != s_st_imu ||
	    g_kp != s_st_p || g_kslow != s_st_s || g_base_speed != s_st_b ||
	    g_kd != s_st_d || g_kgyro != s_st_k)
	{
		Send_StatusLine();
	}
}

/* ============================================================
 * 上电电机自检
 *   每个方向跑 1 秒，中间停 0.5 秒。LED 闪 N 下预告第 N 个动作。
 *   车轮必须架空！
 * ============================================================ */
#if AUTO_MOTOR_TEST
static void Motor_SelfTest(void)
{
	int i, k;

	for (i = 0; i < 4; i++)
	{
		Set_Car_Speed(0, 0);
		for (k = 0; k <= i; k++)
		{
			LED_Open();  Delay_ms(150);
			LED_Close(); Delay_ms(150);
		}
		Delay_ms(300);

		switch (i)
		{
			case 0: Set_Car_Speed( MOTOR_TEST_SPEED,  MOTOR_TEST_SPEED); break; /* 前进 */
			case 1: Set_Car_Speed(-MOTOR_TEST_SPEED, -MOTOR_TEST_SPEED); break; /* 后退 */
			case 2: Set_Car_Speed(-MOTOR_TEST_SPEED,  MOTOR_TEST_SPEED); break; /* 左转 */
			case 3: Set_Car_Speed( MOTOR_TEST_SPEED, -MOTOR_TEST_SPEED); break; /* 右转 */
			default: break;
		}

		Delay_ms(1000);
		Set_Car_Speed(0, 0);
		Delay_ms(500);
	}

	LED_Open();  Delay_ms(500);
	LED_Close();
}
#endif

/* ============================================================ */
int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	/* ---- 底层先起来 ---- */
	MOTOR_GPIO_Init();                /* 方向脚 + STBY 拉高（并先刹车） */
	TIM4_PWM_Init(TIM4_PWM_ARR, 1);   /* 5kHz 双路 PWM，CH1=PB6 左 CH2=PB7 右 */
	TIM1_TICK_Init();                 /* 1ms 时基（只用于计时，不跑控制） */
	LED_Init();

	/* ---- 灰度 ---- */
	Trace_Init();

	/* ---- 编码器（里程计）---- */
	ENCODER_Init();

	/* ---- MPU-6050 ----
	   USE_IMU = 0 时整段跳过（不初始化、不占启动时间）。 */
#if USE_IMU
	s_imu_addr   = IMU_ScanAddr();   /* 先扫总线，看芯片在不在、在哪 */
	s_imu_who    = IMU_WhoAmI();     /* 先记下来，错了才知道读到的是什么 */
	s_imu_status = IMU_Init();
	if (s_imu_status == IMU_OK)
	{
		/* 静止标定零偏：200 次 x 5ms 约 1 秒。这段时间车必须不动！ */
		IMU_GyroCalib(200);
	}
#else
	s_imu_status = 0xFE;      /* 0xFE = 本次编译没启用 IMU */
#endif

	/* ---- 蓝牙放最后初始化 ----
	   上电瞬间串口线上可能有杂波，先让电机侧处于刹车状态再开串口 */
	USART1_Init(USART1_BAUD_DEFAULT);
	g_last_rx_tick = g_ms_tick;       /* 失联计时从此刻起算 */

	/* 上电先报一次状态，方便确认参数和 IMU 情况 */
	Send_StatusLine();

#if AUTO_MOTOR_TEST
	/* 上电电机自检。车轮必须架空！ */
	Motor_SelfTest();
#endif

#if AUTO_TRACE
	/* 调试模式：上电直接进循迹，不等蓝牙指令。
	   注意上电时灰度如果全没压线（s_last_dir==0），车会停住不动 ——
	   这是保护，所以【必须先把车摆到线上再上电】。 */
	Trace_Reset();
	g_mode = MODE_TRACE;
	g_dir  = DIR_NONE;
#endif

	while (1)
	{
		/* ---- 1. 处理蓝牙指令（只在字节变化时执行一次）---- */
		if (RxData != s_last_cmd)
		{
			s_last_cmd = RxData;

			switch (s_last_cmd)
			{
				case 0x00:                       /* 停止 */
					g_mode = MODE_STOP;
					g_dir  = DIR_NONE;
					break;

				case 0x01: g_mode = MODE_MANUAL; g_dir = DIR_FWD;   break;
				case 0x02: g_mode = MODE_MANUAL; g_dir = DIR_BACK;  break;
				case 0x03: g_mode = MODE_MANUAL; g_dir = DIR_LEFT;  break;
				case 0x04: g_mode = MODE_MANUAL; g_dir = DIR_RIGHT; break;

				case 0x05:                       /* 循迹 暂停 / 继续 */
					if (g_mode == MODE_TRACE)
					{
						g_mode = MODE_STOP;
						g_dir  = DIR_NONE;
					}
					else
					{
						Trace_Reset();           /* 别沿用上次的丢线记忆 */
						g_mode = MODE_TRACE;
						g_dir  = DIR_NONE;
					}
					break;

				case 0x06:                       /* 启动循迹 */
					Trace_Reset();
					g_mode = MODE_TRACE;
					g_dir  = DIR_NONE;
					break;

				default:
					break;                       /* 未知指令忽略 */
			}
		}

		/* ---- 2. 失联保护（只对手动模式生效）----
		   手动模式靠遥控开着，人一走车不能保持最后一条指令继续冲。
		   循迹模式【不检查】—— 车是自己沿赛道跑的，比赛时手机掉线很正常，
		   这里要是也停车，一发完 0x06 车就会在 0.5 秒后自己停。 */
		if (g_mode == MODE_MANUAL && USART1_IsLost())
		{
			g_mode = MODE_STOP;
			g_dir  = DIR_NONE;
		}

		/* ---- 3. 控制：采灰度 -> 陀螺 -> 控制律 -> 设 PWM ----
		   主循环一圈约 450us。这是整块代码最关键的一步，
		   别在它前面塞耗时的东西。 */
		Control_Tick();

		/* ---- 4. 状态灯：亮 = 蓝牙失联 ---- */
		if (USART1_IsLost()) LED_Open();
		else                 LED_Close();

		/* ---- 5. 串口遥测（非阻塞发送，不会卡控制）----
		   【注意】这里绝对不能加 "!USART1_IsLost()" 之类的门！
		   失联判定的意思是"500ms 没收到过字节"，刚连上还没发指令时它是 true，
		   加上门就变成"你不先说话我就不吭声" —— 监听不到任何数据。
		   发送是环形缓冲 + 中断，满了自己丢，不会拖累控制，放心一直发。 */
		if ((uint32_t)(g_ms_tick - s_telem_tick) >= TELEM_PERIOD_MS)
		{
			s_telem_tick = g_ms_tick;
			Send_DataLine();
		}

		/* 参数/模式/IMU 状态一变就补一行状态；另外每 STATUS_PERIOD_MS
		   兜底发一次，免得刚上电那一行错过了就再也看不到。 */
		if ((uint32_t)(g_ms_tick - s_status_tick) >= STATUS_PERIOD_MS)
		{
			s_status_tick = g_ms_tick;
			Send_StatusLine();
		}
		else
		{
			Status_Changed();
		}
	}
}
