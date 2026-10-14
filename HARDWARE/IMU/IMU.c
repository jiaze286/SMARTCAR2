#include "IMU.h"
#include "Delay.h"

/* MPU-6050 从机地址 0x68，左移一位后是 0xD0 */
#define IMU_ADDR_W   0xD0u
#define IMU_ADDR_R   0xD1u

/* 每个等待循环的超时计数。
   SPL 的硬件 I2C 在总线异常（比如模块没插好）时会【死等】，
   加超时是为了不把主循环卡死 —— 宁可跳一拍，不要雪崩。 */
#define I2C_TIMEOUT  5000u   /* 约 0.2ms。太长会把控制环拖死 */

/* --- MPU-6050 寄存器 --- */
#define REG_SMPLRT_DIV    0x19u
#define REG_CONFIG        0x1Au
#define REG_GYRO_CONFIG   0x1Bu
#define REG_ACCEL_CONFIG  0x1Cu
#define REG_GYRO_XOUT_H   0x43u
#define REG_PWR_MGMT_1    0x6Bu
#define REG_WHO_AM_I      0x75u

/* 常见兼容芯片的 WHO_AM_I。
   【重要】很多模块丝印写 MPU-6050，里面装的其实是 6500 —— 实测本例就是 0x70。
   这几款芯片的陀螺/加速度寄存器是兼容的，只有身份值不同，所以一起放行。 */
#define WHO_AM_I_MPU6050  0x68u
#define WHO_AM_I_MPU6500  0x70u
#define WHO_AM_I_MPU9250  0x71u
#define WHO_AM_I_MPU9255  0x73u

/* 陀螺 Z 轴数据在 0x43 + 4 = 0x47 */
#define REG_GYRO_ZOUT_H   (REG_GYRO_XOUT_H + 4u)

static uint8_t s_ready  = 0u;        /* 初始化成功才允许读，避免 I2C 超时拖累控制环 */
static float   s_bias_z = 0.0f;      /* 陀螺 Z 轴零偏，°/s */
static float   s_gz_dps = 0.0f;      /* 减零偏后的角速度，°/s */
static float   s_gx_dps = 0.0f;      /* X/Y 轴：只用于诊断"模块装歪了" */
static float   s_gy_dps = 0.0f;
static int16_t s_gz_raw = 0;

/* ============================================================
 * I2C 底层（全部带超时）
 * ============================================================ */
static void I2C_HW_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	I2C_InitTypeDef  I2C_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

	/* 重映射到 PB8/PB9（默认的 PB6/PB7 给了 TIM4 出 PWM） */
	GPIO_PinRemapConfig(GPIO_Remap_I2C1, ENABLE);

	/* 开漏复用输出。上拉靠模块自带的 4.7k，不用开内部上拉。 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_OwnAddress1         = 0x00;
	I2C_InitStructure.I2C_Ack                 = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed          = 400000;   /* 芯片确认正常（6500），恢复 400kHz */
	I2C_Init(I2C1, &I2C_InitStructure);

	I2C_Cmd(I2C1, ENABLE);
}

static uint8_t IMU_WriteReg(uint8_t reg, uint8_t val)
{
	volatile uint32_t t;

	I2C_AcknowledgeConfig(I2C1, ENABLE);

	I2C_GenerateSTART(I2C1, ENABLE);
	t = I2C_TIMEOUT;
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
		if (--t == 0u) return 1u;

	I2C_Send7bitAddress(I2C1, IMU_ADDR_W, I2C_Direction_Transmitter);
	t = I2C_TIMEOUT;
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
		if (--t == 0u) return 1u;

	I2C_SendData(I2C1, reg);
	t = I2C_TIMEOUT;
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTING) != SUCCESS)
		if (--t == 0u) return 1u;

	I2C_SendData(I2C1, val);
	t = I2C_TIMEOUT;
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
		if (--t == 0u) return 1u;

	I2C_GenerateSTOP(I2C1, ENABLE);
	return 0u;
}

static uint8_t IMU_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
	volatile uint32_t t;
	uint8_t i;

	I2C_AcknowledgeConfig(I2C1, ENABLE);

	I2C_GenerateSTART(I2C1, ENABLE);
	t = I2C_TIMEOUT;
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
		if (--t == 0u) return 1u;

	I2C_Send7bitAddress(I2C1, IMU_ADDR_W, I2C_Direction_Transmitter);
	t = I2C_TIMEOUT;
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
		if (--t == 0u) return 1u;

	I2C_SendData(I2C1, reg);
	t = I2C_TIMEOUT;
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED) != SUCCESS)
		if (--t == 0u) return 1u;

	/* 重复起始 + 切到读方向 */
	I2C_GenerateSTART(I2C1, ENABLE);
	t = I2C_TIMEOUT;
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
		if (--t == 0u) return 1u;

	I2C_Send7bitAddress(I2C1, IMU_ADDR_R, I2C_Direction_Receiver);
	t = I2C_TIMEOUT;
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED) != SUCCESS)
		if (--t == 0u) return 1u;

	for (i = 0u; i < len; i++)
	{
		/* 最后一个字节：先 NACK + STOP，再等事件把它读走 */
		if (i == (uint8_t)(len - 1u))
		{
			I2C_AcknowledgeConfig(I2C1, DISABLE);
			I2C_GenerateSTOP(I2C1, ENABLE);
		}

		t = I2C_TIMEOUT;
		while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED) != SUCCESS)
		{
			if (--t == 0u)
			{
				I2C_AcknowledgeConfig(I2C1, ENABLE);
				return 1u;
			}
		}

		buf[i] = I2C_ReceiveData(I2C1);
	}

	I2C_AcknowledgeConfig(I2C1, ENABLE);
	return 0u;
}

static uint8_t IMU_ReadReg(uint8_t reg, uint8_t *val)
{
	return IMU_ReadRegs(reg, val, 1u);
}

/* ============================================================
 * 初始化
 *   返回 IMU_OK / IMU_ERR_I2C / IMU_ERR_WHOAMI
 * ============================================================ */
uint8_t IMU_Init(void)
{
	uint8_t who = 0u;

	I2C_HW_Config();
	Delay_ms(20);                      /* 等模块上电稳定 */

	/* 先只读 WHO_AM_I —— 读不到就是接线/供电/重映射的问题，
	   这一步能最快区分"连线错"和"芯片不对" */
	if (IMU_ReadReg(REG_WHO_AM_I, &who) != 0u) return IMU_ERR_I2C;
	if (who != WHO_AM_I_MPU6050 && who != WHO_AM_I_MPU6500 &&
	    who != WHO_AM_I_MPU9250 && who != WHO_AM_I_MPU9255)
	{
		return IMU_ERR_WHOAMI;
	}

	/* 软复位 */
	IMU_WriteReg(REG_PWR_MGMT_1, 0x80u);
	Delay_ms(100);

	/* 唤醒 + 时钟源选 X 轴陀螺 PLL（比内部 RC 稳得多） */
	IMU_WriteReg(REG_PWR_MGMT_1, 0x01u);
	Delay_ms(20);

	/* 采样率 1kHz/(1+4) = 200Hz，配合 5ms 轮询刚好 */
	IMU_WriteReg(REG_SMPLRT_DIV, 0x04u);

	/* 数字低通：陀螺带宽 44Hz，能压掉不少电机振动噪声 */
	IMU_WriteReg(REG_CONFIG, 0x03u);

	/* 陀螺 ±2000dps（16.4 LSB 每 °/s），加速度 ±8g */
	IMU_WriteReg(REG_GYRO_CONFIG,  0x18u);
	IMU_WriteReg(REG_ACCEL_CONFIG, 0x10u);

	Delay_ms(50);                      /* 等第一次转换完成 */

	s_bias_z = 0.0f;
	s_gz_dps = 0.0f;
	s_gz_raw = 0;
	s_ready  = 1u;

	return IMU_OK;
}

/* 探测某个 7 位地址有没有应答（只发地址，不发数据） */
static uint8_t IMU_Probe(uint8_t addr7)
{
	volatile uint32_t t;

	I2C_GenerateSTART(I2C1, ENABLE);
	t = I2C_TIMEOUT;
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT) != SUCCESS)
		if (--t == 0u) { I2C_GenerateSTOP(I2C1, ENABLE); return 0u; }

	I2C_Send7bitAddress(I2C1, (uint8_t)(addr7 << 1), I2C_Direction_Transmitter);
	t = I2C_TIMEOUT;
	while (I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED) != SUCCESS)
		if (--t == 0u) { I2C_GenerateSTOP(I2C1, ENABLE); return 0u; }

	I2C_GenerateSTOP(I2C1, ENABLE);
	Delay_ms(1);
	return 1u;
}

/* 扫描常见 IMU 地址，返回第一个应答的 7 位地址 */
uint8_t IMU_ScanAddr(void)
{
	/* 0x68=MPU6050/6500/9250 默认，0x69=AD0 拉高，0x76/0x77=BMP 系 */
	static const uint8_t cand[4] = { 0x68u, 0x69u, 0x76u, 0x77u };
	uint8_t i;

	for (i = 0u; i < 4u; i++)
	{
		if (IMU_Probe(cand[i])) return cand[i];
	}
	return 0u;
}

uint8_t IMU_WhoAmI(void)
{
	uint8_t who = 0u;
	if (IMU_ReadReg(REG_WHO_AM_I, &who) != 0u) return 0u;
	return who;
}

/* ============================================================
 * 读陀螺（只读 Z 轴那 2 字节，省时间）
 * ============================================================ */
void IMU_Update(void)
{
	uint8_t buf[6];

	if (!s_ready) return;

	/* 一次把 X/Y/Z 三轴都读回来（0x43 起 6 个字节）。
	   X/Y 只用来诊断：如果转车时是 X 或 Y 在大动、Z 却几乎不动，
	   说明模块装歪了（Z 轴不是竖直的）。 */
	if (IMU_ReadRegs(REG_GYRO_XOUT_H, buf, 6u) != 0u)
		return;                        /* 读失败就保留上次的值，别清零 */

	s_gx_dps = (float)(int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1])
	           / IMU_GYRO_LSB_PER_DPS;
	s_gy_dps = (float)(int16_t)(((uint16_t)buf[2] << 8) | (uint16_t)buf[3])
	           / IMU_GYRO_LSB_PER_DPS;

	s_gz_raw = (int16_t)(((uint16_t)buf[4] << 8) | (uint16_t)buf[5]);
	s_gz_dps = ((float)s_gz_raw / IMU_GYRO_LSB_PER_DPS - s_bias_z)
	           * IMU_GYRO_Z_SIGN;
}

float   IMU_GetGyroZDps(void)  { return s_gz_dps; }
int16_t IMU_GetGyroZRaw(void)  { return s_gz_raw; }
float   IMU_GetGyroXDps(void)  { return s_gx_dps; }
float   IMU_GetGyroYDps(void)  { return s_gy_dps; }
float   IMU_GetGyroBiasZ(void) { return s_bias_z; }

/* ============================================================
 * 静止标定零偏
 *   车必须完全不动、电机不能转（振动会被当成零偏）。
 *   MPU6050 的零偏随温度漂，所以上电后尽快标定。
 * ============================================================ */
uint8_t IMU_GyroCalib(uint16_t samples)
{
	uint16_t i;
	int32_t  sum = 0;
	uint8_t  buf[2];
	int16_t  raw;

	if (samples == 0u) return 0u;

	for (i = 0u; i < samples; i++)
	{
		if (IMU_ReadRegs(REG_GYRO_ZOUT_H, buf, 2u) != 0u) return 0u;

		raw  = (int16_t)(((uint16_t)buf[0] << 8) | (uint16_t)buf[1]);
		sum += raw;

		Delay_ms(5);
	}

	s_bias_z = ((float)sum / (float)samples) / IMU_GYRO_LSB_PER_DPS;
	return 1u;
}
