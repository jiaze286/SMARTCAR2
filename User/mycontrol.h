#ifndef __MYCONTROL_H
#define __MYCONTROL_H

#include "stm32f10x.h"

/* ============================================================
 * 车辆控制 —— 在主循环里执行，一圈约 450us（灰度采样占 400us）
 *
 * 转向环（PD）+ 弯道降速：
 *     速度   = g_base_speed - g_kslow * |err|
 *     修正量 = -( g_kp * err + g_kd * Δerr )
 *     左轮   = 速度 - 修正量
 *     右轮   = 速度 + 修正量
 *
 * 符号约定：
 *     err < 0  线在车的【左边】 -> 修正量为正 -> 左轮减速 -> 左转
 *     err > 0  线在车的【右边】 -> 修正量为负 -> 左轮加速 -> 右转
 *
 * 【重要】Set_Car_Speed() 只允许被 Control_Tick() 调用，
 *        别的地方（包括中断）都不许碰电机。
 * ============================================================ */

/* ---- 工作模式 ---- */
#define MODE_STOP     0
#define MODE_MANUAL   1
#define MODE_TRACE    2
#define MODE_MAP      3      /* 建图：电机自由，人推着走，只记账 */
#define MODE_NAV      4      /* 惯导跑：按路表开 */

/* ---- 手动方向 ---- */
#define DIR_NONE      0
#define DIR_FWD       1
#define DIR_BACK      2
#define DIR_LEFT      3
#define DIR_RIGHT     4

/* ---- 主循环里各函数共享的状态 ---- */
extern volatile uint8_t g_mode;
extern volatile uint8_t g_dir;

/* ---- 可调参数（改完重新编译）---- */
extern volatile int g_base_speed;   /* 直道基础速度，建议 1500~3500 */
extern volatile int g_kp;           /* 转向环 P，建议 100~400 */
extern volatile int g_kd;           /* 转向环 D，先保持 0；
                                       直道轻微画龙时试 30~80 */
extern volatile int g_kgyro;       /* 【陀螺阻尼】增益。用陀螺测到的角速度做转向阻尼，
                                       替代（或补充）原来对量化误差求微分的 D 项 ——
                                       陀螺信号干净得多。
                                       0 = 关闭（默认，先验证陀螺再接进来）。
                                       建议从 30 试起，太大会抖。 */
extern volatile int g_manual_fb;    /* 【手动前进/后退】的速度。只影响蓝牙的
                                       0x01/0x02，循迹和转向都不受影响。
                                       用来试电机的最大安全速度。
                                       上限 MOTOR_SPEED_MAX = 7199（满占空比） */
extern volatile int g_kslow;        /* 弯道降速系数：每 1 点误差降多少速度。
                                       弯道偏外就加大（试 100~250）；
                                       直道太慢就减小 */

/* 控制节拍：采灰度 -> 控制律 -> 设 PWM。
   由 main 的主循环每圈调用一次（约 450us 一轮）。 */
void Control_Tick(void);

/* 上电初始化一次（内部会初始化灰度模块） */
void Trace_Init(void);

/* 每次【启动循迹】时调用：清掉上一次的方向记忆，
   否则会沿用旧方向，一启动就往错的方向拐 */
void Trace_Reset(void);

#endif
