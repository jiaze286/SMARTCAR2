#ifndef __NAV_H
#define __NAV_H

#include "stm32f10x.h"

/* ============================================================
 * 惯性导航（惯导为主、光电为辅）—— 两圈架构
 *
 * 【第一圈 建图】
 *   人手推着走（电机自由），编码器给 ds、陀螺给 omega，
 *   推算出自已走过的 (x, y, yaw)，每走 16mm 在路表上记一个点。
 *   走完存进片内 Flash，断电不丢。
 *
 * 【第二圈 惯导跑】
 *   按编码器里程在路表上取【前瞻点】，把惯导当成车来打方向
 *   （纯跟踪）。光电只在【长直道中段】用来修陀螺的航向漂移。
 *
 * 【为什么这样能过葫芦圈 / 十字 / 直角弯】
 *   路表里记的就是"该走的路径"。相切点该跟哪条线、十字该不该直行，
 *   全都不需要任何判断 —— 不需要找标志，也没有专门的元素逻辑。
 *
 * 【坐标与角度约定】
 *   起点 (0,0)，航向 yaw 逆时针为正（左转为正），单位 0.1 度。
 *   横向偏移【左正右负】—— 跟光电 E: 反号（E: 正 = 线在右边），
 *   换算的地方注意符号。
 * ============================================================ */

/* ---- 路表参数 ---- */
#define NAV_DS_MM        20          /* 采样间隔(mm)：每走这么远记一个点 */
#define NAV_N_MAX        2200        /* 最多点数（2200 x 20mm = 44m，队友地图量单圈 43.43m）*/
#define NAV_FLASH_ADDR   0x0800C000u /* 路表存在这里（片内 48KB 处起，留 16KB）*/

/* ---- 可调参数（改完重新编译）---- */
extern volatile int g_nav_vmax;      /* 惯导跑：直道速度（PWM） */
extern volatile int g_nav_vmin;      /* 惯导跑：最急弯的速度（PWM） */
extern volatile int g_nav_look_mm;   /* 前瞻距离(mm)：大了稳，小了跟得紧 */
extern volatile int g_nav_kgyro;     /* 陀螺阻尼增益（比光电模式该大一点） */
extern volatile int g_nav_corr_div;  /* 直道光电修航向的强度：越大越慢 */
extern volatile int g_nav_mm_per_e;  /* 光电 E: 1 点约等于多少 mm（现场标定） */

/* 【发车点偏置】单位 mm。
 * 内置路表的 0 点是地图上的"S、G 两条横线正中间"。
 * 如果你的发车点比它更靠西（前）多少毫米，就填多少 —— 相当于把整张表
 * 往后挪这么多。填错的表现：车到弯道时表还在说直走（提前/滞后冲出去）。
 * 注意：偏置必须落在直道上（方向相同），否则起始朝向会对不上。 */
extern volatile int g_nav_start_off_mm;

/* 内置路表（队友 CAD 地图生成，20mm 间隔，37.10m）——
   上电时如果 Flash 里没有自己推的路表，就用这张 */
extern const uint16_t g_nav_map_n;
extern const int16_t  g_nav_map_x[];
extern const int16_t  g_nav_map_y[];

void     NAV_Init(void);

/* 每 5 ms 调一次（跟陀螺读取同步）：积分航向/位置，建图模式下顺便记点 */
void     NAV_Update(int16_t w_dps);

/* 惯导跑的控制拍（由 Control_Tick 里 MODE_NAV 时调用） */
void     NAV_Run(void);

/* 建图：开始 / 结束（结束时算直道分类） */
void     NAV_StartMap(void);
void     NAV_StopMap(void);
uint8_t  NAV_IsMapping(void);

/* 开始惯导跑：清里程/位置/航向（路表保留） */
void     NAV_StartRun(void);

/* 存进 Flash / 上电读出来。返回 1 = 成功 / 点数(0 = 没有表) */
uint8_t  NAV_Store(void);
uint16_t NAV_Restore(void);

/* 选表：Flash 里有自己推的就用推的，否则用内置 CAD 表。返回 1=Flash表 2=内置表 */
uint8_t  NAV_LoadMap(void);

/* ---- 遥测用 ---- */
int32_t  NAV_Xmm(void);              /* 当前位置 x，mm（相对起点） */
int32_t  NAV_Ymm(void);              /* 当前位置 y，mm */
int32_t  NAV_Yaw10(void);            /* 陀螺积出来的航向，0.1 度 */
int32_t  NAV_UpdCnt(void);           /* 惯导更新次数（上电起算，诊断用）*/
int32_t  NAV_GetCrossTrack(void);    /* 车相对路表的横向偏移 mm（左正，诊断用）*/
int32_t  NAV_GetLostBrake(void);     /* 1 = 保命刹停触发过（诊断）*/
int32_t  NAV_GetPwm(void);           /* 这一拍下发的平均速度（诊断）*/
int32_t  NAV_YawComp10(void);        /* 光电修出来的航向补偿角，0.1 度 */
int32_t  NAV_Idx(void);              /* 当前在路表上的下标 */
int32_t  NAV_LastErr(void);          /* 上一拍换算到 E: 量纲的转向误差 */
uint16_t NAV_Count(void);            /* 路表点数 */

#endif
