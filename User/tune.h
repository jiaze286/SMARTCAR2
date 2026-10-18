#ifndef __TUNE_H
#define __TUNE_H

#include "stm32f10x.h"

/* ============================================================
 * 调参串口协议 + 每圈 LAP 统计（给 工具/调参迭代.py 用）
 *
 * 【协议】ASCII 行，LF 结尾（\r 可有可无）：
 *   SET <键> <值>   -> OK            值可以是 70 / 68.375 / 4e-06
 *   GET <键>        -> OK <值>       回读值和写进去的相对误差 <= 1e-3
 *   RUN <n>         -> ACK           随后每跑完一圈打一行 LAP，跑满 n 圈自动停车
 *   STOP            -> OK            急停（调参器自己不发送，留给人用）
 *   PING            -> OK            探活
 *
 * 【为什么命令期间要压住遥测】调参器每发一条命令之前都会清空接收缓冲，
 * 而且只把"下一行"当回应。命令/回复期间插播遥测行，回读校验就会失败，
 * 现象是 PUSHFAIL。见 TUNE_Quiet()：主循环的遥测/状态行按它让路。
 *
 * 【单位约定】表里所有值都按工程单位 ×1000 存（定点），这样
 *   ① GET 能原样回读（float 脚本发 68.375，回 "68.375"，误差 0）
 *   ② 控制里用整数除法还原，不用软浮点
 *   举例：kp_steer 70 存 70000，vmax 3200 存 3200000，corr_enc_k 1.0 存 1000
 *
 * 【LAP 13 字段】顺序固定、逗号分隔（调参器不足 13 段整行丢弃）：
 *   LAP,圈号,跑完0|1,耗时ms,里程mm,线RMS,线P95,线MAX,惯导RMS,惯导P95,
 *       补偿角均值deg,脱线次数,结束原因
 * ============================================================ */

#define TUNE_LINE_MAX        80      /* 一条命令最长多少字节 */
#define TUNE_QUIET_MS        1500u   /* 一条命令之后压住遥测多久(ms) */
#define TUNE_LAP_TIMEOUT_MS  30000u  /* 一圈最多给多久；超时 -> 停车并报 TIMEOUT */

void    TUNE_Init(void);

/* 主循环里每圈调一次：收字节、拼行、执行 SET/GET/RUN */
void    TUNE_Poll(void);

/* 控制拍里调一次（跟在 Control_Tick 后面）：累计这一圈的统计量 */
void    TUNE_Tick(void);

uint8_t TUNE_Quiet(void);      /* 1 = 现在不要插播遥测行 */
uint8_t TUNE_RunActive(void);  /* 1 = RUN 还没跑完 */
int32_t TUNE_Autorun(void);    /* 上电自动开跑开关（autorun 键，1=开） */

/* main.c 实现的两个动作：等价于蓝牙 0x06（开跑）/ 0x00（停） */
void App_StartRun(void);
void App_Stop(void);

#endif
