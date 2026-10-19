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
 *   R               -> OK            开跑（等同蓝牙 0x06；连不上换行也对）
 *   STOP            -> OK            急停（调参器自己不发送，留给人用）
 *   PING            -> OK            探活
 *   TIK             -> OK <n>        记一个里程打点（最多 64 笔）
 *   END             -> OK <n>        【锁车 + 静默 + 吐出全部打点】
 *                                     TIK,序号,t_ms,enc_mm,odo_mm,mode,8路,E
 *   RES             -> OK            恢复遥测（END 之后想继续看数据）
 *   CLR             -> OK            清空打点
 *   GLD [0|1]       -> OK <0|1>      葫芦圈陀螺逻辑开关（不带参数=只读）
 *   MOVE <mm>       -> OK n abs w a  当前里程窗口整体平移（-30 = 前移 30mm）
 *   WID <mm>        -> OK n abs w a  当前窗口全宽
 *   SEL <n>         -> OK n abs w a  选第 n 个窗口当"当前窗口"
 *   IN [mm]         -> OK <mm>       读/改葫芦圈进圈里程
 *   WIN [mm]        -> 逐行 WIN,i,abs,w,act,slow,seg + OK <n>（带数字 = 顺便设宽度）
 *   HELP            -> 一行列出所有命令
 *   FIX [e]         -> OK <e>        葫芦圈段固定打角（0=关；等价另一队的 -25）
 *   ACT [n]         -> OK i abs w act slow seg   当前窗口动作位
 *                      0=无 1=禁右转 2=禁左转 4=减速（可叠加，默认 4）
 *   SLW [pct]       -> OK <pct>      窗口内速度百分比（默认 40）
 *   GSL [pct]       -> OK <pct>      葫芦圈整段速度百分比（默认 70）
 *   WST [C]         -> WST,in,trig,ticks,minmm,maxmm,maskused   （C = 清空）
 *                      跑完读一次：进圈次数/窗口触发次数/窗口生效拍数/
 *                      窗口内里程范围/屏蔽真正削掉 err 的次数
 *
 * 【默认静默】上电后不再周期打印遥测。窗口相关的行：
 *   进圈      -> NOW IN
 *   进窗口    -> NOW MOVE <n> <mm>
 *   窗口期内  -> 串口不发（PC13 蓝灯常亮，避免 TXE 中断抢控制环时间）
 *   窗口外    -> 什么都不发
 *   想看回周期遥测就发 RES；发 END 会再次静默并锁车。
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

uint8_t TUNE_Quiet(void);      /* 1 = 现在不要插播遥测行（命令后的短窗口） */
uint8_t TUNE_Silent(void);     /* 1 = 彻底静默（发过 END：只吐打点，不报遥测） */
uint8_t TUNE_RunActive(void);  /* 1 = RUN 还没跑完 */
int32_t TUNE_Autorun(void);    /* 上电自动开跑开关（autorun 键，1=开） */

/* main.c 实现的两个动作：等价于蓝牙 0x06（开跑）/ 0x00（停） */
void App_StartRun(void);
void App_Stop(void);

#endif
