#ifndef __USART1_H
#define __USART1_H

#include "stm32f10x.h"

/* ============================================================
 * USART1 —— 接 JDY-31 蓝牙模块
 *
 *   PA9  -> TX      PA10 -> RX
 *
 * 参数：9600 / 8 位 / 无校验 / 1 停止位（与遥控 APP 一致）
 *
 * 【发送是非阻塞的】字节先丢进环形缓冲，由 TXE 中断慢慢发。
 *   9600 波特率下一个字节要 1ms，如果阻塞式发送，回传一行 40 字节
 *   就会卡住主循环 40ms —— 控制循环直接废掉。所以必须走中断。
 *   缓冲满了就丢数据，绝不阻塞。
 * ============================================================ */

/* 【2026-10-19】9600 -> 115200：串口已经替代蓝牙（PA9/PA10 接 DAPLink 的
   CDC），主机那边普遍默认 115200，对不上就收不到数据、发的命令也认不出来。
   带宽 ×12 以后，遥测和 LAP 长行都不挤了。
   【注意】换回 JDY-31 蓝牙时，蓝牙模块和手机 APP 都得改回 9600。 */
#define USART1_BAUD_DEFAULT     115200

/* 失联判定：超过这个毫秒数没收到任何字节，就认为蓝牙断了 */
#define USART1_LINK_TIMEOUT_MS  500

/* 发送环形缓冲大小。9600 波特率下每毫秒才发 1 个字节，
   缓冲开大点可以吸收突发，代价只是 256 字节 RAM（共 20KB，够用） */
#define USART1_TX_BUF_SIZE      256

/* 文本行接收缓冲（SET/GET/RUN 协议）：中断只往里塞，拼行在主循环做。
   单字节指令那条路（RxData + g_rx_flag）完全不变，两套并存。 */
#define USART1_RX_BUF_SIZE      128

extern volatile uint8_t  RxData;          /* 最近一次收到的指令字节 */

/* 收到新字节置 1，主循环处理完清 0。
   为什么不用"RxData 变了才处理"：那样同一个指令连按两次只会执行一次，
   而建图/存表/开跑都是要反复按的。 */
extern volatile uint8_t  g_rx_flag;
extern volatile uint32_t g_last_rx_tick;  /* 最近一次收到字节时的 g_ms_tick */

void USART1_Init(uint32_t bound);

/* ---- 发送：全部非阻塞，丢进缓冲就返回 ---- */
void USART1_SendByte(uint8_t dat);
void USART1_SendString(const char *str);
void USART1_SendNum(int32_t num);
void USART1_SendMilli(int32_t v);       /* 定点 ×1000，打 "70.000" */          /* 带符号十进制，例：-12 */

/* 缓冲里还有多少字节没发出去（调试用） */
uint16_t USART1_TxPending(void);

/* ---- 失联检测 ----
 * 返回 1 表示蓝牙已失联。注意：上电后还没收到过任何数据时也算失联，
 * 这是刻意的 —— 没连手机就不该让车动。 */
uint8_t USART1_IsLost(void);

/* ---- 文本行接收（给调参器协议用）----
   中断只管往环形缓冲里塞；拼行、解析都在主循环里做。
   返回 1 = 取到一个字节，0 = 缓冲空了。 */
uint8_t USART1_RxChar(uint8_t *ch);

/* 清空接收缓冲（重新同步用，比如刚打开串口时） */
void    USART1_ResetRx(void);

/* 运行时改波特率：USART1 挂在 APB2=72MHz 上。
   9600 = 手机 APP/JDY-31；115200 = 调参器要快的时候。两边要一起改。 */
void    USART1_SetBaud(uint32_t bound);

#endif
