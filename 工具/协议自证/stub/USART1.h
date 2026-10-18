#ifndef USART1_STUB_H
#define USART1_STUB_H
#include "stm32f10x.h"

void    USART1_SendByte(uint8_t dat);
void    USART1_SendString(const char *str);
void    USART1_SendNum(int32_t num);
void    USART1_SendMilli(int32_t v);
uint8_t USART1_RxChar(uint8_t *ch);

#endif
