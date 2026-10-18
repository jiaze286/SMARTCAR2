#ifndef ENCODER_STUB_H
#define ENCODER_STUB_H
#include "stm32f10x.h"
extern volatile int32_t g_enc_k;
float ENCODER_GetDist(void);
#endif
