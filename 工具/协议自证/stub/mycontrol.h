#ifndef MYCONTROL_STUB_H
#define MYCONTROL_STUB_H
#include "stm32f10x.h"
extern volatile int32_t g_kp;
extern volatile int32_t g_kd;
extern volatile int32_t g_kgyro;
extern volatile int32_t g_kslow;
extern volatile int32_t g_ramp_up;
extern volatile int32_t g_ramp_dn;
extern volatile int32_t g_trace_vmin;
extern volatile int g_base_speed;
#endif
