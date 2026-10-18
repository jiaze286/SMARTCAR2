#ifndef NAV_STUB_H
#define NAV_STUB_H
#include "stm32f10x.h"
extern volatile int g_nav_vmax;
extern volatile int g_nav_vmin;
extern volatile int g_nav_look_mm;
extern volatile int g_nav_map_vmax;
extern volatile int g_nav_corr_k;
extern volatile int g_nav_corr_on;
extern volatile int g_nav_corner_pct;
extern volatile int g_nav_straight_min;
extern volatile int g_nav_straight_deg10;
extern volatile int g_nav_mm_per_e;
int32_t NAV_LapSeq(void);
int32_t NAV_GetCrossTrack(void);
int32_t NAV_YawComp10(void);
#endif
