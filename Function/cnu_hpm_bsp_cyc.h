#ifndef CNU_HPM_BSP_CYC_H
#define CNU_HPM_BSP_CYC_H

#include "stdint.h"

typedef struct
{
  uint32_t s;
  uint16_t ms;
  uint16_t us;
} DWT_Time_t;

extern float GetDeltaT(uint32_t *cnt_last);
extern void DWT_SysTimeUpdate(void);
extern float DWT_GetTimeline_s(void);
extern void DWT_Delay(float Delay);
#endif