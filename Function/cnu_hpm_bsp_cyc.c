#include "cnu_hpm_bsp_cyc.h"
#include "hpm_csr_drv.h"

#define CPU_FREQ_Hz 648000000 // 648MHZ
#define CPU_FREQ_Hz_ms 648000
#define CPU_FREQ_Hz_us 648

DWT_Time_t SysTime;
static uint32_t CYCCNT_LAST = 0;
static uint32_t CYCCNT_RountCount = 0;
uint64_t CYCCNT64;

float GetDeltaT(uint32_t *cnt_last) {
  volatile uint32_t cnt_now = hpm_csr_get_core_mcycle();
  float dt = ((uint32_t)(cnt_now - *cnt_last)) / ((float)(CPU_FREQ_Hz));
  *cnt_last = cnt_now;

  return dt;
}

static void DWT_CNT_Update(void) {
  volatile uint32_t cnt_now = hpm_csr_get_core_mcycle();

  if (cnt_now < CYCCNT_LAST)
    CYCCNT_RountCount++;

  CYCCNT_LAST = cnt_now;
}

void DWT_SysTimeUpdate(void) {
  volatile uint32_t cnt_now = hpm_csr_get_core_mcycle();
  static uint64_t CNT_TEMP1, CNT_TEMP2, CNT_TEMP3;

  DWT_CNT_Update();

  CYCCNT64 = (uint64_t)CYCCNT_RountCount * (uint64_t)UINT32_MAX + (uint64_t)cnt_now;
  CNT_TEMP1 = CYCCNT64 / CPU_FREQ_Hz;
  CNT_TEMP2 = CYCCNT64 - CNT_TEMP1 * CPU_FREQ_Hz;
  SysTime.s = CNT_TEMP1;
  SysTime.ms = CNT_TEMP2 / CPU_FREQ_Hz_ms;
  CNT_TEMP3 = CNT_TEMP2 - SysTime.ms * CPU_FREQ_Hz_ms;
  SysTime.us = CNT_TEMP3 / CPU_FREQ_Hz_us;
}

float DWT_GetTimeline_s(void) {
  DWT_SysTimeUpdate();

  float DWT_Timelinef32 = SysTime.s + SysTime.ms * 0.001f + SysTime.us * 0.000001f;

  return DWT_Timelinef32;
}

void DWT_Delay(float Delay) {
  uint32_t tickstart = hpm_csr_get_core_mcycle();
  float wait = Delay;

  while ((hpm_csr_get_core_mcycle() - tickstart) < wait * (float)CPU_FREQ_Hz) {
  }
}
