#ifndef DETECT_TASK_H
#define DETECT_TASK_H

#include "board.h"

#define DETECT_TASK_INIT_TIME 50
#define DETECT_CONTROL_TIME 10

// 错误码以及对应设备顺序
enum Detect_List {
#if !BOARD_RUNNING_CORE
  My_dualcore = 0, // 双核
  My_referee,      // 裁判系统
  My_computer,     // 电脑通信

#else
  My_dualcore = 0, // 双核
  My_remote,       // 遥控器
  My_vtm,          // 图传
  My_super,        // 超电
  My_shootl,       // 左摩擦轮
  My_shootr,       // 右摩擦轮
  My_tri2006,      // 拨弹2006
  My_pitch,        // pitch
  My_drive1,       // 驱动1
  My_drive2,       // 驱动2
  My_drive3,       // 驱动2
  My_drive4,       // 驱动4
  My_yaw,          // yaw
  My_tri3508,      // 拨弹3508
#endif
  DETECT_ERROR_LIST_LENGHT,
};

typedef struct
{
  uint32_t new_beat;
  uint32_t last_beat;
  uint32_t lost_beat;
  uint32_t work_beat;
  uint16_t set_offline_beat : 12;
  uint16_t set_online_beat : 12;
  uint8_t enable : 1;
  uint8_t priority : 4;
  uint8_t error_exist : 1;
  uint8_t is_lost : 1;
  uint8_t data_is_error : 1;

  float frequency;
  bool (*data_is_error_fun)(void);
  void (*solve_lost_fun)(void);
  void (*solve_data_error_fun)(void);
} __packed detect_error_t;

// 检测任务
extern void Detect_task(void *pvParameters);

/**
 * @brief          获取设备对应的错误状态
 * @param[in]      err:设备名称
 * @retval         true(错误) 或者false(没错误)
 */
extern bool detect_error(uint8_t heart_number);

/**
 * @brief          记录时间
 * @param[in]      toe:设备名称
 * @retval         none
 */
extern void detect_hook(uint8_t heart_number);

extern void detect_hook_ISR(uint8_t heart_number); // 钩子函数

/**
 * @brief          得到错误列表
 * @param[in]      none
 * @retval         error_list的指针
 */
extern const detect_error_t *get_detect_error_list_point(void);

#endif
