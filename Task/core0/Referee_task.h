#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H

#include "board.h"

#define PM_UART_RX_BUF_LENGHT 512 // 电管链路裁判系统数据接收数组大小
#define VT_UART_RX_BUF_LENGHT 120 // 图传链路裁判系统数据接收数组大小

#define VAU_AID_0_UI_INDEX (7)
#define VAU_AID_1_UI_INDEX (8)
#define VAU_AID_2_UI_INDEX (9)

#define AIM_SIGHT_0_UI_INDEX (10)
#define AIM_SIGHT_1_UI_INDEX (11)

#define SAFE_RIGHT_BARRIER_WARNING_LINE_UI_INDEX (12)
#define DANGER_RIGHT_BARRIER_WARNING_LINE_UI_INDEX (13)

#define LIFTER_LEFT_MOTOR_OVER_TEMP_WARNING_UI_INDEX (14)
#define LIFTER_RIGHT_MOTOR_OVER_TEMP_WARNING_UI_INDEX (15)

extern void referee_task(void *pvParameters);

#endif
