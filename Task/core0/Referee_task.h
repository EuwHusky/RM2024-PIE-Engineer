#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H

#include "board.h"

#define PM_UART_RX_BUF_LENGHT 512 // 电管链路裁判系统数据接收数组大小
#define VT_UART_RX_BUF_LENGHT 120 // 图传链路裁判系统数据接收数组大小

#define SILVER_AID_0_UI_INDEX (7)
#define SILVER_AID_1_UI_INDEX (8)
#define SILVER_AID_2_UI_INDEX (9)

#define LEFT_GOLD_INDICATOR_AID_UI_INDEX (10)
#define MID_GOLD_INDICATOR_AID_UI_INDEX (11)
#define RIGHT_GOLD_INDICATOR_AID_UI_INDEX (12)

#define SAFE_RIGHT_BARRIER_WARNING_LINE_UI_INDEX (13)
#define DANGER_RIGHT_BARRIER_WARNING_LINE_UI_INDEX (14)

extern void referee_task(void *pvParameters);

#endif
