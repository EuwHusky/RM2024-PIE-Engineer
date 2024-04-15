#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H

#include "board.h"

#define PM_UART_RX_BUF_LENGHT 512 // 电管链路裁判系统数据接收数组大小
#define VT_UART_RX_BUF_LENGHT 48  // 图传链路裁判系统数据接收数组大小

extern void referee_task(void *pvParameters);

#endif
