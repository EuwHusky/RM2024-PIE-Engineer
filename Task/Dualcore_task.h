#ifndef Dualcore_H
#define Dualcore_H

#include "board.h"

#if !BOARD_RUNNING_CORE /* Core0 */

#define MBX HPM_MBX0A
#define MBX_IRQ IRQn_MBX0A

#else /* Core1 */

#define MBX HPM_MBX0B
#define MBX_IRQ IRQn_MBX0B

#endif

static volatile bool allow_write = false;
static volatile bool allow_read = false;

// 核心0向核心1传输的数据结构体
typedef struct transmit_021
{
    bool arm_grab;

} transmit_data_021;

// 核心1向核心0传输的数据结构体
typedef struct transmit_120
{
    uint8_t struct_keep;

} transmit_data_120;

// 双核通信任务
extern void dualcore_task(void *pvParameters);
// 返回核心0向核心1传输的数据地址
extern transmit_data_021 *get_data_021_point();
// 返回核心1向核心0传输的数据地址
extern transmit_data_120 *get_data_120_point();

#endif
