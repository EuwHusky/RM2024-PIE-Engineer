#ifndef Dualcore_H
#define Dualcore_H

#include "board.h"

#if !BOARD_RUNNING_CORE// core0

#include "Referee_task.h"



#define MBX HPM_MBX0A
#define MBX_IRQ IRQn_MBX0A

#else// core1

#define MBX HPM_MBX0B
#define MBX_IRQ IRQn_MBX0B






#endif

static volatile bool allow_write = false;
static volatile bool allow_read = false;

// 核心0向核心1传输的数据结构体
typedef struct transmit_021
{
    //float Yaw;
    //float Yaw_Speed;
    //float Pitch;
    //float Pitch_Speed;
    //recv_data_t computer_data;
    //Refree_data_t refree_data;
} transmit_data_021;

// 核心1向核心0传输的数据结构体
typedef struct transmit_120
{
    //refree_need_data_t ref_need;
    //// cpu1向cpu0发送自瞄目标切换标志
    //// 0,1分别是左，右切换目标，2是切换自瞄模式，3是切换目标类型
    //bool toggle_targets[4];

} transmit_data_120;

// 双核通信任务
extern void Dualcore_task(void *pvParameters);
// 返回核心0向核心1传输的数据地址
extern transmit_data_021 *get_data_021_point();
// 返回核心1向核心0传输的数据地址
extern transmit_data_120 *get_data_120_point();

#endif
