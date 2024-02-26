#ifndef REFEREE_USART_TASK_H
#define REFEREE_USART_TASK_H

#include "board.h"
#include "referee.h"
#include "stdarg.h"

#define PM_UART_RX_BUF_LENGHT 512    // 电管链路裁判系统数据接收数组大小
#define PM_UART_FIFO_BUF_LENGTH 1024 // 电管链路裁判系统数据fifo数组大小

#define VT_UART_RX_BUF_LENGHT 512    // 图传链路裁判系统数据接收数组大小
#define VT_UART_FIFO_BUF_LENGTH 1024 // 图传链路裁判系统数据fifo数组大小

#pragma pack(1) // 按一字节对齐

// #define NULL 0
// #define __FALSE 100

/****************************开始标志*********************/
#define UI_SOF 0xA5
/****************************CMD_ID数据********************/
#define UI_CMD_Robo_Exchange 0x0301
/****************************内容ID数据********************/
#define UI_Data_ID_Del 0x100
#define UI_Data_ID_Draw1 0x101
#define UI_Data_ID_Draw2 0x102
#define UI_Data_ID_Draw5 0x103
#define UI_Data_ID_Draw7 0x104
#define UI_Data_ID_DrawChar 0x110
/****************************红方机器人ID********************/
#define UI_Data_RobotID_RHero 1
#define UI_Data_RobotID_REngineer 2
#define UI_Data_RobotID_RStandard1 3
#define UI_Data_RobotID_RStandard2 4
#define UI_Data_RobotID_RStandard3 5
#define UI_Data_RobotID_RAerial 6
#define UI_Data_RobotID_RSentry 7
#define UI_Data_RobotID_RRadar 9
/****************************蓝方机器人ID********************/
#define UI_Data_RobotID_BHero 101
#define UI_Data_RobotID_BEngineer 102
#define UI_Data_RobotID_BStandard1 103
#define UI_Data_RobotID_BStandard2 104
#define UI_Data_RobotID_BStandard3 105
#define UI_Data_RobotID_BAerial 106
#define UI_Data_RobotID_BSentry 107
#define UI_Data_RobotID_BRadar 109
/**************************红方操作手ID************************/
#define UI_Data_CilentID_RHero 0x0101
#define UI_Data_CilentID_REngineer 0x0102
#define UI_Data_CilentID_RStandard1 0x0103
#define UI_Data_CilentID_RStandard2 0x0104
#define UI_Data_CilentID_RStandard3 0x0105
#define UI_Data_CilentID_RAerial 0x0106
/***************************蓝方操作手ID***********************/
#define UI_Data_CilentID_BHero 0x0165
#define UI_Data_CilentID_BEngineer 0x0166
#define UI_Data_CilentID_BStandard1 0x0167
#define UI_Data_CilentID_BStandard2 0x0168
#define UI_Data_CilentID_BStandard3 0x0169
#define UI_Data_CilentID_BAerial 0x016A
/***************************删除操作***************************/
#define UI_Data_Del_NoOperate 0
#define UI_Data_Del_Layer 1
#define UI_Data_Del_ALL 2
/***************************图形配置参数__图形操作********************/
#define UI_Graph_ADD 1
#define UI_Graph_Change 2
#define UI_Graph_Del 3
/***************************图形配置参数__图形类型********************/
#define UI_Graph_Line 0      // 直线
#define UI_Graph_Rectangle 1 // 矩形
#define UI_Graph_Circle 2    // 整圆
#define UI_Graph_Ellipse 3   // 椭圆
#define UI_Graph_Arc 4       // 圆弧
#define UI_Graph_Float 5     // 浮点型
#define UI_Graph_Int 6       // 整形
#define UI_Graph_Char 7      // 字符型
/***************************图形配置参数__图形颜色********************/
#define UI_Color_Main 0         // 红蓝主色
#define UI_Color_Yellow 1       // 黄色
#define UI_Color_Green 2        // 绿色
#define UI_Color_Orange 3       // 橘色
#define UI_Color_Purplish_red 4 // 紫红色
#define UI_Color_Pink 5         // 粉色
#define UI_Color_Cyan 6         // 青色
#define UI_Color_Black 7        // 黑色
#define UI_Color_White 8        // 白色

typedef enum RefereeLinkType
{
    PM_REFEREE_LINK = 0,
    VT_REFEREE_LINK,
} referee_link_type_e;

typedef struct
{
    uint8_t Delate[17];
} __packed UI_Delate_t; // UI删除图层数组
typedef struct
{
    uint8_t P1[30];
} __packed UI_Refresh_P1; // UI添加图层数组
typedef struct
{
    uint8_t P2[45];
} __packed UI_Refresh_P2; // UI添加图层数组
typedef struct
{
    uint8_t P5[90];
} __packed UI_Refresh_P5; // UI添加图层数组
typedef struct
{
    uint8_t P7[120];
} __packed UI_Refresh_P7; // UI添加图层数组
typedef struct
{
    uint8_t ch[60];
} __packed UI_Refresh_char; // UI添加图层数组
typedef struct
{
    uint8_t fp[30];
} __packed UI_Refresh_float; // UI添加图层数组
typedef struct
{
    uint8_t communication[113];
} __packed UI_Refresh_communication; // 机器人间通信

typedef struct
{
    uint8_t num;
    uint8_t Picture_Data[128];
    // graphic_data_struct_t *graphic_data[7];
} Refreedata_UI;

// 裁判系统数据结构体，用来向cpu1传输数据
typedef struct
{
    uint8_t robot_id;

} Referee_data_t;

extern void referee_task(void *pvParameters);
extern Referee_data_t *get_referee_data_pointer(void);

#endif
