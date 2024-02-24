#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H

#include "board.h"
#include "hpm_dma_drv.h"
#include "hpm_dmamux_drv.h"
#include "hpm_gptmr_drv.h"
#include "hpm_trgm_drv.h"
#include "hpm_trgmmux_src.h"
#include "hpm_uart_drv.h"

#include "fifo.h"
#include "referee.h"
/* ----------------------- 图传链路数据结构&函数 ------------------------------------- */
#define IMAGE_RX_BUF_LENGHT 512    // 图传链路数据接受数组大小
#define IMAGE_FIFO_BUF_LENGTH 1024 // 图传链路fifo数组大小

/* ----------------------- 遥控器数据结构&函数 ------------------------------------- */
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define switch_is_down(s) (s == RC_SW_DOWN)
#define switch_is_mid(s) (s == RC_SW_MID)
#define switch_is_up(s) (s == RC_SW_UP)

#define SBUS_RX_BUF_NUM 36u        // 遥控器数据接受数组大小
#define RC_CHANNAL_ERROR_VALUE 700 // 遥控器出错数据上限

#define RC_KEY_ELI 10     // 消抖时间，按键有效状态保留时间
#define RC_KEY_DELAY 1000 // 间隔长按开始时间ms
#define RC_KEY_WAIT 800   // 间隔长按的间隔时间ms
#define RC_KEY_NUM 16     // 按键个数

#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)

typedef struct
{
  __packed struct
  {
    int16_t ch[5]; // 遥控器遥感&拨盘
    char s[2];     // 遥控器拨杆
  } rc;
  __packed struct
  {
    int16_t x;       // 鼠标x
    int16_t y;       // 鼠标y
    int16_t z;       // 鼠标滚轮
    uint8_t press_l; // 鼠标左键
    uint8_t press_r; // 鼠标右键
  } mouse;
  __packed struct
  {
    uint16_t v; // 16个按键数据
  } key;
} __packed RC_ctrl_t;

extern bool remote_lose;                         // 遥控器丢失或出错超过上限标志位
extern void Remote_task(void *pvParameters);     // 遥控器接收任务
const RC_ctrl_t *get_remote_control_point(void); // 获取遥控器数据指针

/* ----------------------- 键盘数据结构&函数 ------------------------------------- */

#define KEY_PRESSED_OFFSET_W ((uint16_t)1 << 0)
#define KEY_PRESSED_OFFSET_S ((uint16_t)1 << 1)
#define KEY_PRESSED_OFFSET_A ((uint16_t)1 << 2)
#define KEY_PRESSED_OFFSET_D ((uint16_t)1 << 3)
#define KEY_PRESSED_OFFSET_SHIFT ((uint16_t)1 << 4)
#define KEY_PRESSED_OFFSET_CTRL ((uint16_t)1 << 5)
#define KEY_PRESSED_OFFSET_Q ((uint16_t)1 << 6)
#define KEY_PRESSED_OFFSET_E ((uint16_t)1 << 7)
#define KEY_PRESSED_OFFSET_R ((uint16_t)1 << 8)
#define KEY_PRESSED_OFFSET_F ((uint16_t)1 << 9)
#define KEY_PRESSED_OFFSET_G ((uint16_t)1 << 10)
#define KEY_PRESSED_OFFSET_Z ((uint16_t)1 << 11)
#define KEY_PRESSED_OFFSET_X ((uint16_t)1 << 12)
#define KEY_PRESSED_OFFSET_C ((uint16_t)1 << 13)
#define KEY_PRESSED_OFFSET_V ((uint16_t)1 << 14)
#define KEY_PRESSED_OFFSET_B ((uint16_t)1 << 15)

#define KEY_ELI 10     // 消抖时间，按键有效状态保留时间
#define KEY_DELAY 1000 // 间隔长按开始时间	ms
#define KEY_WAIT 800   // 间隔长按的间隔时间   ms
#define KEY_NUMBER 16  // 按键个数
// 对按键进行功能定义
#define SHOOT_CONTROL_KEY KEY_PRESSED_OFFSET_CTRL // 摩擦轮开关（左CTRL）
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W    // 前进（W）
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S     // 后退（S）
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A     // 左（A）
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D    // 右（D）
#define CHASSIS_POWER_UP KEY_PRESSED_OFFSET_SHIFT // 加速按键（左shift）

// 按键信息结构体
typedef struct {
  uint16_t Up_Down;               // 检测抬起-按下状态
  uint16_t Down_Up;               // 检测按下-抬起状态
  uint16_t Now_State;             // 检测当前是否正在按下（按下就为1）
  uint16_t Last_State;            // 上次按键状态
  uint16_t Continue;              // 检测当前是否按下（间隔脉冲返回1）
  uint64_t Time_Flag[KEY_NUMBER]; // 目标时间
  uint64_t Time_Now;              // 按键时间线
} IfKey;

void rc_keyboard_update(void); // 按键状态更新

/*
 *读取按键抬起按下过程
 *return：1 按下
 *读取后会清除状态
 */
int get_keyboard_up_down(uint16_t Rc_Key);

/*
 *检测按键按下抬起过程
 *读取后会清除该状态
 */
int get_keyboard_down_up(uint16_t Rc_Key);

/*
 *读取按键当前是否按下
 *return 1则正在按下
 */
int get_keyboard_now_state(uint16_t Rc_Key);
/*
 *读取按键长按间隔确认值，与上个函数的差别在于长按时每隔delaytime才返回1。
 */
int get_keyboard_cont_enter(uint16_t Rc_Key);

/*
 *按下抬起过程奇数次返回1，偶数次返回0
 *对于同一个按键get_keyboard_down_up 和 get_switch_keyboard_down_up 不能同时使用
 */
int get_switch_keyboard_down_up(uint16_t Rc_Key);

/*
 *抬起按下过程奇数次返回1，偶数次返回0
 *对于同一个按键get_keyboard_up_down 和 get_switch_keyboard_up_down 不能同时使用
 */
int get_switch_keyboard_up_down(uint16_t Rc_Key);

#endif
