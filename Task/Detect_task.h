#ifndef DETECT_TASK_H
#define DETECT_TASK_H

#include "board.h"

#define DETECT_TASK_INIT_TIME 50
#define DETECT_CONTROL_TIME 10

/**
 * @brief 设备列表
 * @note DH = Detect Handle
 */
enum Detect_List
{
#if !BOARD_RUNNING_CORE
    DUAL_COMM_DH = 0, /* 双核通信 */

    PM_REFEREE_DH, /* 电源管理模块 裁判系统数据链路 */
    UI_REFEREE_DH, /* UI发送 */
    VT_REFEREE_DH, /* 图传模块 裁判系统数据链路 */
    DBUS_DH,       /* 遥控器接收机 */

    ARM_JOINT_1_L_DH,
    ARM_JOINT_1_R_DH,
    ARM_JOINT_2_DH,
    ARM_JOINT_3_DH,
    ARM_JOINT_4_DH,
    ARM_JOINT_56_L_DH,
    ARM_JOINT_56_R_DH,

    CHASSIS_MOTOR_0_DH, /* 底盘电机0 - 左前 */
    CHASSIS_MOTOR_1_DH, /* 底盘电机1 - 左后 */
    CHASSIS_MOTOR_2_DH, /* 底盘电机2 - 右后 */
    CHASSIS_MOTOR_3_DH, /* 底盘电机3 - 右前 */

    GIMBAL_MOTOR_YAW_DH, /* 云台YAW轴电机 */

    LIDAR_DH, /* 激光雷达 */

#else
    DUAL_COMM_DH = 0, /* 双核通信 */

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
extern void detect_task(void *pvParameters);

/**
 * @brief          获取设备对应的错误状态
 * @param[in]      err:设备名称
 * @retval         true(错误) 或者false(没错误)
 */
extern bool detect_error(uint8_t device_dh);

/**
 * @brief          记录时间
 * @param[in]      toe:设备名称
 * @retval         none
 */
extern void detect_hook(uint8_t device_dh);

extern void detect_hook_in_isr(uint8_t device_dh); // 钩子函数

/**
 * @brief          得到错误列表
 * @param[in]      none
 * @retval         error_list的指针
 */
extern const detect_error_t *get_detect_error_list_point(void);

#endif
