#ifndef REFEREE_H
#define REFEREE_H

#include "stdint.h"

#include "board.h"

#include "algo_robomaster_referee_protocol.h"

/**
 * @命令码 0x0003
 * @数据段长度 32
 * @说明 机器人血量数据，固定以 3Hz 频率发送
 * @发送方/接收方 服务器→全体机器人
 * @所属数据链路 常规链路
 */
typedef struct
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} __packed game_robot_HP_t;

/**
 * @命令码 0x0302
 * @数据段长度 30
 * @说明 自定义控制器与机器人交互数据，发送方触发发送，频率上限为 30Hz
 * @发送方/接收方 自定义控制器→选手端图传连接的机器人
 * @所属数据链路 图传链路
 */
typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
    float pitch;
    float roll;
    uint8_t reserved[6];
} __packed custom_robot_data_t;

/**
 * @命令码 0x0304
 * @数据段长度 30
 * @说明 自定义控制器与机器人交互数据，发送方触发发送，频率上限为 30Hz
 * @发送方/接收方 自定义控制器→选手端图传连接的机器人
 * @所属数据链路 图传链路
 */
typedef struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} __packed remote_control_t;

extern void init_referee_struct_data(void);
extern void referee_data_solve(uint8_t *frame);

extern const game_robot_HP_t *getRobotHp(void);

extern const custom_robot_data_t *getCustomerControllerData(void);
extern const remote_control_t *getRemoteControlData(void);

#endif
