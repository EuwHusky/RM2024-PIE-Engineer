#ifndef REFEREE_H
#define REFEREE_H

#include "stdint.h"

#include "board.h"

#include "algo_robomaster_referee_protocol.h"

typedef struct // 0x0302 客户端下发自定义控制器
{
    float x;
    float y;
    float z;
    float yaw;
    float pitch;
    float roll;
    uint8_t reserved[6];
} __packed custom_robot_data_t;

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

extern const custom_robot_data_t *getCustomerControllerData(void);
extern const remote_control_t *getRemoteControlData(void);

#endif
