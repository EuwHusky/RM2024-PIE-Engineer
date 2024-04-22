#ifndef _CUSTOMER_CONTROLLER_H__
#define _CUSTOMER_CONTROLLER_H__

#include "stdbool.h"
#include "stdint.h"

typedef enum
{
    ROUGHLY_KEY = 0,
    PRECISELY_KEY,
    PUMP_KEY,
    SWITCH_KEY,
    RESET_KEY,
} customer_controller_key_index_e;

/**
 * @brief 检测自定义控制器按键是否被按下
 *
 * @param key_status 自定义控制器按键状态簇
 * @param index 按键索引值
 * @return true 按键被按下
 * @return false 按键未被按下
 */
extern bool checkIfCustomerControllerKeyPressed(uint8_t key_status, customer_controller_key_index_e index);

#endif /* _CUSTOMER_CONTROLLER_H__ */