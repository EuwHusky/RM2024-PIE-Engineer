#include "customer_controller.h"

/**
 * @brief 检测自定义控制器按键是否被按下
 *
 * @param key_status 自定义控制器按键状态簇
 * @param index 按键索引值
 * @return true 按键被按下
 * @return false 按键未被按下
 */
bool checkIfCustomerControllerKeyPressed(uint8_t key_status, customer_controller_key_index_e index)
{
    return ((key_status >> index) & 1);
}
