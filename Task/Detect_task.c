#include "detect_task.h"

#include "FreeRTOS.h"
#include "FreeRTOSConfig.h"
#include "task.h"

#if !BOARD_RUNNING_CORE
#include "INS_task.h"
#else
#endif

/*初始化error列表*/
static void detect_system_data_init(uint32_t time);

detect_error_t detect_error_list[DETECT_ERROR_LIST_LENGHT + 1];

// 检测任务
void detect_task(void *pvParameters)
{
#if !BOARD_RUNNING_CORE
    while (!INS_init_finished)
        vTaskDelay(8);
#else
#endif
    vTaskDelay(10);

    detect_system_data_init(xTaskGetTickCount()); // init,初始化

    static uint32_t system_time = 0;

    while (true)
    {
        static uint8_t detect_error_num_display = 0;
        system_time = xTaskGetTickCount();

        detect_error_num_display = DETECT_ERROR_LIST_LENGHT;
        detect_error_list[DETECT_ERROR_LIST_LENGHT].is_lost = 0;
        detect_error_list[DETECT_ERROR_LIST_LENGHT].error_exist = 0;

        for (int i = 0; i < DETECT_ERROR_LIST_LENGHT; i++)
        {
            // 未使能，跳过
            if (detect_error_list[i].enable == 0)
                continue;

            // 判断掉线
            if (system_time - detect_error_list[i].new_beat > detect_error_list[i].set_offline_beat)
            {
                if (detect_error_list[i].error_exist == 0)
                {
                    // 记录错误以及掉线时间
                    detect_error_list[i].is_lost = 1;
                    detect_error_list[i].error_exist = 1;
                    detect_error_list[i].lost_beat = system_time;
                }
                // 判断错误优先级， 保存优先级最高的错误码
                if (detect_error_list[i].priority > detect_error_list[detect_error_num_display].priority)
                {
                    detect_error_num_display = i;
                }

                detect_error_list[DETECT_ERROR_LIST_LENGHT].is_lost = 1;
                detect_error_list[DETECT_ERROR_LIST_LENGHT].error_exist = 1;

                // 如果提供解决函数，运行解决函数
                if (detect_error_list[i].solve_lost_fun != NULL)
                    detect_error_list[i].solve_lost_fun();
            }
            else if (system_time - detect_error_list[i].work_beat < detect_error_list[i].set_online_beat)
            {
                // 刚刚上线，可能存在数据不稳定，只记录不丢失，
                detect_error_list[i].is_lost = 0;
                detect_error_list[i].error_exist = 1;
            }
            else
            {
                detect_error_list[i].is_lost = 0;
                // 判断是否存在数据错误
                if (detect_error_list[i].data_is_error != NULL)
                    detect_error_list[i].error_exist = 1;
                else
                    detect_error_list[i].error_exist = 0;
                // 计算频率
                if (detect_error_list[i].new_beat > detect_error_list[i].last_beat)
                    detect_error_list[i].frequency =
                        configTICK_RATE_HZ / (float)(detect_error_list[i].new_beat - detect_error_list[i].last_beat);
            }
        }
        vTaskDelay(DETECT_CONTROL_TIME);
    }
}

/**
 * @brief 获取设备对应的错误状态
 * @param[in] device_dh 设备的detect handle
 * @retval true(错误) 或者false(没错误)
 */
bool detect_error(uint8_t device_dh)
{
    return (detect_error_list[device_dh].error_exist == 1);
}

/**
 * @brief 刷新设备在线状态
 * @param[in] device_dh 设备的detect handle
 * @retval none
 */
void detect_hook(uint8_t device_dh)
{
    detect_error_list[device_dh].last_beat = detect_error_list[device_dh].new_beat;
    detect_error_list[device_dh].new_beat = xTaskGetTickCount();

    if (detect_error_list[device_dh].is_lost)
    {
        detect_error_list[device_dh].is_lost = 0;
        detect_error_list[device_dh].work_beat = detect_error_list[device_dh].new_beat;
    }

    if (detect_error_list[device_dh].data_is_error_fun != NULL)
    {
        if (detect_error_list[device_dh].data_is_error_fun())
        {
            detect_error_list[device_dh].error_exist = 1;
            detect_error_list[device_dh].data_is_error = 1;

            if (detect_error_list[device_dh].solve_data_error_fun != NULL)
                detect_error_list[device_dh].solve_data_error_fun();
        }
        else
            detect_error_list[device_dh].data_is_error = 0;
    }
    else
        detect_error_list[device_dh].data_is_error = 0;
}

/**
 * @brief 刷新设备在线状态 可在中断服务函数中使用
 * @param[in] device_dh 设备的detect handle
 * @retval none
 */
void detect_hook_in_isr(uint8_t device_dh)
{
    detect_error_list[device_dh].last_beat = detect_error_list[device_dh].new_beat;
    detect_error_list[device_dh].new_beat = xTaskGetTickCountFromISR();

    if (detect_error_list[device_dh].is_lost)
    {
        detect_error_list[device_dh].is_lost = 0;
        detect_error_list[device_dh].work_beat = detect_error_list[device_dh].new_beat;
    }

    if (detect_error_list[device_dh].data_is_error_fun != NULL)
    {
        if (detect_error_list[device_dh].data_is_error_fun())
        {
            detect_error_list[device_dh].error_exist = 1;
            detect_error_list[device_dh].data_is_error = 1;

            if (detect_error_list[device_dh].solve_data_error_fun != NULL)
            {
                detect_error_list[device_dh].solve_data_error_fun();
            }
        }
        else
        {
            detect_error_list[device_dh].data_is_error = 0;
        }
    }
    else
    {
        detect_error_list[device_dh].data_is_error = 0;
    }
}

static void detect_system_data_init(uint32_t time) // 初始化
{
    // 设置离线时间，上线稳定工作时间，优先级
    uint16_t set_item[DETECT_ERROR_LIST_LENGHT][3] = {
#if !BOARD_RUNNING_CORE
        {10, 10, 10},  // 双核
        {500, 500, 1}, // 裁判系统
        {50, 50, 8},   // 图传
        {50, 50, 9},   // 遥控器
        {250, 250, 1}, // 驱动1
        {250, 250, 1}, // 驱动2
        {250, 250, 1}, // 驱动3
        {250, 250, 1}, // 驱动4
#else
        {10, 10, 10}, // 双核
#endif
    };

    for (uint8_t i = 0; i < DETECT_ERROR_LIST_LENGHT; i++)
    {
        detect_error_list[i].set_offline_beat = set_item[i][0];
        detect_error_list[i].set_online_beat = set_item[i][1];
        detect_error_list[i].priority = set_item[i][2];
        detect_error_list[i].data_is_error_fun = NULL;
        detect_error_list[i].solve_lost_fun = NULL;
        detect_error_list[i].solve_data_error_fun = NULL;

        detect_error_list[i].enable = 1;
        detect_error_list[i].error_exist = 1;
        detect_error_list[i].is_lost = 1;
        detect_error_list[i].data_is_error = 1;
        detect_error_list[i].frequency = 0.0f;
        detect_error_list[i].new_beat = time;
        detect_error_list[i].last_beat = time;
        detect_error_list[i].lost_beat = time;
        detect_error_list[i].work_beat = time;
    }
}

const detect_error_t *get_detect_error_list_point(void) // 反馈错误列表
{
    return detect_error_list;
}
