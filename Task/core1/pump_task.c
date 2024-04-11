#include "stdbool.h"

#include "pump_task.h"

#include "board.h"

#include "FreeRTOS.h"
#include "task.h"

#include "dualcore_task.h"

const transmit_data_021 *core0_data_for_pump;

bool sucking = false;

void pump_task(void *pvParameters)
{
    vTaskDelay(1000);

    // 气泵
    HPM_IOC->PAD[IO01].FUNC_CTL = IOC_PA24_FUNC_CTL_GPIO_A_24;
    gpio_set_pin_output_with_initial(HPM_GPIO0, GPIO_DO_GPIOA, 24, 0);

    // 电磁阀
    HPM_IOC->PAD[I2C1_SDA].FUNC_CTL = IOC_PA10_FUNC_CTL_GPIO_A_10;
    gpio_set_pin_output_with_initial(HPM_GPIO0, GPIO_DO_GPIOA, 10, 1);

    core0_data_for_pump = get_data_021_point();

    uint32_t key_timer = 0;
    int16_t pump_rocker_value = 0;
    while (1)
    {
        pump_rocker_value = core0_data_for_pump->rc_data_image.rc.ch[4];

        // 长拨切换吸取状态
        if (pump_rocker_value < -600)
        {
            key_timer++;
            if (key_timer > 25)
            {
                sucking = !sucking;
                key_timer = 0;
            }
        }

        gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, 24, sucking ? 1 : 0);  // 气泵
        gpio_write_pin(HPM_GPIO0, GPIO_DO_GPIOA, 10, !sucking ? 1 : 0); // 电磁阀

        vTaskDelay(20);
    }
}
