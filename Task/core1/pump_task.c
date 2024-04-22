#include "stdbool.h"

#include "pump_task.h"

#include "board.h"

#include "FreeRTOS.h"
#include "task.h"

#include "dualcore_task.h"

const transmit_data_021 *core0_data_for_pump;

void pump_task(void *pvParameters)
{
    vTaskDelay(1000);

    // 气泵
    HPM_IOC->PAD[IOC_PAD_PB31].FUNC_CTL = IOC_PB31_FUNC_CTL_GPIO_B_31;
    gpio_set_pin_output_with_initial(HPM_GPIO0, ENGINEER_ARM_PUMP_GPIO_PORT, ENGINEER_ARM_PUMP_GPIO_PIN, 0);

    // 电磁阀
    HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_GPIO_A_31;
    gpio_set_pin_output_with_initial(HPM_GPIO0, ENGINEER_ARM_VALVE_GPIO_PORT, ENGINEER_ARM_VALVE_GPIO_PIN, 1);

    core0_data_for_pump = get_data_021_point();

    while (1)
    {
        gpio_write_pin(HPM_GPIO0, ENGINEER_ARM_PUMP_GPIO_PORT, ENGINEER_ARM_PUMP_GPIO_PIN,
                       core0_data_for_pump->arm_grab ? 1 : 0); // 气泵
        gpio_write_pin(HPM_GPIO0, ENGINEER_ARM_VALVE_GPIO_PORT, ENGINEER_ARM_VALVE_GPIO_PIN,
                       !core0_data_for_pump->arm_grab ? 1 : 0); // 电磁阀

        vTaskDelay(20);
    }
}
