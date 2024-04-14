#include "board.h"

#include "multicore_common.h"

#include "FreeRTOS.h"
#include "task.h"
#include "usb_osal.h"

#include "drv_can.h"
#include "drv_dma.h"

#include "INS_task.h"
#include "arm_task.h"
#include "behavior_task.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "dualcore_task.h"
#include "print_task.h"
#include "rc_task.h"
#include "referee_task.h"

/* Freertos任务优先级（数字越小优先级越高1-7） */
#define INS_TASK_PRIORITY (configMAX_PRIORITIES - 1U)
#define DUALCORE_TASK_PRIORITY (configMAX_PRIORITIES - 1U)
#define DETECT_TASK_PRIORITY (configMAX_PRIORITIES - 2U)
#define BEHAVIOR_TASK_PRIORITY (configMAX_PRIORITIES - 2U)
#define ARM_TASK_PRIORITY (configMAX_PRIORITIES - 2U)
#define CHASSIS_TASK_PRIORITY (configMAX_PRIORITIES - 2U)
#define RC_TASK_PRIORITY (configMAX_PRIORITIES - 3U)
#define REFEREE_TASK_PRIORITY (configMAX_PRIORITIES - 4U)
#define PRINT_TASK_PRIORITY (configMAX_PRIORITIES - 7U)
#define TEST_TASK_PRIORITY (configMAX_PRIORITIES - 7U)

// 按键中断服务函数
void isr_gpio(void)
{
    gpio_clear_pin_interrupt_flag(BOARD_KEY_GPIO_CTRL, BOARD_KEY_GPIO_INDEX, BOARD_KEY0_GPIO_PIN);

    printf("KEY DOWN!\n");
}
SDK_DECLARE_EXT_ISR_M(BOARD_KEY_GPIO_IRQ, isr_gpio)

void test_task(void *pvParameters)
{
    // 配置按键0中断
    HPM_IOC->PAD[KEY_0].FUNC_CTL = IOC_PA00_FUNC_CTL_GPIO_A_00;
    gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOA, BOARD_KEY0_GPIO_PIN, gpiom_soc_gpio0);
    gpio_set_pin_input(BOARD_KEY_GPIO_CTRL, BOARD_KEY_GPIO_INDEX, BOARD_KEY0_GPIO_PIN);
    gpio_enable_pin_interrupt(BOARD_KEY_GPIO_CTRL, BOARD_KEY_GPIO_INDEX, BOARD_KEY0_GPIO_PIN);
    gpio_config_pin_interrupt(BOARD_KEY_GPIO_CTRL, BOARD_KEY_GPIO_INDEX, BOARD_KEY0_GPIO_PIN,
                              gpio_interrupt_trigger_edge_falling);
    intc_m_enable_irq_with_priority(BOARD_KEY_GPIO_IRQ, 1);

    while (1)
    {
        board_write_led_g(LED_ON);
        vTaskDelay(200);
        board_write_led_g(LED_OFF);
        vTaskDelay(200);
    }
}

static void start_cpu1(void)
{
    multicore_release_cpu(HPM_CORE1, SEC_CORE_IMG_START);
}

int main(void)
{
    board_init();           // 初始化板子
    board_init_beep_pins(); // 初始化蜂鸣器
    board_init_led_pins();  // 初始化LED
    start_cpu1();           // 启动cpu1

    rflDmaInit();
    rflCanInit();

    xTaskCreate(INS_task, "INS_task", configMINIMAL_STACK_SIZE + 128U, NULL, INS_TASK_PRIORITY, NULL);
    xTaskCreate(dualcore_task, "dualcore_task", configMINIMAL_STACK_SIZE + 128U, NULL, DUALCORE_TASK_PRIORITY, NULL);
    xTaskCreate(detect_task, "detect_task", configMINIMAL_STACK_SIZE + 128U, NULL, DETECT_TASK_PRIORITY, NULL);

    xTaskCreate(referee_task, "referee_task", configMINIMAL_STACK_SIZE + 128U, NULL, REFEREE_TASK_PRIORITY, NULL);
    xTaskCreate(rc_task, "rc_task", configMINIMAL_STACK_SIZE + 128U, NULL, RC_TASK_PRIORITY, NULL);

    xTaskCreate(behavior_task, "behavior_task", configMINIMAL_STACK_SIZE, NULL, BEHAVIOR_TASK_PRIORITY, NULL);

    xTaskCreate(arm_task, "arm_task", configMINIMAL_STACK_SIZE + 512U, NULL, ARM_TASK_PRIORITY, NULL);
    xTaskCreate(chassis_task, "chassis_task", configMINIMAL_STACK_SIZE + 256U, NULL, CHASSIS_TASK_PRIORITY, NULL);

    xTaskCreate(test_task, "test_task", configMINIMAL_STACK_SIZE, NULL, TEST_TASK_PRIORITY, NULL);
    xTaskCreate(print_task, "print_task", configMINIMAL_STACK_SIZE, NULL, PRINT_TASK_PRIORITY, NULL);

    // 启动freertos
    vTaskStartScheduler();

    while (1)
    {
    }

    return 0;
}
