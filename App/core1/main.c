#include "board.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

#include "detect_task.h"
#include "dualcore_task.h"
#include "print_task.h"
#include "pump_task.h"

/* Freertos任务优先级（数字越小优先级越高1-7） */
#define DUALCORE_TASK_PRIORITY (configMAX_PRIORITIES - 1U)
#define DETECT_TASK_PRIORITY (configMAX_PRIORITIES - 3U)
#define PUMP_TASK_PRIORITY (configMAX_PRIORITIES - 4U)
#define PRINT_TASK_PRIORITY (configMAX_PRIORITIES - 7U)
#define TEST_TASK_PRIORITY (configMAX_PRIORITIES - 7U)

void test_task(void *pvParameters)
{
    board_write_led_b(LED_ON);
    vTaskDelay(1000);
    int a = 0;

    while (true)
    {
        a = (a > 100) ? 0 : a++;
        // board_write_led_b(LED_ON);
        vTaskDelay(200);
        // board_write_led_b(LED_OFF);
        vTaskDelay(200);
    }
}

int main(void)
{
    board_init_pmp(); // 双核通信共享内存保护

    // xTaskCreate(dualcore_task, "dualcore_task", configMINIMAL_STACK_SIZE + 256U, NULL, DUALCORE_TASK_PRIORITY, NULL);
    // xTaskCreate(detect_task, "detect_task", configMINIMAL_STACK_SIZE + 128U, NULL, DETECT_TASK_PRIORITY, NULL);

    // xTaskCreate(pump_task, "pump_task", configMINIMAL_STACK_SIZE, NULL, PUMP_TASK_PRIORITY, NULL);

    xTaskCreate(test_task, "test_task", configMINIMAL_STACK_SIZE, NULL, TEST_TASK_PRIORITY, NULL);
    xTaskCreate(print_task, "print_task", configMINIMAL_STACK_SIZE, NULL, PRINT_TASK_PRIORITY, NULL);

    vTaskStartScheduler(); // 启动freertos
    while (1)
        ;
    return 0;
}
