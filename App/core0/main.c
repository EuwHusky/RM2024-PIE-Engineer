#include "FreeRTOS.h"
#include "board.h"
#include "hpm_can_drv.h"
#include "task.h"
#include "usb_osal.h"

//#include "Computer_task.h"
#include "Detect_task.h"
#include "Dualcore_task.h"
#include "INS_task.h"
#include "Print_task.h"
#include "Referee_task.h"
#include "multicore_common.h"

/* Freertos任务优先级（数字越小优先级越高1-7） */
#define INS_task_PRIORITY (configMAX_PRIORITIES - 1U)
#define Dualcore_task_PRIORITY (configMAX_PRIORITIES - 1U)
#define Computer_task_PRIORITY (configMAX_PRIORITIES - 1U)
#define Referee_task_PRIORITY (configMAX_PRIORITIES - 2U)
#define Detect_task_PRIORITY (configMAX_PRIORITIES - 3U)
#define Print_task_PRIORITY (configMAX_PRIORITIES - 7U)
#define test_task_PRIORITY (configMAX_PRIORITIES - 7U)

// 按键中断服务函数
void isr_gpio(void) {
  gpio_clear_pin_interrupt_flag(BOARD_KEY_GPIO_CTRL, BOARD_KEY_GPIO_INDEX, BOARD_KEY0_GPIO_PIN);

  printf("KEY DOWN!\n");
}
SDK_DECLARE_EXT_ISR_M(BOARD_KEY_GPIO_IRQ, isr_gpio)

void test_task(void *pvParameters) {
  // 配置按键0中断
  HPM_IOC->PAD[KEY_0].FUNC_CTL = IOC_PA00_FUNC_CTL_GPIO_A_00;
  gpiom_set_pin_controller(HPM_GPIOM, GPIOM_ASSIGN_GPIOA, BOARD_KEY0_GPIO_PIN, gpiom_soc_gpio0);
  gpio_set_pin_input(BOARD_KEY_GPIO_CTRL, BOARD_KEY_GPIO_INDEX, BOARD_KEY0_GPIO_PIN);
  gpio_enable_pin_interrupt(BOARD_KEY_GPIO_CTRL, BOARD_KEY_GPIO_INDEX, BOARD_KEY0_GPIO_PIN);
  gpio_config_pin_interrupt(BOARD_KEY_GPIO_CTRL, BOARD_KEY_GPIO_INDEX, BOARD_KEY0_GPIO_PIN, gpio_interrupt_trigger_edge_falling);
  intc_m_enable_irq_with_priority(BOARD_KEY_GPIO_IRQ, 1);

  while (1) {
    // board_write_led_r(LED_ON);
    board_write_led_g(LED_ON);
    // board_write_led_b(LED_ON);
    vTaskDelay(200);
    // board_write_led_r(LED_OFF);
    board_write_led_g(LED_OFF);
    // board_write_led_b(LED_OFF);
    vTaskDelay(200);
  }
}

static void start_cpu1(void) { multicore_release_cpu(HPM_CORE1, SEC_CORE_IMG_START); }

int main(void) {
  board_init();           // 初始化板子
  board_init_beep_pins(); // 初始化蜂鸣器
  board_init_led_pins();  // 初始化LED
  start_cpu1();           // 启动cpu1

  // 创建任务
  xTaskCreate(Detect_task, "Detect_task", configMINIMAL_STACK_SIZE, NULL, Detect_task_PRIORITY, NULL);                   // 任务检测任务
  xTaskCreate(INS_task, "INS_task", configMINIMAL_STACK_SIZE, NULL, INS_task_PRIORITY, NULL);                            // 陀螺仪读取解算任务
  xTaskCreate(Dualcore_task, "Dualcore_task", configMINIMAL_STACK_SIZE, NULL, Dualcore_task_PRIORITY, NULL);             // 双核通信任务
  xTaskCreate(Referee_task, "Referee_task", configMINIMAL_STACK_SIZE, NULL, Referee_task_PRIORITY, NULL);                // 裁判系统任务

  xTaskCreate(Print_task, "Print_task", configMINIMAL_STACK_SIZE, NULL, Print_task_PRIORITY, NULL); // 调试输出任务
  xTaskCreate(test_task, "test_task", configMINIMAL_STACK_SIZE, NULL, test_task_PRIORITY, NULL);    // 测试任务
  //    启动freertos
  vTaskStartScheduler();
  while (1)
    ;
  return 0;
}
