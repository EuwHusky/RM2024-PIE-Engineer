#include "FreeRTOS.h"
#include "board.h"
#include "semphr.h"
#include "task.h"

#include "drv_can.h"

#include "bsp_ma600.h"

#include "dev_motor.h"

#include "CAN_receive.h"
// #include "Can_task.h"
// #include "Chassis_task.h"
#include "Detect_task.h"
#include "Dualcore_task.h"
// #include "Gimbal_task.h"
#include "Print_task.h"
// #include "Shoot_task.h"
#include "remote_control.h"

/* Freertos任务优先级（数字越小优先级越高1-7） */
#define Dualcore_task_PRIORITY (configMAX_PRIORITIES - 1U)
#define Gimbal_task_PRIORITY (configMAX_PRIORITIES - 2U)
#define Shoot_task_PRIORITY (configMAX_PRIORITIES - 2U)
#define Chassis_task_PRIORITY (configMAX_PRIORITIES - 2U)
#define Can_task_PRIORITY (configMAX_PRIORITIES - 2U)
#define Remote_task_PRIORITY (configMAX_PRIORITIES - 3U)
#define Detect_task_PRIORITY (configMAX_PRIORITIES - 3U)
#define Print_task_PRIORITY (configMAX_PRIORITIES - 7U)
#define test_task_PRIORITY (configMAX_PRIORITIES - 7U)

void test_task(void *pvParameters)
{
    uint8_t *test_motor_feedback;
    test_motor_feedback = malloc(8 * sizeof(uint8_t));
    rflCanRxMessageBoxAddId(1, 0x201);

    vTaskDelay(3000);

    rfl_motor_s test_motor;
    rfl_motor_config_s test_motor_config;
    rflMotorGetDefaultConfig(&test_motor_config, RFL_MOTOR_RM_M2006, RFL_MOTOR_CONTROLLER_PID);
    rflAngleUpdate(&test_motor_config.max_angle, RFL_ANGLE_FORMAT_DEGREE, 660.0f);
    rflAngleUpdate(&test_motor_config.min_angle, RFL_ANGLE_FORMAT_DEGREE, -660.0f);
    rflMotorInit(&test_motor, &test_motor_config);
    rflMotorSetMode(&test_motor, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);

    test_motor_feedback = rflCanGetRxMessageBoxData(1, 0x201);

    const RC_ctrl_t *rc_data;
    rc_data = get_remote_control_point();

    // MA600_init();
    // uint16_t angle = 0;
    // bool error;
    bool has_error = false;

    while (true)
    {
        rflMotorSetAngle(&test_motor, RFL_ANGLE_FORMAT_DEGREE, (float)rc_data->rc.ch[2]);

        rflMotorUpdateStatus(&test_motor);
        rflMotorUpdateControl(&test_motor);

        rflRmMotorControl(1, 0x200, (int16_t)rflMotorGetOutput(&test_motor), 0, 0, 0);

        printf("%f,%f\r\n", rflMotorGetAngle(&test_motor, RFL_ANGLE_FORMAT_DEGREE), rflMotorGetSpeed(&test_motor));
        // printf("%x,%x,%x,%x,%x,%x,%x,%x\r\n", test_motor_feedback[0], test_motor_feedback[1], test_motor_feedback[2],
        //        test_motor_feedback[3], test_motor_feedback[4], test_motor_feedback[5], test_motor_feedback[6],
        //        test_motor_feedback[7]);

        // angle = MA600_read_with_check(&error);
        // printf("%d,%d\n", error, angle);
        // for (uint8_t i = My_dualcore; i < DETECT_ERROR_LIST_LENGHT; i++) {
        //  if (detect_error(i)) {
        //    has_error = true;
        //    break;
        //  } else {
        //    has_error = false;
        //  }
        //}
        board_write_led_r(!has_error);
        board_write_led_b(LED_ON);
        vTaskDelay(2);
        board_write_led_b(LED_OFF);
        vTaskDelay(2);
    }
}

int main(void)
{
    board_init_pmp(); // 双核通信共享内存保护
    rflCanInit();

    // 创建任务
    xTaskCreate(Detect_task, "Detect_task", configMINIMAL_STACK_SIZE + 128U, NULL, Detect_task_PRIORITY,
                NULL); // 任务检测任务
    xTaskCreate(Dualcore_task, "Dualcore_task", configMINIMAL_STACK_SIZE + 256U, NULL, Dualcore_task_PRIORITY,
                NULL); // 核间通信任务
    // xTaskCreate(Gimbal_task, "Gimbal_task", configMINIMAL_STACK_SIZE + 256U, NULL, Gimbal_task_PRIORITY, NULL); //
    // 云台控制任务 xTaskCreate(Shoot_task, "Shoot_task", configMINIMAL_STACK_SIZE + 256U, NULL, Shoot_task_PRIORITY,
    // NULL);          // 发射控制任务 xTaskCreate(Chassis_task, "Chassis_task", configMINIMAL_STACK_SIZE + 256U, NULL,
    // Chassis_task_PRIORITY, NULL);    // 底盘控制任务 xTaskCreate(Can_task, "Can_task", configMINIMAL_STACK_SIZE +
    // 256U, NULL, Can_task_PRIORITY, NULL);                // can控制数据发送任务
    xTaskCreate(Remote_task, "Remote_task", configMINIMAL_STACK_SIZE + 128U, NULL, Remote_task_PRIORITY,
                NULL); // 控制数据接收任务（包括遥控器和图传链路）

    xTaskCreate(Print_task, "Print_task", configMINIMAL_STACK_SIZE, NULL, Print_task_PRIORITY, NULL); // 调试输出任务
    xTaskCreate(test_task, "test_task", configMINIMAL_STACK_SIZE, NULL, test_task_PRIORITY, NULL);    // 测试任务

    vTaskStartScheduler(); //    启动freertos
    while (1)
        ;
    return 0;
}
