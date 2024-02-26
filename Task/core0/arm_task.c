#include "stdio.h"
#include "string.h"

#include "arm_task.h"

#include "board.h"

#include "FreeRTOS.h"
#include "task.h"

#include "drv_can.h"

#include "dev_motor.h"

#include "INS_task.h"
#include "detect_task.h"
#include "remote_control.h"

void arm_task(void *pvParameters)
{
    while (!INS_init_finished)
        vTaskDelay(2);
    vTaskDelay(30);

    const RC_ctrl_t *rc_data;
    rc_data = get_remote_control_point();

    rfl_motor_s motor_1;
    rfl_motor_s motor_2;

    damiao_motor_s *drv_1 = NULL;
    damiao_motor_s *drv_2 = NULL;

    rfl_motor_config_s motor_config;
    rflMotorGetDefaultConfig(&motor_config, RFL_MOTOR_DM_J8009_2EC, RFL_MOTOR_CONTROLLER_DAMIAO);
    motor_config.is_reversed = false;
    motor_config.can_ordinal = 1;
    motor_config.master_can_id = 0x02;
    motor_config.slave_can_id = 0x00;
    rflMotorInit(&motor_1, &motor_config);
    motor_config.is_reversed = true;
    motor_config.can_ordinal = 1;
    motor_config.master_can_id = 0x03;
    motor_config.slave_can_id = 0x01;
    rflMotorInit(&motor_2, &motor_config);

    // drv_1 = (damiao_motor_s *)(motor_1.driver);
    // drv_2 = (damiao_motor_s *)(motor_2.driver);

    rfl_motor_s test_motor;
    rfl_motor_config_s test_motor_config;
    rflMotorGetDefaultConfig(&test_motor_config, RFL_MOTOR_RM_M2006, RFL_MOTOR_CONTROLLER_PID);
    motor_config.can_ordinal = 1;
    motor_config.master_can_id = 0x201;
    rflAngleUpdate(&test_motor_config.max_angle, RFL_ANGLE_FORMAT_DEGREE, 660.0f);
    rflAngleUpdate(&test_motor_config.min_angle, RFL_ANGLE_FORMAT_DEGREE, -660.0f);
    motor_config.angle_pid_kp = 0.8f;
    motor_config.angle_pid_kd = 0.1f;
    rflMotorInit(&test_motor, &test_motor_config);
    rflMotorSetMode(&test_motor, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);

    char mode_control_key_value = 0;
    char last_mode_control_key_value = 0;

    while (1)
    {
        mode_control_key_value = rc_data->rc.s[1];
        if (last_mode_control_key_value != 3 && mode_control_key_value == 3)
        {
            rflMotorSetMode(&motor_1, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);

            printf("enable\r\n");
            rflMotorSetMode(&motor_2, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        }
        else if (last_mode_control_key_value != 1 && mode_control_key_value == 1)
        {
            rflMotorSetMode(&motor_1, RFL_MOTOR_CONTROL_MODE_NO_FORCE);

            printf("failure\r\n");
            rflMotorSetMode(&motor_2, RFL_MOTOR_CONTROL_MODE_NO_FORCE);
        }
        last_mode_control_key_value = mode_control_key_value;

        if (rflMotorGetMode(&motor_1) == RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE &&
            rflMotorGetMode(&motor_2) == RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE)
        {
            rflMotorSetAngle(&motor_1, RFL_ANGLE_FORMAT_RADIAN, (float)rc_data->rc.ch[2] / 660.0f * 3.14f);
            rflMotorSetSpeed(&motor_1, 5.0f);
            rflMotorSetAngle(&motor_2, RFL_ANGLE_FORMAT_RADIAN, (float)rc_data->rc.ch[2] / 660.0f * 6.28f);
            rflMotorSetSpeed(&motor_2, 5.0f);
        }

        rflMotorUpdateStatus(&motor_1);
        rflMotorUpdateControl(&motor_1);
        rflMotorExecuteControl(&motor_1);
        // vTaskDelay(1);
        rflMotorUpdateStatus(&motor_2);
        rflMotorUpdateControl(&motor_2);
        rflMotorExecuteControl(&motor_2);
        // vTaskDelay(1);

        printf("%f,%f,%d\r\n", rflMotorGetAngle(&motor_1, RFL_ANGLE_FORMAT_DEGREE),
               rflMotorGetAngle(&motor_2, RFL_ANGLE_FORMAT_DEGREE), mode_control_key_value);

        rflMotorSetAngle(&test_motor, RFL_ANGLE_FORMAT_DEGREE, (float)rc_data->rc.ch[2]);

        rflMotorUpdateStatus(&test_motor);
        rflMotorUpdateControl(&test_motor);

        rflRmMotorControl(1, 0x200, (int16_t)rflMotorGetOutput(&test_motor), 0, 0, 0);

        printf("%f,%f\r\n", rflMotorGetAngle(&test_motor, RFL_ANGLE_FORMAT_DEGREE), rflMotorGetSpeed(&test_motor));

        vTaskDelay(2);
    }
}
