#include "print_task.h"

#include "FreeRTOS.h"
#include "task.h"

#define PRINT_ERROR (false) // 是否输出异常
#define PRINT_TIME_MS 5     // 输出数据的周期

#if !BOARD_RUNNING_CORE
// core0
#include "INS_task.h"
#include "detect_task.h"
#include "dualcore_task.h"
#include "referee_task.h"
#include "remote_control.h"

// 需求变量
INS_t *ins_;
transmit_data_021 *data_send;
transmit_data_120 *data_read;

void print_task(void *pvParameters)
{
    while (!INS_init_finished)
        vTaskDelay(8);
    vTaskDelay(300);

    ins_ = get_INS_data_point();
    data_read = get_data_120_point();
    data_send = get_data_021_point();

    while (true)
    {
#if PRINT_ERROR
        for (uint8_t i = DUAL_COMM_DH; i < DETECT_ERROR_LIST_LENGHT; i++)
        {
            if (detect_error(i))
                switch (i)
                {
                case DUAL_COMM_DH: // 双核
                    printf("核间通信异常\n");
                    break;
                case PM_REFEREE_DH: // 裁判系统
                    printf("裁判系统串口异常\n");
                    break;
                case DBUS_DH: // 遥控器
                    printf("遥控器串口连接异常\n");
                    break;
                case My_vtm: // 图传
                    printf("图传串口连接异常\n");
                    break;
                case CHASSIS_MOTOR_0_DH: // 驱动1
                    printf("左前驱动信号异常\n");
                    break;
                case CHASSIS_MOTOR_1_DH: // 驱动2
                    printf("右前驱动信号异常\n");
                    break;
                case CHASSIS_MOTOR_2_DH: // 驱动2
                    printf("左后驱动信号异常\n");
                    break;
                case CHASSIS_MOTOR_3_DH: // 驱动4
                    printf("右后驱动信号异常\n");
                    break;
                }
        }
        printf("--------------------\n");
        vTaskDelay(2000);
#else
        // float speed_fliter_1, speed_fliter_2, speed_fliter_3;
        // static const float fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
        // float gyro = ins_->Gyro[2];
        // speed_fliter_1 = speed_fliter_2;
        // speed_fliter_2 = speed_fliter_3;
        // speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + gyro * fliter_num[2];
        // printf("%.3f,%.3f\n", gyro, speed_fliter_3);
        // printf("%f,%f,%f,%f\n", ins_->Yaw, -ins_->Pitch, computer_print->aim_yaw, computer_print->aim_pitch);

        // printf("%f,%f,%f\n", ins_->Yaw, -ins_->Pitch, ins_->Roll);

        // printf("running\n");
        vTaskDelay(PRINT_TIME_MS);
#endif
    }
}

#else
// core1
#include "detect_task.h"
#include "dualcore_task.h"
#include "print_task.h"

// 需求变量
const transmit_data_021 *core0_data_print;

void print_task(void *pvParameters)
{
    board_init_console(); // 初始化调试用串口
    while (detect_error(DUAL_COMM_DH))
        vTaskDelay(PRINT_TIME_MS);

    core0_data_print = get_data_021_point();

    while (true)
    {
#if PRINT_ERROR
        for (uint8_t i = DUAL_COMM_DH; i < DETECT_ERROR_LIST_LENGHT; i++)
        {
            if (detect_error(i))
                switch (i)
                {
                case DUAL_COMM_DH: // 双核
                    printf("核间通信异常\n");
                    break;
                }
        }
        printf("--------------------\n");
        vTaskDelay(2000);
#else

        printf("%d,%d,%d,%d,%d\r\n", core0_data_print->rc_data_image.rc.ch[0], core0_data_print->rc_data_image.rc.ch[1],
               core0_data_print->rc_data_image.rc.ch[2], core0_data_print->rc_data_image.rc.ch[3],
               core0_data_print->rc_data_image.rc.ch[4]);

        // printf("%f,%d,%d,%d,%d,%d,%d,%f,%f,%f,%d,%f,%d\n",
        //     core0_data_print->refree_data.shoot_speed,
        //     shoot_print->shootl.motor_measure->speed_rpm,
        //     shoot_print->shootr.motor_measure->speed_rpm,
        //     shoot_print->tri_2006.motor_measure->speed_rpm,
        //     shoot_print->shootl.set_rpm,
        //     shoot_print->shootr.set_rpm,
        //     shoot_print->tri_2006.set_rpm,
        //     shoot_print->tri_3508.angle_set,
        //     shoot_print->tri_3508.angle,
        //     shoot_print->tri_3508.speed_set,
        //     shoot_print->tri_3508.set_current,
        //     shoot_print->tri_3508.speed,
        //     shoot_print->tri_3508.motor_measure->speed_rpm);

        // printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
        //     shoot_print->Rc_ctrl->rc.s[0],
        //     gimbal_print->gimbal_behaviour,
        //     shoot_print->mode,
        //     shoot_print->shootl.motor_measure->speed_rpm,
        //     shoot_print->shootr.motor_measure->speed_rpm,
        //     shoot_print->tri_2006.motor_measure->speed_rpm,
        //     shoot_print->shootl.set_rpm,
        //     shoot_print->shootr.set_rpm,
        //     shoot_print->tri_2006.set_rpm,
        //     shoot_print->shootl.send_current,
        //     shoot_print->shootr.send_current,
        //     shoot_print->tri_2006.send_current);

        // PITCH轴相关参数输出
        // printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n",
        //    gimbal_print->gimbal_behaviour,
        //    gimbal_print->pitch_motor.relative_angle,
        //    gimbal_print->pitch_motor.relative_angle_set,
        //    gimbal_print->pitch_motor.absolute_angle,
        //    gimbal_print->pitch_motor.absolute_angle_set,
        //    gimbal_print->pitch_motor.gyro_speed,
        //    gimbal_print->pitch_motor.gyro_speed_set,
        //    gimbal_print->pitch_motor.send_current);

        // printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n",
        //     gimbal_print->gimbal_behaviour,
        //     gimbal_print->pitch_motor.absolute_angle,
        //     gimbal_print->pitch_motor.absolute_angle_set,
        //     gimbal_print->pitch_motor.gyro_speed,
        //     gimbal_print->pitch_motor.gyro_speed_set,
        //     gimbal_print->yaw_motor.gyro_speed,
        //     gimbal_print->pitch_motor.set_current);

        // YAW轴相关参数输出
        // printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n",
        //    gimbal_print->gimbal_direction,
        //    gimbal_print->yaw_motor.add_angle,
        //    gimbal_print->yaw_motor.relative_angle,
        //    gimbal_print->yaw_motor.relative_angle_set,
        //    gimbal_print->yaw_motor.absolute_angle,
        //    gimbal_print->yaw_motor.absolute_angle_set,
        //    gimbal_print->yaw_motor.gyro_speed,
        //    gimbal_print->yaw_motor.gyro_speed_set,
        //    gimbal_print->yaw_motor.send_current);

        // printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
        //  gimbal_print->yaw_motor.add_angle,
        //  core0_data_print->computer_data.aim_yaw,
        // core0_data_print->refree_data.shoot_speed,
        // core0_data_print->computer_data.aim_yaw / 180.0 * PI,
        // gimbal_print->yaw_motor.absolute_angle,
        // gimbal_print->yaw_motor.absolute_angle_set,

        //-core0_data_print->computer_data.aim_pitch / 180.0 * PI,
        // gimbal_print->pitch_motor.absolute_angle,
        // gimbal_print->pitch_motor.absolute_angle_set
        // gimbal_print->yaw_motor.gyro_speed,
        // gimbal_print->yaw_motor.gyro_speed_set,
        // gimbal_print->yaw_motor.send_current
        //);

        // 底盘输出
        // printf("%f,%f,%f,%f,%f,%f,%f,%f\n",
        //    chassis_print->drive_motor[0].speed,
        //    chassis_print->drive_motor[1].speed,
        //    chassis_print->drive_motor[2].speed,
        //    chassis_print->drive_motor[3].speed,
        //    chassis_print->drive_motor[0].speed_set,
        //    chassis_print->drive_motor[1].speed_set,
        //    chassis_print->drive_motor[2].speed_set,
        //    chassis_print->drive_motor[3].speed_set);

        // 跟随参数
        // printf("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
        //    chassis_print->chassis_follow_angle,
        //    gimbal_print->yaw_motor.relative_angle,
        //    chassis_print->drive_motor[0].speed,
        //    chassis_print->drive_motor[1].speed,
        //    chassis_print->drive_motor[2].speed,
        //    chassis_print->drive_motor[3].speed,
        //    chassis_print->drive_motor[0].speed_set,
        //    chassis_print->drive_motor[1].speed_set,
        //    chassis_print->drive_motor[2].speed_set,
        //    chassis_print->drive_motor[3].speed_set);

        // printf("%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d\n",
        //     chassis_print->chassis_mode,

        //    chassis_print->drive_motor[0].speed,
        //    chassis_print->drive_motor[1].speed,
        //    chassis_print->drive_motor[2].speed,
        //    chassis_print->drive_motor[3].speed,

        //    chassis_print->drive_motor[0].speed_set,
        //    chassis_print->drive_motor[1].speed_set,
        //    chassis_print->drive_motor[2].speed_set,
        //    chassis_print->drive_motor[3].speed_set,

        //    chassis_print->drive_motor[0].send_current,
        //    chassis_print->drive_motor[1].send_current,
        //    chassis_print->drive_motor[2].send_current,
        //    chassis_print->drive_motor[3].send_current

        //);
        vTaskDelay(PRINT_TIME_MS);
#endif
    }
}

#endif
