#include "print_task.h"

#include "FreeRTOS.h"
#include "task.h"

#define PRINT_ERROR (false) // 是否输出异常
#define PRINT_TIME_MS 600   // 输出数据的周期

#if !BOARD_RUNNING_CORE // core0

#include "drv_dma.h"

#include "INS_task.h"
#include "detect_task.h"
#include "dualcore_task.h"

#include "arm_task.h"
#include "chassis_task.h"

#include "referee.h"

INS_t *ins_;
transmit_data_021 *data_send;
transmit_data_120 *data_read;

engineer_scara_arm_s *arm_data;
engineer_chassis_s *chassis_data;

const game_robot_HP_t *referee_robot_hp;
const robot_status_t *referee_robot_status;

const custom_robot_data_t *customer_controller;

ATTR_PLACE_AT_NONCACHEABLE uint8_t test_txt[512];
volatile bool print_uart_tx_dma_done = true; // dma传输完成标志位

rfl_motor_pid_controller_s *motor_controller_test;
rm_motor_s *motor_driver_test;

void print_dma_isr(void)
{
    print_uart_tx_dma_done = true;
}

void print_task(void *pvParameters)
{
    while (!INS_init_finished)
        vTaskDelay(8);
    vTaskDelay(300);

    uart_config_t config = {0}; // 串口配置
    board_init_uart(BOARD_UART6);
    uart_default_config(BOARD_UART6, &config);                    // 填充默认配置
    config.fifo_enable = true;                                    // 使能FIFO
    config.dma_enable = true;                                     // 使能DMA
    config.baudrate = 460800;                                     // 设置波特率
    config.src_freq_in_hz = clock_get_frequency(BOARD_UART6_CLK); // 获得时钟频率
    config.tx_fifo_level = uart_tx_fifo_trg_not_full;
    if (uart_init(BOARD_UART6, &config) != status_success)
    {
        printf("failed to initialize uart\n");
        while (1)
        {
        }
    }
    intc_m_enable_irq_with_priority(BOARD_HDMA_IRQ, 1);
    dmamux_config(BOARD_DMAMUX, BOARD_UART6_TX_DMAMUX_CHN, BOARD_UART6_TX_DMA_REQ, true);

    rflDmaAddCallbackFunction(BOARD_HDMA, BOARD_UART6_TX_DMA_CHN, print_dma_isr);

    ins_ = get_INS_data_point();
    data_read = get_data_120_point();
    data_send = get_data_021_point();

    arm_data = getArmDataPointer();
    chassis_data = getChassisDataPointer();

    referee_robot_hp = getRobotHp();
    referee_robot_status = getRobotStatus();
    customer_controller = getCustomerControllerData();

    motor_controller_test = (rfl_motor_pid_controller_s *)arm_data->joints_motors[MOTOR_JOINT56_RIGHT].controller;
    motor_driver_test = (rm_motor_s *)arm_data->joints_motors[MOTOR_JOINT56_RIGHT].driver;

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
                case VT_REFEREE_DH: // 图传
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

        if (print_uart_tx_dma_done)
        {
            print_uart_tx_dma_done = false;

            /**
             * @brief Scara Arm
             */
            // sprintf((char *)test_txt, "\r\n%x,%d,%d\r\n%f,%f,%f,%f,%f,%f,%f\r\n%f,%f,%f,%f,%f,%f\r\n",
            //         arm_data->start_up_status, arm_data->mode,
            //         rflMotorGetMode(&arm_data->joints_motors[MOTOR_JOINT4]),
            //         rflMotorGetAngle(&arm_data->joints_motors[MOTOR_JOINT1_LEFT], RFL_ANGLE_FORMAT_RADIAN),
            //         rflMotorGetAngle(&arm_data->joints_motors[MOTOR_JOINT1_RIGHT], RFL_ANGLE_FORMAT_RADIAN),
            //         rflMotorGetAngle(&arm_data->joints_motors[MOTOR_JOINT23_BACK], RFL_ANGLE_FORMAT_RADIAN),
            //         rflMotorGetAngle(&arm_data->joints_motors[MOTOR_JOINT23_FRONT], RFL_ANGLE_FORMAT_RADIAN),
            //         rflMotorGetAngle(&arm_data->joints_motors[MOTOR_JOINT4], RFL_ANGLE_FORMAT_RADIAN),
            //         rflMotorGetAngle(&arm_data->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_RADIAN),
            //         rflMotorGetAngle(&arm_data->joints_motors[MOTOR_JOINT56_RIGHT], RFL_ANGLE_FORMAT_RADIAN),
            //         arm_data->joints_value[JOINT_1], arm_data->joints_value[JOINT_2],
            //         arm_data->joints_value[JOINT_3], arm_data->joints_value[JOINT_4],
            //         arm_data->joints_value[JOINT_5], arm_data->joints_value[JOINT_6]);
            sprintf((char *)test_txt, "%f,%f,%f,%f,%f,%f\r\n", motor_controller_test->angle_pid.set,
                    motor_controller_test->angle_pid.fdb, motor_controller_test->angle_pid.out,
                    motor_controller_test->speed_pid.set, motor_controller_test->speed_pid.fdb,
                    motor_controller_test->speed_pid.out);

            /**
             * @brief Referee System Comm
             */
            // sprintf((char *)test_txt, "%d,%d,%d\r\n ", referee_robot_status->robot_id,
            // referee_robot_status->current_HP,
            //         referee_robot_status->maximum_HP);

            uart_tx_trigger_dma(BOARD_HDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                strlen((char *)test_txt) - 1);
        }

        /**
         * @brief IMU
         */
        // printf("%f,%f,%f\n", ins_->Yaw, -ins_->Pitch, ins_->Roll);

        vTaskDelay(PRINT_TIME_MS);
#endif
    }
}

#else // core1

#include "detect_task.h"
#include "dualcore_task.h"

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
