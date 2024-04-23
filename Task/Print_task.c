#include "print_task.h"

#include "hpm_uart_drv.h"

#include "FreeRTOS.h"
#include "task.h"

#define PRINT_ERROR (false) // 是否输出异常
#define PRINT_TIME_MS 10    // 输出数据的周期

#if !BOARD_RUNNING_CORE // core0

#include "drv_dma.h"

#include "referee.h"
#include "remote_control.h"

#include "INS_task.h"
#include "detect_task.h"
#include "dualcore_task.h"

#include "arm_task.h"
#include "behavior_task.h"
#include "chassis_task.h"
#include "gimbal_task.h"

INS_t *ins_;
transmit_data_021 *data_send;
transmit_data_120 *data_read;

const engineer_behavior_manager_s *behavior_print;

engineer_scara_arm_s *arm_data;
engineer_chassis_s *chassis_data;
const engineer_gimbal_s *gimbal_print;

const remote_control_s *print_rc_pointer;

const game_robot_HP_t *referee_robot_hp;
const robot_status_t *referee_robot_status;
const custom_robot_data_t *customer_controller;
const vt_link_remote_control_t *vt_link_rc_p;

ATTR_PLACE_AT_NONCACHEABLE uint8_t test_txt[512];
volatile bool print_uart_tx_dma_done = true; // dma传输完成标志位

rm_motor_s *motor_0_driver_test;
rm_motor_s *motor_1_driver_test;

rfl_motor_pid_controller_s *motor_0_controller_test;
rfl_motor_pid_controller_s *motor_1_controller_test;

extern uint32_t fuck_pm;
extern uint32_t fuck_vt;

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
    intc_m_enable_irq_with_priority(BOARD_XDMA_IRQ, 1);
    dmamux_config(BOARD_DMAMUX, BOARD_UART6_TX_DMAMUX_CHN, BOARD_UART6_TX_DMA_REQ, true);

    rflDmaAddCallbackFunction(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, print_dma_isr);

    ins_ = get_INS_data_point();
    data_read = get_data_120_point();
    data_send = get_data_021_point();

    behavior_print = getEngineerBehaviorManagerPointer();

    arm_data = getArmDataPointer();
    chassis_data = getChassisDataPointer();
    gimbal_print = getGimbalDataPointer();
    print_rc_pointer = getRemoteControlPointer();

    referee_robot_hp = getRobotHp();
    referee_robot_status = getRobotStatus();
    customer_controller = getCustomerControllerData();
    vt_link_rc_p = getVtLinkRemoteControlData();

    while (true)
    {
#if PRINT_ERROR
        sprintf((char *)test_txt, "========================\n");
        uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                            core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                            strlen((char *)test_txt));
        vTaskDelay(5);
        for (uint8_t i = DUAL_COMM_DH; i < DETECT_ERROR_LIST_LENGHT; i++)
        {
            if (detect_error(i))
                switch (i)
                {
                case DUAL_COMM_DH: // 双核
                    sprintf((char *)test_txt, "核间通信异常\n");
                    uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                        strlen((char *)test_txt));
                    vTaskDelay(5);
                    break;
                case PM_REFEREE_DH: // 电管
                    sprintf((char *)test_txt, "电管串口异常\n");
                    uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                        strlen((char *)test_txt));
                    vTaskDelay(5);
                    break;
                case VT_REFEREE_DH: // 图传
                    sprintf((char *)test_txt, "图传串口异常\n");
                    uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                        strlen((char *)test_txt));
                    vTaskDelay(5);
                    break;
                case DBUS_DH: // 遥控器
                    sprintf((char *)test_txt, "遥控器串口异常\n");
                    uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                        strlen((char *)test_txt));
                    vTaskDelay(5);
                    break;
                case ARM_JOINT_1_L_DH: // 机械臂电机
                    sprintf((char *)test_txt, "关节1左电机异常\n");
                    uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                        strlen((char *)test_txt));
                    vTaskDelay(5);
                    break;
                case ARM_JOINT_1_R_DH: // 机械臂电机
                    sprintf((char *)test_txt, "关节1右电机异常\n");
                    uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                        strlen((char *)test_txt));
                    vTaskDelay(5);
                    break;
                case ARM_JOINT_4_DH: // 机械臂电机
                    sprintf((char *)test_txt, "关节4电机异常\n");
                    uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                        strlen((char *)test_txt));
                    vTaskDelay(5);
                    break;
                case ARM_JOINT_56_L_DH: // 机械臂电机
                    sprintf((char *)test_txt, "关节56左电机异常\n");
                    uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                        strlen((char *)test_txt));
                    vTaskDelay(5);
                    break;
                case ARM_JOINT_56_R_DH: // 机械臂电机
                    sprintf((char *)test_txt, "关节56右电机异常\n");
                    uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                        strlen((char *)test_txt));
                    vTaskDelay(5);
                    break;
                case CHASSIS_MOTOR_0_DH: // 底盘电机0 - 左前
                    sprintf((char *)test_txt, "底盘电机0异常\n");
                    uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                        strlen((char *)test_txt));
                    vTaskDelay(5);
                    break;
                case CHASSIS_MOTOR_1_DH: // 底盘电机1 - 左后
                    sprintf((char *)test_txt, "底盘电机1异常\n");
                    uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                        strlen((char *)test_txt));
                    vTaskDelay(5);
                    break;
                case CHASSIS_MOTOR_2_DH: // 底盘电机2 - 右后
                    sprintf((char *)test_txt, "底盘电机2异常\n");
                    uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                        strlen((char *)test_txt));
                    vTaskDelay(5);
                    break;
                case CHASSIS_MOTOR_3_DH: // 底盘电机3 - 右前
                    sprintf((char *)test_txt, "底盘电机3异常\n");
                    uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                        strlen((char *)test_txt));
                    vTaskDelay(5);
                    break;
                }
        }

        vTaskDelay(500);

#else

        motor_0_driver_test = (rm_motor_s *)arm_data->joints_motors[MOTOR_JOINT1_LEFT].driver;
        motor_1_driver_test = (rm_motor_s *)arm_data->joints_motors[MOTOR_JOINT1_RIGHT].driver;

        motor_0_controller_test = (rfl_motor_pid_controller_s *)arm_data->joints_motors[MOTOR_JOINT1_LEFT].controller;
        motor_1_controller_test = (rfl_motor_pid_controller_s *)arm_data->joints_motors[MOTOR_JOINT1_RIGHT].controller;

        if (print_uart_tx_dma_done)
        {
            print_uart_tx_dma_done = false;

            /**
             * @brief Chassis
             */
            // sprintf((char *)test_txt, "%f,%f,%f,%f,%f,%f,%f,%f\r\n", chassis_data->follow_offset,
            //         chassis_data->set_control_angle.deg, chassis_data->model.control_vector.deg,
            //         chassis_data->model.set_forward_vector.deg, chassis_data->model.forward_vector_->deg,
            //         chassis_data->model.speed_vector_[0], chassis_data->model.speed_vector_[1],
            //         chassis_data->model.speed_vector_[2]);

            /**
             * @brief Gimbal
             */
            // sprintf((char *)test_txt, "%d,%d,%d,%d\r\n", gimbal_print->pitch_pwm_clk_freq,
            // gimbal_print->pitch_pwm_freq,
            //         gimbal_print->pitch_pwm_reload, gimbal_print->pitch_pwm_compare);

            /**
             * @brief Scara Arm
             */
            // sprintf((char *)test_txt, "%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\r\n",
            //         arm_data->set_pose_6d[0], arm_data->set_pose_6d[1], arm_data->set_pose_6d[2],
            //         arm_data->set_pose_6d[3], arm_data->set_pose_6d[4], arm_data->set_pose_6d[5],
            //         arm_data->pose_6d[0], arm_data->pose_6d[1], arm_data->pose_6d[2], arm_data->pose_6d[3],
            //         arm_data->pose_6d[4], arm_data->pose_6d[5]);
            // sprintf((char *)test_txt, "%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\r\n", arm_data->cc_pose_6d[0],
            //         arm_data->cc_pose_6d[1], arm_data->cc_pose_6d[2], arm_data->customer_controller->pose[0],
            //         arm_data->customer_controller->pose[1], arm_data->customer_controller->pose[2]);
            // sprintf((char *)test_txt, "%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\r\n", arm_data->cc_pose_6d[3],
            //         arm_data->cc_pose_6d[4], arm_data->cc_pose_6d[5], arm_data->customer_controller->pose[3],
            //         arm_data->customer_controller->pose[4], arm_data->customer_controller->pose[5]);
            // sprintf((char *)test_txt, "%d,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\r\n",
            //         arm_data->start_up_status, arm_data->set_joints_value[4] * RADIAN_TO_DEGREE_FACTOR,
            //         arm_data->joints_value[4] * RADIAN_TO_DEGREE_FACTOR,
            //         arm_data->joints_motors[MOTOR_JOINT56_LEFT].control_output_,
            //         arm_data->joints_motors[MOTOR_JOINT56_LEFT].torque_,
            //         arm_data->joints_motors[MOTOR_JOINT56_LEFT].set_speed_,
            //         arm_data->joints_motors[MOTOR_JOINT56_LEFT].speed_,
            //         arm_data->joints_motors[MOTOR_JOINT56_RIGHT].control_output_,
            //         arm_data->joints_motors[MOTOR_JOINT56_RIGHT].torque_,
            //         arm_data->joints_motors[MOTOR_JOINT56_RIGHT].set_speed_,
            //         arm_data->joints_motors[MOTOR_JOINT56_RIGHT].speed_);
            sprintf((char *)test_txt, "%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\r\n",
                    arm_data->set_joints_value[JOINT_1], arm_data->joints_value[JOINT_1],
                    arm_data->joints_motors[MOTOR_JOINT1_LEFT].set_angle_.deg,
                    arm_data->joints_motors[MOTOR_JOINT1_LEFT].angle_.deg,
                    arm_data->joints_motors[MOTOR_JOINT1_LEFT].torque_,
                    arm_data->joints_motors[MOTOR_JOINT1_LEFT].speed_,
                    arm_data->joints_motors[MOTOR_JOINT1_RIGHT].set_angle_.deg,
                    arm_data->joints_motors[MOTOR_JOINT1_RIGHT].angle_.deg,
                    arm_data->joints_motors[MOTOR_JOINT1_RIGHT].torque_,
                    arm_data->joints_motors[MOTOR_JOINT1_RIGHT].speed_/*  motor_0_controller_test->angle_pid.set,
                    motor_0_controller_test->angle_pid.fdb, motor_0_controller_test->angle_pid.out,
                    motor_0_controller_test->speed_pid.set, motor_0_controller_test->speed_pid.fdb,
                    motor_0_controller_test->speed_pid.out */);
            // sprintf((char *)test_txt, "%f,%f,%f\r\n", arm_data->printer[0], arm_data->printer[1],
            // arm_data->printer[2]);

            /**
             * @brief Motor PID
             */
            // sprintf((char *)test_txt, "%f,%f,%f,%f,%f,%f\r\n", motor_controller_test->angle_pid.set,
            //         motor_controller_test->angle_pid.fdb, motor_controller_test->angle_pid.out,
            //         motor_controller_test->speed_pid.set, motor_controller_test->speed_pid.fdb,
            //         motor_controller_test->speed_pid.out);

            /**
             * @brief Referee System Comm
             */
            // sprintf((char *)test_txt, "===\r\n%d,%d,%d\r\n%d,%d\r\n", referee_robot_status->robot_id,
            //         referee_robot_status->current_HP, referee_robot_status->maximum_HP,
            //         vt_link_rc_p->left_button_down, vt_link_rc_p->right_button_down);
            // sprintf((char *)test_txt, "%f,%f,%f,%f,%f,%f,%d\r\n", customer_controller->pose[0],
            //         customer_controller->pose[1], customer_controller->pose[2], customer_controller->pose[3],
            //         customer_controller->pose[4], customer_controller->pose[5], customer_controller->key);
            // sprintf((char *)test_txt, "%d,%d,%d,%d,%d\r\n", fuck_pm, fuck_vt, vt_link_rc_p->mouse_x,
            //         vt_link_rc_p->mouse_y, vt_link_rc_p->keyboard_value);

            /**
             * @brief Remote Control
             */
            // sprintf((char *)test_txt, "%d,%d,%d,%d,%d,%d\r\n", getRcMouseX(), getRcMouseY(), getRcMouseZ(),
            //         checkIsRcKeyPressed(RC_F), checkIfRcKeyFallingEdgeDetected(RC_F),
            //         checkIfRcKeyRisingEdgeDetected(RC_F));

            /**
             * @brief Behavior Manager
             */
            // sprintf((char *)test_txt, "%d,%d,%d,%d,%d,%d\r\n", behavior_print->behavior,
            // behavior_print->last_behavior,
            //         *behavior_print->arm_reset_success, *behavior_print->gimbal_reset_success,
            //         *behavior_print->arm_move_homing_success, *behavior_print->arm_operation_homing_success);

            uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                strlen((char *)test_txt));
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
        // for (uint8_t i = DUAL_COMM_DH; i < DETECT_ERROR_LIST_LENGHT; i++)
        // {
        //     if (detect_error(i))
        //         switch (i)
        //         {
        //         case DUAL_COMM_DH: // 双核
        //             printf("核间通信异常\n");
        //             break;
        //         }
        // }
        // printf("--------------------\n");
        // vTaskDelay(2000);
#else

        // printf("%d,%d,%d,%d,%d\r\n", core0_data_print->rc_data_image.dt7_dr16_data.rc.ch[0],
        //        core0_data_print->rc_data_image.dt7_dr16_data.rc.ch[1],
        //        core0_data_print->rc_data_image.dt7_dr16_data.rc.ch[2],
        //        core0_data_print->rc_data_image.dt7_dr16_data.rc.ch[3],
        //        core0_data_print->rc_data_image.dt7_dr16_data.rc.ch[4]);

        vTaskDelay(PRINT_TIME_MS);
#endif
    }
}

#endif
