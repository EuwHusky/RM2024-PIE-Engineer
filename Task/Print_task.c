#include "print_task.h"

#include "hpm_uart_drv.h"

#include "FreeRTOS.h"
#include "task.h"

#define PRINT_ERROR (false) // 是否输出异常
#define PRINT_TIME_MS 10    // 输出数据的周期

#if !BOARD_RUNNING_CORE // core0

#include "hpm_can_drv.h"

#include "drv_dma.h"

#include "referee.h"
#include "remote_control.h"

#include "INS_task.h"
#include "detect_task.h"
#include "dualcore_task.h"

#include "arm_task.h"
#include "behavior_task.h"
#include "chassis_task.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "storage_task.h"

INS_t *ins_;
transmit_data_021 *data_send;
transmit_data_120 *data_read;

const engineer_behavior_manager_s *behavior_print;

engineer_scara_arm_s *arm_print;
engineer_chassis_s *chassis_print;
const engineer_gimbal_s *gimbal_print;
const engineer_storage_s *storage_print;

const remote_control_s *rc_print;

const game_robot_HP_t *referee_robot_hp;
const robot_status_t *referee_robot_status;
const custom_robot_data_t *customer_controller;
const vt_link_remote_control_t *vt_link_rc_p;

ATTR_PLACE_AT_NONCACHEABLE uint8_t test_txt[256];
volatile bool print_uart_tx_dma_done = true; // dma传输完成标志位

rm_motor_s *motor_0_driver_test;
rm_motor_s *motor_1_driver_test;

rfl_motor_pid_controller_s *motor_0_controller_test;
rfl_motor_pid_controller_s *motor_1_controller_test;

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

    arm_print = getArmDataPointer();
    chassis_print = getChassisDataPointer();
    gimbal_print = getGimbalDataPointer();
    storage_print = getStorageDataPointer();

    rc_print = getRemoteControlPointer();

    referee_robot_hp = getRobotHp();
    referee_robot_status = getRobotStatus();
    customer_controller = getCustomerControllerData();
    vt_link_rc_p = getVtLinkRemoteControlData();

    while (true)
    {
#if !PRINT_ERROR

        // motor_0_driver_test = (rm_motor_s *)arm_print->joints_motors[MOTOR_JOINT1_LEFT].driver;
        // motor_1_driver_test = (rm_motor_s *)arm_print->joints_motors[MOTOR_JOINT1_RIGHT].driver;

        // motor_0_controller_test = (rfl_motor_pid_controller_s
        // *)arm_print->joints_motors[MOTOR_JOINT1_LEFT].controller; motor_1_controller_test =
        // (rfl_motor_pid_controller_s *)arm_print->joints_motors[MOTOR_JOINT1_RIGHT].controller;

        if (print_uart_tx_dma_done)
        {
            print_uart_tx_dma_done = false;

            /**
             * @brief Storage
             */
            // sprintf((char *)test_txt, "%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
            //         behavior_print->behavior == 8 ? 1 : (behavior_print->behavior == 9 ? 2 : 0),
            //         storage_print->storage_used_num, storage_print->current_target_slot,
            //         storage_print->storage_slot_status[STORAGE_BACK],
            //         storage_print->storage_slot_status[STORAGE_FRONT],
            //         storage_print->storage_slot_needed[STORAGE_BACK],
            //         storage_print->storage_slot_needed[STORAGE_FRONT], arm_print->storage_push_step,
            //         arm_print->storage_pop_step);
            // sprintf((char *)test_txt, "%d,%d,%d\r\n", storage_print->latest_nugget_type_to_grab,
            //         storage_print->slot_nugget_type[0], storage_print->slot_nugget_type[1]);

            /**
             * @brief Chassis
             */
            // sprintf((char *)test_txt, "%f,%f,%f,%f,%f,%f,%f,%f\r\n", chassis_print->follow_offset,
            //         chassis_print->set_control_angle.deg, chassis_print->model.control_vector.deg,
            //         chassis_print->model.set_forward_vector.deg, chassis_print->model.forward_vector_->deg,
            //         chassis_print->model.speed_vector_[0], chassis_print->model.speed_vector_[1],
            //         chassis_print->model.speed_vector_[2]);

            /**
             * @brief Gimbal
             */
            sprintf((char *)test_txt, "%f,%f,%f,%f\r\n", gimbal_print->yaw_motor.torque_,
                    gimbal_print->yaw_motor.speed_, gimbal_print->yaw_motor.set_angle_.deg,
                    gimbal_print->yaw_motor.angle_.deg);

            /**
             * @brief Scara Arm
             */
            /*设定/测量位姿*/
            // sprintf((char *)test_txt, "%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\r\n",
            //         arm_print->set_pose_6d[POSE_X], arm_print->set_pose_6d[POSE_Y], arm_print->set_pose_6d[POSE_Z],
            //         arm_print->set_pose_6d[POSE_YAW], arm_print->set_pose_6d[POSE_PITCH],
            //         arm_print->set_pose_6d[POSE_ROLL], arm_print->pose_6d[POSE_X], arm_print->pose_6d[POSE_Y],
            //         arm_print->pose_6d[POSE_Z], arm_print->pose_6d[POSE_YAW], arm_print->pose_6d[POSE_PITCH],
            //         arm_print->pose_6d[POSE_ROLL]);
            // sprintf((char *)test_txt, "%f,%f,%f,%f,%f,%f\r\n", arm_print->joints_value[0],
            //         arm_print->joints_value[1] * RADIAN_TO_DEGREE_FACTOR,
            //         arm_print->joints_value[2] * RADIAN_TO_DEGREE_FACTOR,
            //         arm_print->joints_value[3] * RADIAN_TO_DEGREE_FACTOR,
            //         arm_print->joints_value[4] * RADIAN_TO_DEGREE_FACTOR,
            //         arm_print->joints_value[5] * RADIAN_TO_DEGREE_FACTOR);
            // sprintf((char *)test_txt, "%d,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f\r\n", behavior_print->behavior,
            //         arm_print->set_joints_value[JOINT_2] * RADIAN_TO_DEGREE_FACTOR,
            //         arm_print->set_joints_value[JOINT_3] * RADIAN_TO_DEGREE_FACTOR,
            //         arm_print->set_joints_value[JOINT_4] * RADIAN_TO_DEGREE_FACTOR,
            //         arm_print->joints_value[JOINT_2] * RADIAN_TO_DEGREE_FACTOR,
            //         arm_print->joints_value[JOINT_3] * RADIAN_TO_DEGREE_FACTOR,
            //         arm_print->joints_value[JOINT_4] * RADIAN_TO_DEGREE_FACTOR);
            /*测量位姿及关节4平面坐标*/
            // sprintf((char *)test_txt, "%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f\r\n",
            //         arm_print->pose_6d[0], arm_print->pose_6d[1], arm_print->pose_6d[2], arm_print->pose_6d[3],
            //         arm_print->pose_6d[4], arm_print->pose_6d[5], arm_print->printer[0], arm_print->printer[1],
            //         arm_print->printer[2]);
            /*银矿动作调试*/
            // sprintf((char *)test_txt, "%d,%d,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\r\n",
            //         arm_print->silver_mining_step, behavior_print->silver_target, arm_print->set_pose_6d[0],
            //         arm_print->pose_6d[0], arm_print->set_pose_6d[1], arm_print->pose_6d[1],
            //         arm_print->set_pose_6d[2], arm_print->pose_6d[2]);
            /*金矿动作调试*/
            // sprintf((char *)test_txt, "%d,%d,%f,%f\r\n", behavior_print->behavior, arm_print->gold_mining_step,
            //         arm_print->set_joints_value[JOINT_1], arm_print->joints_value[JOINT_1]);
            /*存/取矿动作调试*/
            // sprintf((char *)test_txt,
            //         "%d,%d,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\r\n",
            //         arm_print->storage_push_step, arm_print->storage_pop_step, arm_print->set_pose_6d[0],
            //         arm_print->set_pose_6d[1], arm_print->set_pose_6d[2], arm_print->set_pose_6d[3],
            //         arm_print->set_pose_6d[4], arm_print->set_pose_6d[5], arm_print->pose_6d[0],
            //         arm_print->pose_6d[1], arm_print->pose_6d[2], arm_print->pose_6d[3], arm_print->pose_6d[4],
            //         arm_print->pose_6d[5]);
            /*自定义控制器调试*/
            // sprintf((char *)test_txt, "%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\r\n",
            //         arm_print->cc_pose_6d[0], arm_print->cc_pose_6d[1], arm_print->cc_pose_6d[2],
            //         arm_print->cc_pose_6d[3], arm_print->cc_pose_6d[4], arm_print->cc_pose_6d[5],
            //         arm_print->set_pose_6d[0], arm_print->set_pose_6d[1], arm_print->set_pose_6d[2],
            //         arm_print->set_pose_6d[3], arm_print->set_pose_6d[4], arm_print->set_pose_6d[5]);
            /*电机温度*/
            // sprintf((char *)test_txt, "%d,%d,%7.4f,%7.4f,%7.4f,%7.4f\r\n", detect_error(ARM_JOINT_2_DH),
            //         detect_error(ARM_JOINT_3_DH),
            //         rflMotorGetTemperature(&arm_print->joints_motors[MOTOR_JOINT23_BACK]),
            //         rflMotorGetTemperature(&arm_print->joints_motors[MOTOR_JOINT23_FRONT]),
            //         rflMotorGetTemperature(&arm_print->joints_motors[MOTOR_JOINT1_LEFT]),
            //         rflMotorGetTemperature(&arm_print->joints_motors[MOTOR_JOINT1_RIGHT]));
            /*磁编码器*/
            // sprintf((char *)test_txt, "%d,%f,%f,%d,%f\r\n", arm_print->start_up_status,
            //         arm_print->set_joints_value[JOINT_6], arm_print->joints_value[JOINT_6],
            //         arm_print->joint_6_encoder_value, arm_print->joint_6_encoder_angle);

            /**
             * @brief Motor
             */
            // sprintf((char *)test_txt, "%f,%f,%f,%f,%f,%f\r\n", motor_controller_test->angle_pid.set,
            //         motor_controller_test->angle_pid.fdb, motor_controller_test->angle_pid.out,
            //         motor_controller_test->speed_pid.set, motor_controller_test->speed_pid.fdb,
            //         motor_controller_test->speed_pid.out);
            // sprintf((char *)test_txt, "%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f,%7.4f\r\n",
            //         arm_print->joints_motors[MOTOR_JOINT1_LEFT].set_angle_.deg,
            //         arm_print->joints_motors[MOTOR_JOINT1_LEFT].angle_.deg,
            //         arm_print->joints_motors[MOTOR_JOINT1_LEFT].speed_,
            //         arm_print->joints_motors[MOTOR_JOINT1_LEFT].torque_,
            //         arm_print->joints_motors[MOTOR_JOINT1_RIGHT].set_angle_.deg,
            //         arm_print->joints_motors[MOTOR_JOINT1_RIGHT].angle_.deg,
            //         arm_print->joints_motors[MOTOR_JOINT1_RIGHT].speed_,
            //         arm_print->joints_motors[MOTOR_JOINT1_RIGHT].torque_);

            /**
             * @brief Referee System Comm
             */
            // sprintf((char *)test_txt, "===\r\n%d,%d,%d\r\n%d,%d\r\n", referee_robot_status->robot_id,
            //         referee_robot_status->current_HP, referee_robot_status->maximum_HP,
            //         vt_link_rc_p->left_button_down, vt_link_rc_p->right_button_down);
            // sprintf((char *)test_txt, "%f,%f,%f,%f,%f,%f,%d\r\n", customer_controller->pose[0],
            //         customer_controller->pose[1], customer_controller->pose[2], customer_controller->pose[3],
            //         customer_controller->pose[4], customer_controller->pose[5], customer_controller->key);

            /**
             * @brief Remote Control
             */
            // sprintf((char *)test_txt, "%d,%d,%d,%d,%d,%d\r\n", getRcMouseX(), getRcMouseY(), getRcMouseZ(),
            //         checkIsRcKeyPressed(RC_F), checkIfRcKeyFallingEdgeDetected(RC_F),
            //         checkIfRcKeyRisingEdgeDetected(RC_F));
            // sprintf((char *)test_txt, "%d,%d,%d,%d,%d\r\n", test_pm, test_vt, rc_print->use_vt_link_control,
            //         detect_error(DBUS_DH), detect_error(VT_REFEREE_DH));
            // sprintf((char *)test_txt, "%d,%d,%d,%d,%d,%d,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%d\r\n",
            //         rc_print->vt_link_data->mouse_x, rc_print->vt_link_data->mouse_y,
            //         rc_print->vt_link_data->mouse_z, rc_print->vt_link_data->left_button_down,
            //         rc_print->vt_link_data->right_button_down, rc_print->vt_link_data->keyboard_value,
            //         rc_print->cc_data->pose[0], rc_print->cc_data->pose[1], rc_print->cc_data->pose[2],
            //         rc_print->cc_data->pose[3], rc_print->cc_data->pose[4], rc_print->cc_data->pose[5],
            //         rc_print->cc_data->key);

            /**
             * @brief Behavior Manager
             */
            // sprintf((char *)test_txt, "%d,%d,%d,%d,%d,%d\r\n", behavior_print->behavior,
            // behavior_print->last_behavior,
            //         *behavior_print->arm_reset_success, *behavior_print->gimbal_reset_success,
            //         *behavior_print->arm_move_homing_success, *behavior_print->arm_operation_homing_success);
            // sprintf((char *)test_txt, "%d,%d,%d,%d,%f,%f,%f,%f,%f\r\n", behavior_print->behavior,
            //         behavior_print->last_behavior, *behavior_print->arm_operation_homing_success,
            //         *behavior_print->gimbal_operation_homing_success, gimbal_print->set_gimbal_angle.deg,
            //         gimbal_print->gimbal_angle.deg, rflMotorGetAngle(&gimbal_print->yaw_motor,
            //         RFL_ANGLE_FORMAT_DEGREE), getArmSetJointsValue(JOINT_2) * RADIAN_TO_DEGREE_FACTOR,
            //         getArmJointsValue(JOINT_2) * RADIAN_TO_DEGREE_FACTOR);
            // sprintf((char *)test_txt, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
            //         can_is_primary_transmit_buffer_full(HPM_CAN0), can_is_secondary_transmit_buffer_full(HPM_CAN0),
            //         can_is_in_bus_off_mode(HPM_CAN0), can_is_primary_transmit_buffer_full(HPM_CAN1),
            //         can_is_secondary_transmit_buffer_full(HPM_CAN1), can_is_in_bus_off_mode(HPM_CAN1),
            //         can_is_primary_transmit_buffer_full(HPM_CAN2), can_is_secondary_transmit_buffer_full(HPM_CAN2),
            //         can_is_in_bus_off_mode(HPM_CAN2), can_is_primary_transmit_buffer_full(HPM_CAN3),
            //         can_is_secondary_transmit_buffer_full(HPM_CAN3), can_is_in_bus_off_mode(HPM_CAN3),
            //         (uint8_t)rflMotorGetTemperature(&arm_print->joints_motors[MOTOR_JOINT1_LEFT]));
            // sprintf((char *)test_txt, "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\r\n",
            // behavior_print->motor_failure_detected,
            //         detect_error(ARM_JOINT_1_L_DH), detect_error(ARM_JOINT_1_R_DH), detect_error(ARM_JOINT_2_DH),
            //         detect_error(ARM_JOINT_3_DH), detect_error(ARM_JOINT_4_DH), detect_error(ARM_JOINT_56_L_DH),
            //         detect_error(ARM_JOINT_56_R_DH), detect_error(CHASSIS_MOTOR_0_DH),
            //         detect_error(CHASSIS_MOTOR_1_DH), detect_error(CHASSIS_MOTOR_2_DH),
            //         detect_error(CHASSIS_MOTOR_3_DH));

            uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                strlen((char *)test_txt));
        }

        vTaskDelay(PRINT_TIME_MS);

#else
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
                // case DUAL_COMM_DH: // 双核
                //     sprintf((char *)test_txt, "核间通信异常\n");
                //     uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                //                         core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                //                         strlen((char *)test_txt));
                //     vTaskDelay(5);
                //     break;
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
                case ARM_JOINT_2_DH: // 机械臂电机
                    sprintf((char *)test_txt, "关节2电机异常\n");
                    uart_tx_trigger_dma(BOARD_XDMA, BOARD_UART6_TX_DMA_CHN, BOARD_UART6,
                                        core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)test_txt),
                                        strlen((char *)test_txt));
                    vTaskDelay(5);
                    break;
                case ARM_JOINT_3_DH: // 机械臂电机
                    sprintf((char *)test_txt, "关节3电机异常\n");
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

        vTaskDelay(100);

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
