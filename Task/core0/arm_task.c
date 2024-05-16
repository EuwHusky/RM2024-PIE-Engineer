#include "stdio.h"
#include "string.h"

#include "arm_task.h"

#include "arm_control.h"
#include "arm_kinematics.h"
#include "arm_motor.h"

#include "drv_delay.h"

#include "bsp_ma600.h"

#include "algo_data_limiting.h"

#include "drv_dma.h"
#include "hpm_uart_drv.h"
#include "referee_frame_process.h"

#include "INS_task.h"
#include "detect_task.h"

static void arm_init(engineer_scara_arm_s *scara_arm);
static void update_and_execute_grabber(engineer_scara_arm_s *scara_arm);
static void update_mag_encoder_ma600_feedback(engineer_scara_arm_s *scara_arm);

static engineer_scara_arm_s scara_arm;

// 图传链路串口初始化
static void vt_uart_init(void);

#define VT_UART_RX_BUF_LENGHT 99 // 图传链路裁判系统数据接收数组大小

// 图传链路裁判系统数据相关变量和结构
ATTR_PLACE_AT_NONCACHEABLE uint8_t vt_rx_buf[VT_UART_RX_BUF_LENGHT]; // 接收原始数据
volatile bool vt_uart_rx_dma_done = true;                            // dma传输完成标志位
fifo_s_t *vt_uart_fifo = NULL;

uint32_t test_vt = 0;

void vt_referee_dma_isr(void)
{
    fifo_s_puts(vt_uart_fifo, (char *)vt_rx_buf, VT_UART_RX_BUF_LENGHT);
    vt_uart_rx_dma_done = true; // 更新标志位
    detect_hook(VT_REFEREE_DH);
}

void arm_task(void *pvParameters)
{
    rflOsDelayMs(800);

    while (!INS_init_finished)
        rflOsDelayMs(2);
    rflOsDelayMs(30);

    scara_arm.started = false;

    arm_init(&scara_arm);

    scara_arm.started = true;

    vt_uart_init(); // 初始化图传链路串口

    vt_uart_fifo = get_vt_fifo();

    while (1)
    {
        // 接收行为控制任务的指令 控制机械臂
        arm_mode_control(&scara_arm);

        // 抓取机构更新与执行
        update_and_execute_grabber(&scara_arm);

        // 更新磁编码器反馈
        update_mag_encoder_ma600_feedback(&scara_arm);

        // 机械臂模型更新
        arm_model_update_status(&scara_arm);
        arm_model_update_control(&scara_arm);

        // 机械臂电机更新与执行
        arm_motor_update_and_execute(&scara_arm);

        if (vt_uart_rx_dma_done)
        {
            test_vt++;
            vt_uart_rx_dma_done = false;
            refereeUnpackFifoData(VT_REFEREE_LINK);
            uart_rx_trigger_dma(BOARD_HDMA, VT_UART_RX_DMA_CHN, VT_UART,
                                core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)vt_rx_buf),
                                VT_UART_RX_BUF_LENGHT);
        }

        rflOsDelayMs(1);
    }
}

engineer_scara_arm_s *getArmDataPointer(void)
{
    return &scara_arm;
}

bool *getArmStartedFlag(void)
{
    return &scara_arm.started;
}

float getArmTargetDirection(void)
{
    return atan2f(scara_arm.pose_6d[POSE_Y], scara_arm.pose_6d[POSE_X]);
}

float getArmJointsValue(engineer_scara_arm_joints_e joint)
{
    return scara_arm.joints_value[joint];
}

float getArmSetJointsValue(engineer_scara_arm_joints_e joint)
{
    return scara_arm.set_joints_value[joint];
}

float getArmMotorTemperature(engineer_scara_arm_joints_motors_index_e motor_index)
{
    return rflMotorGetTemperature(&scara_arm.joints_motors[motor_index]);
}

bool checkIfArmInDefaultPose(void)
{
    return (fabsf(scara_arm.pose_6d[POSE_X] - OPERATION_MODE_DEFAULT_X) < 0.1f &&
            fabsf(scara_arm.pose_6d[POSE_Y] - OPERATION_MODE_DEFAULT_Y) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm.pose_6d[POSE_Z] - OPERATION_MODE_DEFAULT_Z) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm.pose_6d[POSE_YAW] - OPERATION_MODE_DEFAULT_YAW) < TOLERABLE_ANGLE_DEVIATION &&
            fabsf(scara_arm.pose_6d[POSE_PITCH] - OPERATION_MODE_DEFAULT_PITCH) < TOLERABLE_ANGLE_DEVIATION &&
            fabsf(scara_arm.pose_6d[POSE_ROLL] - OPERATION_MODE_DEFAULT_ROLL) < TOLERABLE_ANGLE_DEVIATION);
}

bool checkIfArmGrabbed(void)
{
    return scara_arm.grabbed;
}

bool checkIfLifterMotorOverheat(void)
{
    if (rflMotorGetTemperature(&scara_arm.joints_motors[MOTOR_JOINT1_LEFT]) > 65 ||
        rflMotorGetTemperature(&scara_arm.joints_motors[MOTOR_JOINT1_RIGHT]) > 65)
        return true;

    return false;
}

bool checkIfStoragePushInOverTime(void)
{
    return scara_arm.storage_push_overtime_timer > 250;
}

bool *getArmResetStatus(void)
{
    return &scara_arm.reset_success;
}

bool *getArmMoveHomingStatue(void)
{
    return &scara_arm.move_homing_success;
}

bool *getArmOperationHomingStatus(void)
{
    return &scara_arm.operation_homing_success;
}

bool *getSilverMiningStatus(void)
{
    return &scara_arm.silver_mining_success;
}

bool *getStoragePushStatus(void)
{
    return &scara_arm.storage_push_success;
}

bool *getStoragePopStatus(void)
{
    return &scara_arm.storage_pop_success;
}

void resetArmPose(void)
{
    for (uint8_t i = 0; i < 6; i++)
        scara_arm.set_pose_6d[i] = scara_arm.pose_6d[i];
}

void ArmReadyToExchangePose(engineer_scara_arm_solution_e solution)
{
    scara_arm.solution = solution;

    scara_arm.set_pose_6d[POSE_X] = 0.65f;
    scara_arm.set_pose_6d[POSE_Z] = ENGINEER_ARM_Z_MAX_DISTANCE / 2.0f;
}

static void arm_init(engineer_scara_arm_s *scara_arm)
{
    memset(scara_arm, 0, sizeof(engineer_scara_arm_s));

    scara_arm->behavior = ENGINEER_BEHAVIOR_DISABLE;
    scara_arm->last_behavior = ENGINEER_BEHAVIOR_DISABLE;

    resetArmStartUpStatus(scara_arm->start_up_status);
    scara_arm->reset_success = false;
    scara_arm->move_homing_success = false;
    scara_arm->operation_homing_success = false;
    scara_arm->silver_mining_success = false;
    scara_arm->storage_push_success = false;
    scara_arm->storage_pop_success = false;

    scara_arm->joint_6_homing_timer = 0;
    scara_arm->silver_mining_step = 0;
    scara_arm->silver_mining_grab_end_timer = 0;
    scara_arm->gold_mining_step = 0;
    scara_arm->storage_push_step = 0;
    scara_arm->storage_push_end_timer = 0;
    scara_arm->storage_push_overtime_timer = 0;
    scara_arm->storage_pop_step = 0;

    arm_model_init(scara_arm);

    arm_rm_motor_can_init();
    while (detect_error(ARM_JOINT_1_L_DH) || detect_error(ARM_JOINT_1_R_DH) || /* detect_error(ARM_JOINT_4_DH) || */
           detect_error(ARM_JOINT_56_L_DH) || detect_error(ARM_JOINT_56_R_DH))
        rflOsDelayMs(10);
    arm_motor_init(scara_arm);
    scara_arm->joint_23_front_motor_angle_offset = 0.0f;

    scara_arm->rc = getRemoteControlPointer();

    scara_arm->customer_controller = getCustomerControllerData();
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[0], 0.02f, 0.98f);
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[1], 0.02f, 0.98f);
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[2], 0.01f, 0.99f);
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[3], 0.03f, 0.97f);
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[4], 0.03f, 0.97f);
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[5], 0.03f, 0.97f);

    // 气泵
    HPM_IOC->PAD[IOC_PAD_PA25].FUNC_CTL = IOC_PA25_FUNC_CTL_GPIO_A_25;
    gpio_set_pin_output_with_initial(HPM_GPIO0, ENGINEER_ARM_PUMP_GPIO_PORT, ENGINEER_ARM_PUMP_GPIO_PIN, 0);

    // 卸力阀
    HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_GPIO_A_31;
    gpio_set_pin_output_with_initial(HPM_GPIO0, ENGINEER_ARM_VALVE_GPIO_PORT, ENGINEER_ARM_VALVE_GPIO_PIN, 1);

    // 气压传感器
    HPM_IOC->PAD[IOC_PAD_PA05].FUNC_CTL = IOC_PA05_FUNC_CTL_GPIO_A_05;
    gpio_set_pin_input(HPM_GPIO0, ENGINEER_ARM_SENSOR_GPIO_PORT, ENGINEER_ARM_SENSOR_GPIO_PIN);

    MA600_init();
    rflSlidingWindowFilterInit(&scara_arm->joint_6_encoder_angle_filter, 14, 2);
}

static void update_and_execute_grabber(engineer_scara_arm_s *scara_arm)
{
    // 更新

    scara_arm->grabbed = (gpio_read_pin(HPM_GPIO0, ENGINEER_ARM_SENSOR_GPIO_PORT, ENGINEER_ARM_SENSOR_GPIO_PIN) == 0);

    // 执行

    if (scara_arm->behavior != ENGINEER_BEHAVIOR_DISABLE)
    {
        gpio_write_pin(HPM_GPIO0, ENGINEER_ARM_PUMP_GPIO_PORT, ENGINEER_ARM_PUMP_GPIO_PIN,
                       getArmGrabMode() ? 1 : 0); // 气泵
        gpio_write_pin(HPM_GPIO0, ENGINEER_ARM_VALVE_GPIO_PORT, ENGINEER_ARM_VALVE_GPIO_PIN,
                       getArmGrabMode() ? 0 : 1); // 电磁阀
    }
}

static void update_mag_encoder_ma600_feedback(engineer_scara_arm_s *scara_arm)
{
    bool is_error = false;

    if (scara_arm->joint_6_encoder_value = MA600_read_with_check(&is_error), is_error == false)
    {
        scara_arm->joint_6_encoder_angle = (float)scara_arm->joint_6_encoder_value * 0.005493248f;
    }
}

/*图传链路串口初始化**/
static void vt_uart_init(void)
{
    uart_config_t config = {0}; // 串口配置
    board_init_uart(VT_UART);
    uart_default_config(VT_UART, &config);                    // 填充默认配置
    config.fifo_enable = true;                                // 使能FIFO
    config.dma_enable = true;                                 // 使能DMA
    config.baudrate = VT_BAUDRATE;                            // 设置波特率
    config.src_freq_in_hz = clock_get_frequency(VT_UART_CLK); // 获得时钟频率
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
    if (uart_init(VT_UART, &config) != status_success)
    {
        printf("failed to initialize uart\n");
        while (1)
        {
        }
    }
    intc_m_enable_irq_with_priority(BOARD_HDMA_IRQ, 1);
    dmamux_config(BOARD_DMAMUX, VT_UART_RX_DMAMUX_CHN, VT_UART_RX_DMA_REQ, true);

    rflDmaAddCallbackFunction(BOARD_HDMA, VT_UART_RX_DMA_CHN, vt_referee_dma_isr);
}
