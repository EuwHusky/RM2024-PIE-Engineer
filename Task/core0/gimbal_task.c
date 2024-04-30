#include "gimbal_task.h"

#include "hpm_pwm_drv.h"

#include "drv_can.h"
#include "drv_delay.h"

#include "algo_data_limiting.h"

#include "INS_task.h"
#include "chassis_task.h"
#include "detect_task.h"

static void gimbal_init(engineer_gimbal_s *gimbal);
static void gimbal_mode_control(engineer_gimbal_s *gimbal);
static void gimbal_update_and_execute(engineer_gimbal_s *gimbal);
static void gimbal_no_force_control(engineer_gimbal_s *gimbal);
static void gimbal_reset_control(engineer_gimbal_s *gimbal);
static void gimbal_move_control(engineer_gimbal_s *gimbal);
static void gimbal_operation_control(engineer_gimbal_s *gimbal);
static void gimbal_steer_pitch_normal_control(engineer_gimbal_s *gimbal);
static void gimbal_steer_pitch_reset_control(engineer_gimbal_s *gimbal);
static void gimbal_motor_yaw_can_rx_callback(void);

static engineer_gimbal_s gimbal;

void gimbal_task(void *pvParameters)
{
    rflOsDelayMs(1000);

    while (!INS_init_finished)
        rflOsDelayMs(10);
    rflOsDelayMs(2000);

    gimbal_init(&gimbal);

    while (1)
    {
        gimbal_mode_control(&gimbal);

        gimbal_update_and_execute(&gimbal);

        rflOsDelayMs(4);
    }
}

const engineer_gimbal_s *getGimbalDataPointer(void)
{
    return &gimbal;
}

bool *getGimbalResetStatus(void)
{
    return &gimbal.reset_success;
}

float getGimbalYawAngle(rfl_angle_format_e angle_format)
{
    return rflMotorGetAngle(&gimbal.yaw_motor, angle_format);
}

static void gimbal_init(engineer_gimbal_s *gimbal)
{
    memset(gimbal, 0, sizeof(engineer_gimbal_s));

    gimbal->behavior = ENGINEER_BEHAVIOR_DISABLE;
    gimbal->last_behavior = ENGINEER_BEHAVIOR_DISABLE;

    gimbal->reset_step = ENGINEER_GIMBAL_RESET_STEP_HOMING;
    gimbal->reset_success = false;

    rflAngleUpdate(&gimbal->set_gimbal_angle, RFL_ANGLE_FORMAT_DEGREE, GIMBAL_YAW_START_ANGLE);

    // CAN通信
    rflCanRxMessageBoxAddId(GIMBAL_MOTORS_CAN_ORDINAL, 0x201);
    rflCanRxMessageBoxAddRxCallbackFunc(GIMBAL_MOTORS_CAN_ORDINAL, 0x201, gimbal_motor_yaw_can_rx_callback);

    // YAW电机
    while (detect_error(GIMBAL_MOTOR_YAW_DH))
        rflOsDelayMs(10);
    rfl_motor_config_s motor_config = {0};
    rflMotorGetDefaultConfig(&motor_config, RFL_MOTOR_RM_M2006, RFL_MOTOR_CONTROLLER_PID);
    motor_config.control_period_factor = 4.0f;
    motor_config.angle_pid_kp = ENGINEER_GIMBAL_YAW_RM_M2006_ANGLE_PID_KP;
    motor_config.angle_pid_ki = ENGINEER_GIMBAL_YAW_RM_M2006_ANGLE_PID_KI;
    motor_config.angle_pid_kd = ENGINEER_GIMBAL_YAW_RM_M2006_ANGLE_PID_KD;
    motor_config.angle_pid_max_iout = ENGINEER_GIMBAL_YAW_RM_M2006_ANGLE_PID_MAX_IOUT;
    motor_config.angle_pid_max_out = ENGINEER_GIMBAL_YAW_RM_M2006_ANGLE_PID_MAX_OUT;
    motor_config.speed_pid_kp = ENGINEER_GIMBAL_YAW_RM_M2006_SPEED_PID_KP;
    motor_config.speed_pid_ki = ENGINEER_GIMBAL_YAW_RM_M2006_SPEED_PID_KI;
    motor_config.speed_pid_kd = ENGINEER_GIMBAL_YAW_RM_M2006_SPEED_PID_KD;
    motor_config.speed_pid_max_iout = ENGINEER_GIMBAL_YAW_RM_M2006_SPEED_PID_MAX_IOUT;
    motor_config.speed_pid_max_out = ENGINEER_GIMBAL_YAW_RM_M2006_SPEED_PID_MAX_OUT;
    motor_config.can_ordinal = GIMBAL_MOTORS_CAN_ORDINAL;
    motor_config.master_can_id = 0x201;
    rflMotorInit(&gimbal->yaw_motor, &motor_config);

    // 在IO控制器中配置对应引脚
    HPM_IOC->PAD[IOC_PAD_PC01].FUNC_CTL = IOC_PC01_FUNC_CTL_PWM0_P_0;
    // 设置PWM频率
    gimbal->pitch_pwm_freq = 50;
    // 获得时钟频率
    gimbal->pitch_pwm_clk_freq = clock_get_frequency(clock_mot0); // 200000000
    // 计算reload值
    gimbal->pitch_pwm_reload = gimbal->pitch_pwm_clk_freq / gimbal->pitch_pwm_freq - 1;
    // PWM配置（根据需求配置，此处仅示范用法）
    pwm_config_t pwm_config = {0};
    pwm_get_default_pwm_config(HPM_PWM0, &pwm_config);
    pwm_config.invert_output = true; // 反向输出
    // PWM比较器配置（根据需求配置，此处仅示范用法）
    pwm_cmp_config_t cmp_config = {0};
    pwm_get_default_cmp_config(HPM_PWM0, &cmp_config);
    // 停止计数器
    pwm_stop_counter(HPM_PWM0);
    // 设置RLD寄存器（设置reload）
    pwm_set_reload(HPM_PWM0, 0, gimbal->pitch_pwm_reload);
    // 设置STA寄存器（设置初始计数值）
    pwm_set_start_count(HPM_PWM0, 0, 0);
    // 设置波形
    if (status_success != pwm_setup_waveform(HPM_PWM0, 0, &pwm_config, 0, &cmp_config, 1))
    {
        printf("failed to setup waveform\n");
        while (1)
            ;
    }
    // 锁定影子寄存器
    pwm_issue_shadow_register_lock_event(HPM_PWM0);
    // 开始计数
    pwm_start_counter(HPM_PWM0);

    // 改变占空比
    gimbal->pitch_pwm_compare = ENGINEER_GIMBAL_PITCH_PWM_CONPARE_VALUE_MID;
    pwm_update_raw_cmp_edge_aligned(HPM_PWM0, 0, gimbal->pitch_pwm_compare);
    pwm_enable_output(HPM_PWM0, 0); //

    gimbal->rc = getRemoteControlPointer();
}

static void gimbal_mode_control(engineer_gimbal_s *gimbal)
{
    // 模式切换时修改底盘设定

    gimbal->last_behavior = gimbal->behavior;
    gimbal->behavior = getEngineerCurrentBehavior();

    if (gimbal->last_behavior != gimbal->behavior)
    {
        if (gimbal->behavior == ENGINEER_BEHAVIOR_DISABLE)
        {
            rflMotorSetMode(&gimbal->yaw_motor, RFL_MOTOR_CONTROL_MODE_NO_FORCE);
            pwm_disable_output(HPM_PWM0, 0);
        }
        else if (gimbal->behavior != ENGINEER_BEHAVIOR_DISABLE)
        {
            rflMotorSetMode(&gimbal->yaw_motor, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
            pwm_enable_output(HPM_PWM0, 0);
        }

        if (gimbal->behavior == ENGINEER_BEHAVIOR_RESET)
        {
            rflMotorSetDegAngleLimit(&gimbal->yaw_motor, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_GIMBAL_YAW_INITIAL_MAX_ANGLE,
                                     ENGINEER_GIMBAL_YAW_INITIAL_MIN_ANGLE);

            gimbal->reset_step = ENGINEER_GIMBAL_RESET_STEP_HOMING;
        }
        else if (getEngineerLastBehavior() == ENGINEER_BEHAVIOR_RESET && gimbal->behavior != ENGINEER_BEHAVIOR_RESET)
        {
            rflMotorSetDegAngleLimit(&gimbal->yaw_motor, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_GIMBAL_YAW_MAX_ANGLE,
                                     ENGINEER_GIMBAL_YAW_MIN_ANGLE);
        }
    }

    // 根据不同模式使用不同控制

    if (gimbal->behavior == ENGINEER_BEHAVIOR_RESET)
        rflMotorSetMaxSpeed(&gimbal->yaw_motor, 1.0f);
    else
        rflMotorSetMaxSpeed(&gimbal->yaw_motor, 0.5f);

    switch (gimbal->behavior)
    {
    case ENGINEER_BEHAVIOR_DISABLE:
        gimbal_no_force_control(gimbal);
        break;
    case ENGINEER_BEHAVIOR_RESET:
        gimbal_reset_control(gimbal);
        break;
    case ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING:
    case ENGINEER_BEHAVIOR_MOVE:
        gimbal_move_control(gimbal);
        break;
    case ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING:
    case ENGINEER_BEHAVIOR_AUTO_SILVER_MINING:
    // case ENGINEER_BEHAVIOR_AUTO_GOLD_MINING:
    case ENGINEER_BEHAVIOR_MANUAL_OPERATION:
        gimbal_operation_control(gimbal);
        break;

    default:
        break;
    }
}

static void gimbal_update_and_execute(engineer_gimbal_s *gimbal)
{
    rflMotorSetAngle(&gimbal->yaw_motor, RFL_ANGLE_FORMAT_DEGREE, gimbal->set_gimbal_angle.deg);

    rflMotorUpdateStatus(&gimbal->yaw_motor);
    rflMotorUpdateControl(&gimbal->yaw_motor);
    rflRmMotorControl(GIMBAL_MOTORS_CAN_ORDINAL, GIMBAL_MOTORS_CAN_SLAVE_ID,
                      (int16_t)rflMotorGetOutput(&gimbal->yaw_motor), 0, 0, 0);

    gimbal->pitch_pwm_compare =
        rflUint32Constrain(gimbal->pitch_pwm_compare, ENGINEER_GIMBAL_PITCH_PWM_CONPARE_VALUE_MIN,
                           ENGINEER_GIMBAL_PITCH_PWM_CONPARE_VALUE_MAX);
    pwm_update_raw_cmp_edge_aligned(HPM_PWM0, 0, gimbal->pitch_pwm_compare);
}

static void gimbal_no_force_control(engineer_gimbal_s *gimbal)
{
    rflAngleUpdate(&gimbal->set_gimbal_angle, RFL_ANGLE_FORMAT_DEGREE,
                   rflMotorGetAngle(&gimbal->yaw_motor, RFL_ANGLE_FORMAT_DEGREE));
}

static void gimbal_reset_control(engineer_gimbal_s *gimbal)
{
    // gimbal->reset_success = true; // 只是测试时用来取消归中
    if (gimbal->reset_success == false)
    {
        if (gimbal->reset_step == ENGINEER_GIMBAL_RESET_STEP_HOMING)
        {
            rflAngleUpdate(&gimbal->set_gimbal_angle, RFL_ANGLE_FORMAT_DEGREE,
                           gimbal->set_gimbal_angle.deg + GIMBAL_YAW_HOMING_STEP_ANGLE);

            if (rflMotorGetTorque(&gimbal->yaw_motor) > GIMBAL_YAW_HOMING_TORQUE_THRESHOLD &&
                fabsf(rflMotorGetSpeed(&gimbal->yaw_motor)) < 0.2f)
            {
                rflMotorResetAngle(&gimbal->yaw_motor, RFL_ANGLE_FORMAT_DEGREE, GIMBAL_YAW_HOMING_ANGLE, false);
                gimbal->reset_step = ENGINEER_GIMBAL_RESET_STEP_STARTING;
            }
        }
        else if (gimbal->reset_step == ENGINEER_GIMBAL_RESET_STEP_STARTING)
        {
            rflAngleUpdate(&gimbal->set_gimbal_angle, RFL_ANGLE_FORMAT_DEGREE, -getChassisFollowOffsetMemory());

            if (fabsf(rflMotorGetAngle(&gimbal->yaw_motor, RFL_ANGLE_FORMAT_DEGREE) + getChassisFollowOffsetMemory()) <
                1.0f)
            {
                gimbal->reset_success = true;
            }
        }
    }

    gimbal_steer_pitch_reset_control(gimbal);
}

static void gimbal_move_control(engineer_gimbal_s *gimbal)
{
    rflAngleUpdate(&gimbal->set_gimbal_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_MOVE_BEHAVIOR_GIMBAL_SET_ANGLE);

    gimbal_steer_pitch_normal_control(gimbal);
}

static void gimbal_operation_control(engineer_gimbal_s *gimbal)
{
    rflAngleUpdate(&gimbal->set_gimbal_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_OPERATION_BEHAVIOR_GIMBAL_SET_ANGLE);
}

static void gimbal_steer_pitch_normal_control(engineer_gimbal_s *gimbal)
{
    uint32_t temp_value = 0;

    // DT7 & 键鼠

    temp_value = rflDeadZoneZero(gimbal->rc->dt7_dr16_data.rc.ch[4], 3);
    temp_value = temp_value ? temp_value : (gimbal->rc->mouse_z * 100);

    if (temp_value > 0)
        gimbal->pitch_pwm_compare += (uint32_t)(temp_value);
    else if (temp_value < 0)
        gimbal->pitch_pwm_compare -= (uint32_t)(-temp_value);
}

static void gimbal_steer_pitch_reset_control(engineer_gimbal_s *gimbal)
{
    gimbal->pitch_pwm_compare = ENGINEER_GIMBAL_PITCH_PWM_CONPARE_VALUE_MID;
}

static void gimbal_motor_yaw_can_rx_callback(void)
{
    detect_hook(GIMBAL_MOTOR_YAW_DH);
}
