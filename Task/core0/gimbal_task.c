#include "gimbal_task.h"

#include "hpm_pwm_drv.h"

#include "drv_can.h"
#include "drv_delay.h"

#include "algo_data_limiting.h"

#include "INS_task.h"
#include "arm_control.h"
#include "chassis_task.h"
#include "detect_task.h"

static void gimbal_init(engineer_gimbal_s *gimbal);
static void gimbal_mode_control(engineer_gimbal_s *gimbal);
static void gimbal_update_and_execute(engineer_gimbal_s *gimbal);
static void gimbal_no_force_control(engineer_gimbal_s *gimbal);
static void gimbal_reset_control(engineer_gimbal_s *gimbal);
static void gimbal_move_homing_control(engineer_gimbal_s *gimbal);
static void gimbal_move_control(engineer_gimbal_s *gimbal);
static void gimbal_operation_homing_control(engineer_gimbal_s *gimbal);
static void gimbal_manual_operation_control(engineer_gimbal_s *gimbal);
static void gimbal_auto_operation_control(engineer_gimbal_s *gimbal);
static void gimbal_motor_yaw_can_rx_callback(uint8_t *rx_data);

static engineer_gimbal_s gimbal;

void gimbal_task(void *pvParameters)
{
    rflOsDelayMs(1000);

    while (!INS_init_finished)
        rflOsDelayMs(10);
    rflOsDelayMs(2000);

    gimbal.started = false;

    gimbal_init(&gimbal);

    gimbal.started = true;

    while (1)
    {
        gimbal_mode_control(&gimbal);

        gimbal_update_and_execute(&gimbal);

        rflOsDelayMs(1);
    }
}

const engineer_gimbal_s *getGimbalDataPointer(void)
{
    return &gimbal;
}

bool *getGimbalStartedFlag(void)
{
    return &gimbal.started;
}

bool *getGimbalResetStatus(void)
{
    return &gimbal.reset_success;
}

bool *getGimbalMoveHomingStatus(void)
{
    return &gimbal.move_homing_success;
}

bool *getGimbalOperationHomingStatus(void)
{
    return &gimbal.operation_homing_success;
}

float getGimbalYawAngle(rfl_angle_format_e angle_format)
{
    if (angle_format == RFL_ANGLE_FORMAT_RADIAN)
        return gimbal.gimbal_angle.rad;

    return gimbal.gimbal_angle.deg;
}

static void gimbal_init(engineer_gimbal_s *gimbal)
{
    memset(gimbal, 0, sizeof(engineer_gimbal_s));

    gimbal->behavior = ENGINEER_BEHAVIOR_DISABLE;
    gimbal->last_behavior = ENGINEER_BEHAVIOR_DISABLE;

    gimbal->reset_step = ENGINEER_GIMBAL_RESET_STEP_HOMING;
    gimbal->reset_success = false;
    gimbal->move_homing_success = false;
    gimbal->operation_homing_success = false;

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
    motor_config.max_speed = 0.8f;
    rflAngleUpdate(&motor_config.max_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_GIMBAL_YAW_MOTOR_MAX_ANGLE);
    rflAngleUpdate(&motor_config.min_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_GIMBAL_YAW_MOTOR_MIN_ANGLE);
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
        }
        else if (gimbal->behavior != ENGINEER_BEHAVIOR_DISABLE)
        {
            rflMotorSetMode(&gimbal->yaw_motor, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        }

        if (gimbal->behavior == ENGINEER_BEHAVIOR_RESET)
        {
            rflMotorSetDegAngleLimit(&gimbal->yaw_motor, RFL_ANGLE_FORMAT_DEGREE,
                                     ENGINEER_GIMBAL_YAW_MOTOR_INITIAL_MAX_ANGLE,
                                     ENGINEER_GIMBAL_YAW_MOTOR_INITIAL_MIN_ANGLE);

            gimbal->reset_step = ENGINEER_GIMBAL_RESET_STEP_HOMING;
        }
        else if (getEngineerLastBehavior() == ENGINEER_BEHAVIOR_RESET && gimbal->behavior != ENGINEER_BEHAVIOR_RESET)
        {
            rflMotorSetDegAngleLimit(&gimbal->yaw_motor, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_GIMBAL_YAW_MOTOR_MAX_ANGLE,
                                     ENGINEER_GIMBAL_YAW_MOTOR_MIN_ANGLE);
        }

        if (gimbal->behavior == ENGINEER_BEHAVIOR_MOVE)
        {
            checkIfRcKeyFallingEdgeDetected(RC_LEFT);
            checkIfRcKeyFallingEdgeDetected(RC_RIGHT);
        }
        gimbal->enable_scout_mode = false;
    }

    // 根据不同模式使用不同控制

    if (gimbal->behavior == ENGINEER_BEHAVIOR_RESET)
        rflMotorSetMaxSpeed(&gimbal->yaw_motor, 1.2f);
    else
        rflMotorSetMaxSpeed(&gimbal->yaw_motor, 0.8f);

    switch (gimbal->behavior)
    {
    case ENGINEER_BEHAVIOR_DISABLE:
        gimbal_no_force_control(gimbal);
        break;
    case ENGINEER_BEHAVIOR_RESET:
        gimbal_reset_control(gimbal);
        break;
    case ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING:
        gimbal_move_homing_control(gimbal);
        break;
    case ENGINEER_BEHAVIOR_MOVE:
    case ENGINEER_BEHAVIOR_AUTO_GOLD_MINING: // 取大资源岛特殊处理
        if (gimbal->reset_success)
            gimbal_move_control(gimbal);
        else
            gimbal_no_force_control(gimbal);
        break;
    case ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING:
        gimbal_operation_homing_control(gimbal);
        break;
    case ENGINEER_BEHAVIOR_MANUAL_OPERATION:
        gimbal_manual_operation_control(gimbal);
        break;
    case ENGINEER_BEHAVIOR_AUTO_SILVER_MINING:
    case ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH:
    case ENGINEER_BEHAVIOR_AUTO_STORAGE_POP:
        gimbal_auto_operation_control(gimbal);
        break;

    default:
        break;
    }
}

static void gimbal_update_and_execute(engineer_gimbal_s *gimbal)
{
    rflAngleUpdate(&gimbal->gimbal_angle, RFL_ANGLE_FORMAT_RADIAN,
                   rflMotorGetAngle(&gimbal->yaw_motor, RFL_ANGLE_FORMAT_RADIAN) + getArmJointsValue(JOINT_2));

    rflAngleUpdate(
        &gimbal->set_gimbal_angle, RFL_ANGLE_FORMAT_DEGREE,
        rflFloatConstrain(gimbal->set_gimbal_angle.deg, ENGINEER_GIMBAL_YAW_MIN_ANGLE, ENGINEER_GIMBAL_YAW_MAX_ANGLE));

    if (gimbal->reset_step == ENGINEER_GIMBAL_RESET_STEP_HOMING)
    {
        rflMotorSetAngle(&gimbal->yaw_motor, RFL_ANGLE_FORMAT_DEGREE, gimbal->yaw_motor_homing_set_angle);
    }
    else
    {
        rflMotorSetAngle(&gimbal->yaw_motor, RFL_ANGLE_FORMAT_RADIAN,
                         gimbal->set_gimbal_angle.rad - getArmJointsValue(JOINT_2));

        gimbal->yaw_motor_homing_set_angle = rflMotorGetAngle(&gimbal->yaw_motor, RFL_ANGLE_FORMAT_DEGREE);
    }

    rflMotorUpdateStatus(&gimbal->yaw_motor);
    rflMotorUpdateControl(&gimbal->yaw_motor);
    rflRmMotorControl(GIMBAL_MOTORS_CAN_ORDINAL, GIMBAL_MOTORS_CAN_SLAVE_ID,
                      (int16_t)rflMotorGetOutput(&gimbal->yaw_motor), 0, 0, 0);
}

static void gimbal_no_force_control(engineer_gimbal_s *gimbal)
{
    rflAngleUpdate(&gimbal->set_gimbal_angle, RFL_ANGLE_FORMAT_DEGREE, gimbal->gimbal_angle.deg);
}

static void gimbal_reset_control(engineer_gimbal_s *gimbal)
{
    // gimbal->reset_success = true; // 只是测试时用来取消归中
    if (gimbal->reset_success == false)
    {
        if (gimbal->reset_step == ENGINEER_GIMBAL_RESET_STEP_HOMING)
        {
            gimbal->yaw_motor_homing_set_angle -= GIMBAL_YAW_HOMING_STEP_ANGLE;

            if (rflMotorGetTorque(&gimbal->yaw_motor) < -GIMBAL_YAW_HOMING_TORQUE_THRESHOLD &&
                fabsf(rflMotorGetSpeed(&gimbal->yaw_motor)) < 0.01f &&
                fabsf(rflMotorGetAngle(&gimbal->yaw_motor, RFL_ANGLE_FORMAT_DEGREE) -
                      gimbal->yaw_motor_homing_set_angle) > GIMBAL_YAW_HOMING_ANGLE_THRESHOLD)
            {
                rflMotorResetAngle(&gimbal->yaw_motor, RFL_ANGLE_FORMAT_DEGREE, GIMBAL_YAW_HOMING_ANGLE, false);
                gimbal->reset_step = ENGINEER_GIMBAL_RESET_STEP_STARTING;
            }
        }
        else if (gimbal->reset_step == ENGINEER_GIMBAL_RESET_STEP_STARTING)
        {
            rflAngleUpdate(&gimbal->set_gimbal_angle, RFL_ANGLE_FORMAT_DEGREE, -getChassisFollowOffsetMemory());

            if (fabsf(gimbal->gimbal_angle.deg - gimbal->set_gimbal_angle.deg) < 2.0f)
            {
                gimbal->reset_success = true;
            }
        }
    }
}

static void gimbal_move_homing_control(engineer_gimbal_s *gimbal)
{
    rflAngleUpdate(&gimbal->set_gimbal_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_MOVE_BEHAVIOR_GIMBAL_DEFAULT_SET_ANGLE);

    if (fabsf(gimbal->gimbal_angle.deg - gimbal->set_gimbal_angle.deg) < 0.8f)
    {
        gimbal->move_homing_success = true;
    }
}

static void gimbal_move_control(engineer_gimbal_s *gimbal)
{
    if (checkIfRcKeyFallingEdgeDetected(RC_RIGHT))
        gimbal->enable_scout_mode = true;
    if (checkIfRcKeyFallingEdgeDetected(RC_LEFT))
        gimbal->enable_scout_mode = false;

    if (gimbal->behavior != ENGINEER_BEHAVIOR_MOVE)
        gimbal->enable_scout_mode = false;

    rflAngleUpdate(&gimbal->set_gimbal_angle, RFL_ANGLE_FORMAT_DEGREE,
                   gimbal->enable_scout_mode ? 0.0f : ENGINEER_MOVE_BEHAVIOR_GIMBAL_DEFAULT_SET_ANGLE);
}

static void gimbal_operation_homing_control(engineer_gimbal_s *gimbal)
{
    rflAngleUpdate(&gimbal->set_gimbal_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_OPERATION_BEHAVIOR_GIMBAL_SET_ANGLE);

    if (fabsf(gimbal->gimbal_angle.deg - gimbal->set_gimbal_angle.deg) < 0.8f)
    {
        gimbal->operation_homing_success = true;
    }
}

static void gimbal_manual_operation_control(engineer_gimbal_s *gimbal)
{
    rflAngleUpdate(&gimbal->set_gimbal_angle, RFL_ANGLE_FORMAT_RADIAN, getArmTargetDirection());
}

static void gimbal_auto_operation_control(engineer_gimbal_s *gimbal)
{
    rflAngleUpdate(&gimbal->set_gimbal_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_OPERATION_BEHAVIOR_GIMBAL_SET_ANGLE);
}

static void gimbal_motor_yaw_can_rx_callback(uint8_t *rx_data)
{
    detect_hook_in_isr(GIMBAL_MOTOR_YAW_DH);
}
