#include "stdio.h"
#include "string.h"

#include "chassis_task.h"

#include "board.h"

#include "drv_delay.h"

#include "drv_can.h"

#include "algo_data_limiting.h"

#include "detect_task.h"
#include "gimbal_task.h"

static void chassis_init(engineer_chassis_s *chassis);
static void chassis_mode_control(engineer_chassis_s *chassis);
static void chassis_update_and_execute(engineer_chassis_s *chassis);
static void chassis_normal_control(engineer_chassis_s *chassis);
static void chassis_motor_0_can_rx_callback(void);
static void chassis_motor_1_can_rx_callback(void);
static void chassis_motor_2_can_rx_callback(void);
static void chassis_motor_3_can_rx_callback(void);

static engineer_chassis_s chassis;

void chassis_task(void *pvParameters)
{
    rflOsDelayMs(1000);

    while (!INS_init_finished)
        rflOsDelayMs(2);
    rflOsDelayMs(60);

    chassis_init(&chassis);

    while (1)
    {
        chassis_mode_control(&chassis);

        chassis_update_and_execute(&chassis);

        rflOsDelayMs(2);
    }
}

engineer_chassis_s *getChassisDataPointer(void)
{
    return &chassis;
}

float getChassisFollowOffsetMemory(void)
{
    return chassis.follow_offset;
}

static void chassis_init(engineer_chassis_s *chassis)
{
    memset(chassis, 0, sizeof(engineer_chassis_s));

    chassis->behavior = ENGINEER_BEHAVIOR_DISABLE;
    chassis->last_behavior = ENGINEER_BEHAVIOR_DISABLE;

    // 底盘运动学模型模块
    rfl_chassis_config_s config = {0};
    rflChassisGetDefaultConfig(&config, RFL_CHASSIS_MECANUM);
    config.mecanum_length = 0.370f;
    config.mecanum_width = 0.370f;
    config.reference_frame = RFL_CHASSIS_INERTIAL_FRAME;
    rflAngleUpdate(&chassis->set_control_angle, RFL_ANGLE_FORMAT_DEGREE, GIMBAL_YAW_START_ANGLE);
    config.set_control_vector = &chassis->set_control_angle;
    config.direction_pid_param[0] = 0.1f;
    config.direction_pid_param[1] = 0.0f;
    config.direction_pid_param[2] = 0.0f;
    config.direction_pid_param[3] = 0.0f;
    config.direction_pid_param[4] = 4.0f;
    rflChassisInit(&chassis->model, &config, &chassis->yaw, chassis->wheel_speed);
    chassis->wheel_set_speed = rflChassisGetMotorOutputArray(&chassis->model);

    chassis->follow_offset = -GIMBAL_YAW_START_ANGLE;

    // 底盘姿态
    chassis->ins = get_INS_data_point();

    // 底盘控制
    chassis->rc = getRemoteControlPointer();
    rflRampInit(chassis->speed_ramper + 0, 0.008f, CHASSIS_WZ_DT7_CONTROL_MAX, -CHASSIS_WZ_DT7_CONTROL_MAX);
    rflRampInit(chassis->speed_ramper + 1, 0.008f, CHASSIS_WZ_DT7_CONTROL_MAX, -CHASSIS_WZ_DT7_CONTROL_MAX);
    rflRampInit(chassis->speed_ramper + 2, 0.05f, CHASSIS_WZ_DT7_CONTROL_MAX, -CHASSIS_WZ_DT7_CONTROL_MAX);

    // CAN通信
    rflCanRxMessageBoxAddId(CHASSIS_MOTORS_CAN_ORDINAL, 0x201);
    rflCanRxMessageBoxAddId(CHASSIS_MOTORS_CAN_ORDINAL, 0x202);
    rflCanRxMessageBoxAddId(CHASSIS_MOTORS_CAN_ORDINAL, 0x203);
    rflCanRxMessageBoxAddId(CHASSIS_MOTORS_CAN_ORDINAL, 0x204);
    rflCanRxMessageBoxAddRxCallbackFunc(CHASSIS_MOTORS_CAN_ORDINAL, 0x201, chassis_motor_0_can_rx_callback);
    rflCanRxMessageBoxAddRxCallbackFunc(CHASSIS_MOTORS_CAN_ORDINAL, 0x202, chassis_motor_1_can_rx_callback);
    rflCanRxMessageBoxAddRxCallbackFunc(CHASSIS_MOTORS_CAN_ORDINAL, 0x203, chassis_motor_2_can_rx_callback);
    rflCanRxMessageBoxAddRxCallbackFunc(CHASSIS_MOTORS_CAN_ORDINAL, 0x204, chassis_motor_3_can_rx_callback);

    // 底盘电机
    while (detect_error(CHASSIS_MOTOR_0_DH) || detect_error(CHASSIS_MOTOR_1_DH) || detect_error(CHASSIS_MOTOR_2_DH) ||
           detect_error(CHASSIS_MOTOR_3_DH))
        rflOsDelayMs(10);
    rfl_motor_config_s motor_config = {0};
    rflMotorGetDefaultConfig(&motor_config, RFL_MOTOR_RM_M3508, RFL_MOTOR_CONTROLLER_PID);
    motor_config.speed_pid_kp = ENGINEER_CHASSIS_RM_M3508_SPEED_PID_KP;
    motor_config.speed_pid_ki = ENGINEER_CHASSIS_RM_M3508_SPEED_PID_KI;
    motor_config.speed_pid_kd = ENGINEER_CHASSIS_RM_M3508_SPEED_PID_KD;
    motor_config.speed_pid_max_iout = ENGINEER_CHASSIS_RM_M3508_SPEED_PID_MAX_IOUT;
    motor_config.speed_pid_max_out = ENGINEER_CHASSIS_RM_M3508_SPEED_PID_MAX_OUT;
    for (uint8_t i = 0; i < 4; i++)
    {
        if (i < 2)
            motor_config.is_reversed = false;
        else
            motor_config.is_reversed = true;
        motor_config.can_ordinal = CHASSIS_MOTORS_CAN_ORDINAL;
        motor_config.master_can_id = 0x201 + i;
        rflMotorInit(chassis->motor + i, &motor_config);
    }
}

static void chassis_mode_control(engineer_chassis_s *chassis)
{
    // 模式切换时修改底盘设定

    chassis->last_behavior = chassis->behavior;
    chassis->behavior = getEngineerCurrentBehavior();

    if (chassis->last_behavior != chassis->behavior)
    {
        if (chassis->behavior == ENGINEER_BEHAVIOR_DISABLE || chassis->behavior == ENGINEER_BEHAVIOR_RESET)
        {
            rflChassisSetBehavior(&chassis->model, RFL_CHASSIS_BEHAVIOR_NO_FORCE);
            for (uint8_t i = 0; i < 4; i++)
            {
                rflMotorSetMode(chassis->motor + i, RFL_MOTOR_CONTROL_MODE_NO_FORCE);
            }
        }
        else if (chassis->behavior != ENGINEER_BEHAVIOR_DISABLE && chassis->behavior != ENGINEER_BEHAVIOR_RESET)
        {
            rflChassisSetBehavior(&chassis->model, RFL_CHASSIS_BEHAVIOR_FOLLOW_CONTROL);
            for (uint8_t i = 0; i < 4; i++)
            {
                rflMotorSetMode(chassis->motor + i, RFL_MOTOR_CONTROL_MODE_SPEED);
            }
        }
    }

    // 根据不同模式使用不同控制

    switch (chassis->behavior)
    {
    case ENGINEER_BEHAVIOR_MOVE:
        chassis_normal_control(chassis);
        break;

    default:
        break;
    }
}

static void chassis_update_and_execute(engineer_chassis_s *chassis)
{
    // 更新底盘状态量

    rflAngleUpdate(&chassis->yaw, RFL_ANGLE_FORMAT_DEGREE, chassis->ins->Yaw);

    for (uint8_t i = 0; i < 4; i++)
    {
        rflMotorUpdateStatus(chassis->motor + i);
        chassis->wheel_speed[i] = rflMotorGetSpeed(chassis->motor + i) * CHASSIS_WHEEL_RADIUS;
    }

    // 更新底盘模型控制量

    if (*getGimbalResetStatus())
        chassis->follow_offset = -getGimbalYawAngle(RFL_ANGLE_FORMAT_DEGREE);

    if (chassis->model.mode_ != RFL_CHASSIS_BEHAVIOR_NO_FORCE)
    {
        rflChassisSetFollowOffset(&chassis->model, RFL_ANGLE_FORMAT_DEGREE, chassis->follow_offset);

        rflAngleUpdate(&chassis->set_control_angle, RFL_ANGLE_FORMAT_DEGREE,
                       rflFloatLoopConstrain(chassis->set_control_angle.deg +
                                                 chassis->set_speed_vector[2] * CHASSIS_YAW_CONTROL_SEN,
                                             -DEG_PI, DEG_PI));

        rflChassisSetSpeedVector(&chassis->model, chassis->set_speed_vector[0], chassis->set_speed_vector[1], 0.0f);
    }
    else
    {
        rflAngleUpdate(&chassis->set_control_angle, RFL_ANGLE_FORMAT_DEGREE,
                       rflFloatLoopConstrain(chassis->yaw.deg - chassis->follow_offset, -DEG_PI, DEG_PI));

        rflChassisSetSpeedVector(&chassis->model, 0.0f, 0.0f, 0.0f);
    }

    // 更新底盘模型

    rflChassisUpdate(&chassis->model);

    // 应用底盘模型输出

    for (uint8_t i = 0; i < 4; i++)
    {
        rflMotorSetSpeed(chassis->motor + i, chassis->wheel_set_speed[i] / CHASSIS_WHEEL_RADIUS);
        rflMotorUpdateControl(chassis->motor + i);
    }

    rflRmMotorControl(CHASSIS_MOTORS_CAN_ORDINAL, CHASSIS_MOTORS_CAN_SLAVE_ID, rflMotorGetOutput(chassis->motor + 0),
                      rflMotorGetOutput(chassis->motor + 1), rflMotorGetOutput(chassis->motor + 2),
                      rflMotorGetOutput(chassis->motor + 3));
}

static void chassis_normal_control(engineer_chassis_s *chassis)
{
    if (abs(chassis->rc->dt7_dr16_data.rc.ch[0]) + abs(chassis->rc->dt7_dr16_data.rc.ch[2]) +
            abs(chassis->rc->dt7_dr16_data.rc.ch[3]) >
        3)
    {
        chassis->set_speed_vector[0] =
            rflRampCalc(chassis->speed_ramper + 0,
                        ((float)(rflDeadZoneZero(chassis->rc->dt7_dr16_data.rc.ch[3], CHASSIS_DT7_DEADLINE)) / 660.0f *
                         CHASSIS_VX_DT7_CONTROL_MAX));
        chassis->set_speed_vector[1] =
            rflRampCalc(chassis->speed_ramper + 1,
                        -((float)(rflDeadZoneZero(chassis->rc->dt7_dr16_data.rc.ch[2], CHASSIS_DT7_DEADLINE)) / 660.0f *
                          CHASSIS_VY_DT7_CONTROL_MAX));
        chassis->set_speed_vector[2] =
            rflRampCalc(chassis->speed_ramper + 2,
                        -((float)(rflDeadZoneZero(chassis->rc->dt7_dr16_data.rc.ch[0], CHASSIS_DT7_DEADLINE)) / 660.0f *
                          CHASSIS_WZ_DT7_CONTROL_MAX));
    }
    else
    {
        float km_x = 0.0f;
        float km_y = 0.0f;
        if (checkIsRcKeyPressed(RC_W))
            km_x = 1.0f;
        else if (checkIsRcKeyPressed(RC_S))
            km_x = -1.0f;
        else
            km_x = 0.0f;
        if (checkIsRcKeyPressed(RC_A))
            km_y = 1.0f;
        else if (checkIsRcKeyPressed(RC_D))
            km_y = -1.0f;
        else
            km_y = 0.0f;

        chassis->set_speed_vector[0] = rflRampCalc(chassis->speed_ramper + 0, km_x * CHASSIS_VX_KM_CONTROL_MAX);
        chassis->set_speed_vector[1] = rflRampCalc(chassis->speed_ramper + 1, km_y * CHASSIS_VY_KM_CONTROL_MAX);
        chassis->set_speed_vector[2] =
            rflRampCalc(chassis->speed_ramper + 2, -((float)(getRcMouseX()) / 20.0f) * CHASSIS_WZ_KM_CONTROL_MAX);
    }
}

static void chassis_motor_0_can_rx_callback(void)
{
    detect_hook(CHASSIS_MOTOR_0_DH);
}
static void chassis_motor_1_can_rx_callback(void)
{
    detect_hook(CHASSIS_MOTOR_1_DH);
}
static void chassis_motor_2_can_rx_callback(void)
{
    detect_hook(CHASSIS_MOTOR_2_DH);
}
static void chassis_motor_3_can_rx_callback(void)
{
    detect_hook(CHASSIS_MOTOR_3_DH);
}
