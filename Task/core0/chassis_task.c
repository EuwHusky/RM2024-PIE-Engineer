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
static void chassis_stop_control(engineer_chassis_s *chassis);
static void chassis_normal_control(engineer_chassis_s *chassis);
static void chassis_slowly_control(engineer_chassis_s *chassis);
static void chassis_silver_control(engineer_chassis_s *chassis);
static void chassis_motor_0_can_rx_callback(uint8_t *rx_data);
static void chassis_motor_1_can_rx_callback(uint8_t *rx_data);
static void chassis_motor_2_can_rx_callback(uint8_t *rx_data);
static void chassis_motor_3_can_rx_callback(uint8_t *rx_data);
static void lidar_can_rx_callback(uint8_t *rx_data);

static engineer_chassis_s chassis;

void chassis_task(void *pvParameters)
{
    rflOsDelayMs(1000);

    while (!INS_init_finished)
        rflOsDelayMs(2);
    rflOsDelayMs(60);

    chassis.started = false;

    chassis_init(&chassis);

    chassis.started = true;

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

bool *getChassisStartedFlag(void)
{
    return &chassis.started;
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
    config.direction_pid_param[0] = 0.2f;
    config.direction_pid_param[1] = 0.0f;
    config.direction_pid_param[2] = 0.005f;
    config.direction_pid_param[3] = 0.0f;
    config.direction_pid_param[4] = 4.0f;
    rflChassisInit(&chassis->model, &config, &chassis->yaw, chassis->wheel_speed);
    chassis->wheel_set_speed = rflChassisGetMotorOutputArray(&chassis->model);

    chassis->follow_offset = -GIMBAL_YAW_START_ANGLE;

    // 底盘姿态
    chassis->ins = get_INS_data_point();

    // 底盘控制
    rflRampInit(chassis->speed_ramper + 0, CHASSIS_CONTROL_TIME, CHASSIS_VX_MAX, -CHASSIS_VX_MAX);
    rflRampInit(chassis->speed_ramper + 1, CHASSIS_CONTROL_TIME, CHASSIS_VY_MAX, -CHASSIS_VY_MAX);
    rflRampInit(chassis->speed_ramper + 2, CHASSIS_CONTROL_TIME, CHASSIS_WZ_MAX, -CHASSIS_WZ_MAX);

    // CAN通信
    rflCanRxMessageBoxAddId(CHASSIS_MOTORS_CAN_ORDINAL, 0x201);
    rflCanRxMessageBoxAddId(CHASSIS_MOTORS_CAN_ORDINAL, 0x202);
    rflCanRxMessageBoxAddId(CHASSIS_MOTORS_CAN_ORDINAL, 0x203);
    rflCanRxMessageBoxAddId(CHASSIS_MOTORS_CAN_ORDINAL, 0x204);
    rflCanRxMessageBoxAddRxCallbackFunc(CHASSIS_MOTORS_CAN_ORDINAL, 0x201, chassis_motor_0_can_rx_callback);
    rflCanRxMessageBoxAddRxCallbackFunc(CHASSIS_MOTORS_CAN_ORDINAL, 0x202, chassis_motor_1_can_rx_callback);
    rflCanRxMessageBoxAddRxCallbackFunc(CHASSIS_MOTORS_CAN_ORDINAL, 0x203, chassis_motor_2_can_rx_callback);
    rflCanRxMessageBoxAddRxCallbackFunc(CHASSIS_MOTORS_CAN_ORDINAL, 0x204, chassis_motor_3_can_rx_callback);

    rflCanRxMessageBoxAddId(4, 0x500); // 激光雷达数据
    rflCanRxMessageBoxAddRxCallbackFunc(4, 0x500, lidar_can_rx_callback);

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
    case ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING:
    case ENGINEER_BEHAVIOR_MOVE:
        chassis_normal_control(chassis);
        break;

    case ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING:
    case ENGINEER_BEHAVIOR_MANUAL_OPERATION:
    case ENGINEER_BEHAVIOR_AUTO_GOLD_MINING:
    case ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH:
    case ENGINEER_BEHAVIOR_AUTO_STORAGE_POP:
        chassis_slowly_control(chassis);
        break;

    case ENGINEER_BEHAVIOR_AUTO_SILVER_MINING:
        chassis_silver_control(chassis);
        break;

    default:
        chassis_stop_control(chassis);
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
    {
        if ((chassis->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION ||
             chassis->behavior == ENGINEER_BEHAVIOR_AUTO_SILVER_MINING ||
             chassis->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH ||
             chassis->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_POP) ||
            ((chassis->behavior == ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING) &&
             (getEngineerLastBehavior() == ENGINEER_BEHAVIOR_MANUAL_OPERATION ||
              getEngineerLastBehavior() == ENGINEER_BEHAVIOR_AUTO_SILVER_MINING ||
              getEngineerLastBehavior() == ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH ||
              getEngineerLastBehavior() == ENGINEER_BEHAVIOR_AUTO_STORAGE_POP)))
        {
            chassis->follow_offset = 0.0f;
        }
        else
        {
            chassis->follow_offset = -getGimbalYawAngle(RFL_ANGLE_FORMAT_DEGREE);
        }
    }
    else
    {
        chassis->follow_offset = -GIMBAL_YAW_START_ANGLE;
    }

    if (rflChassisGetBehavior(&chassis->model) != RFL_CHASSIS_BEHAVIOR_NO_FORCE)
    {
        rflChassisSetFollowOffset(&chassis->model, RFL_ANGLE_FORMAT_DEGREE, chassis->follow_offset);

        if (chassis->behavior != ENGINEER_BEHAVIOR_AUTO_SILVER_MINING)
        {
            rflAngleUpdate(
                &chassis->set_control_angle, RFL_ANGLE_FORMAT_DEGREE,
                rflFloatLoopConstrain(chassis->set_control_angle.deg +
                                          chassis->set_speed_vector[2] * RADIAN_TO_DEGREE_FACTOR * CHASSIS_CONTROL_TIME,
                                      -DEG_PI, DEG_PI));
        }

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

    rflRmMotorControl(CHASSIS_MOTORS_CAN_ORDINAL, CHASSIS_MOTORS_CAN_SLAVE_ID,
                      (int16_t)rflMotorGetOutput(chassis->motor + 0), (int16_t)rflMotorGetOutput(chassis->motor + 1),
                      (int16_t)rflMotorGetOutput(chassis->motor + 2), (int16_t)rflMotorGetOutput(chassis->motor + 3));
}

static void chassis_stop_control(engineer_chassis_s *chassis)
{
    for (uint8_t i = 0; i < 3; i++)
        chassis->set_speed_vector[i] = 0.0f;
}

static void chassis_normal_control(engineer_chassis_s *chassis)
{
    if (abs(getDt7RockerPosition(DT7_ROCKER_RIGHT_HORIZONTAL)) + abs(getDt7RockerPosition(DT7_ROCKER_LEFT_HORIZONTAL)) +
            abs(getDt7RockerPosition(DT7_ROCKER_LEFT_VERTICAL)) >
        3)
    {
        chassis->set_speed_vector[0] = rflRampCalc(
            chassis->speed_ramper + 0, CHASSIS_VX_MAX,
            ((float)(rflDeadZoneZero(getDt7RockerPosition(DT7_ROCKER_LEFT_VERTICAL), RC_DT7_ROCKER_DEADLINE)) / 660.0f *
             CHASSIS_VX_MAX));
        chassis->set_speed_vector[1] = rflRampCalc(
            chassis->speed_ramper + 1, CHASSIS_VY_MAX,
            -((float)(rflDeadZoneZero(getDt7RockerPosition(DT7_ROCKER_LEFT_HORIZONTAL), RC_DT7_ROCKER_DEADLINE)) /
              660.0f * CHASSIS_VY_MAX));
        chassis->set_speed_vector[2] = rflRampCalc(
            chassis->speed_ramper + 2, CHASSIS_WZ_MAX * 6.0f,
            -((float)(rflDeadZoneZero(getDt7RockerPosition(DT7_ROCKER_RIGHT_HORIZONTAL), RC_DT7_ROCKER_DEADLINE)) /
              660.0f * CHASSIS_WZ_MAX));
    }
    else
    {
        float x_sign = 0.0f;
        float y_sign = 0.0f;
        float z_sign = 0.0f;
        if (checkIfRcKeyPressed(RC_W) && !checkIfRcKeyPressed(RC_S))
            x_sign = 1.0f;
        else if (!checkIfRcKeyPressed(RC_W) && checkIfRcKeyPressed(RC_S))
            x_sign = -1.0f;
        if (checkIfRcKeyPressed(RC_A) && !checkIfRcKeyPressed(RC_D))
            y_sign = 1.0f;
        else if (!checkIfRcKeyPressed(RC_A) && checkIfRcKeyPressed(RC_D))
            y_sign = -1.0f;
        if (!checkIfRcKeyPressed(RC_CTRL))
        {
            if (checkIfRcKeyPressed(RC_Q) && !checkIfRcKeyPressed(RC_E))
                z_sign = 1.0f;
            else if (!checkIfRcKeyPressed(RC_Q) && checkIfRcKeyPressed(RC_E))
                z_sign = -1.0f;
        }

        float shift_speed_multiplier = 0.5f;
        float shift_accel_multiplier = 0.5f;
        if (checkIfRcKeyPressed(RC_SHIFT))
        {
            shift_speed_multiplier = 1.0f;
            shift_accel_multiplier = 0.8f;
        }

        chassis->set_speed_vector[0] =
            rflRampCalc(chassis->speed_ramper + 0, CHASSIS_VX_MAX * 3.0f * shift_accel_multiplier,
                        x_sign * CHASSIS_VX_MAX * shift_speed_multiplier);
        chassis->set_speed_vector[1] =
            rflRampCalc(chassis->speed_ramper + 1, CHASSIS_VY_MAX * 3.0f * shift_accel_multiplier,
                        y_sign * CHASSIS_VY_MAX * shift_speed_multiplier);
        if (!checkIfRcKeyPressed(RC_Q) && !checkIfRcKeyPressed(RC_E))
        {
            chassis->set_speed_vector[2] =
                rflRampCalc(chassis->speed_ramper + 2, CHASSIS_WZ_MAX * 6.0f,
                            -((float)(getRcMouseX()) / 18.0f) * CHASSIS_WZ_MAX * shift_speed_multiplier);
        }
        else
        {
            chassis->set_speed_vector[2] = rflRampCalc(chassis->speed_ramper + 2, CHASSIS_WZ_MAX * 6.0f,
                                                       z_sign * CHASSIS_WZ_MAX * shift_speed_multiplier);
        }
    }
}

static void chassis_slowly_control(engineer_chassis_s *chassis)
{
    float x_sign = 0.0f;
    float y_sign = 0.0f;
    float z_sign = 0.0f;
    if (checkIfRcKeyPressed(RC_W) && !checkIfRcKeyPressed(RC_S))
        x_sign = 1.0f;
    else if (!checkIfRcKeyPressed(RC_W) && checkIfRcKeyPressed(RC_S))
        x_sign = -1.0f;
    if (checkIfRcKeyPressed(RC_A) && !checkIfRcKeyPressed(RC_D))
        y_sign = 1.0f;
    else if (!checkIfRcKeyPressed(RC_A) && checkIfRcKeyPressed(RC_D))
        y_sign = -1.0f;
    if (!checkIfRcKeyPressed(RC_CTRL))
    {
        if (checkIfRcKeyPressed(RC_Q) && !checkIfRcKeyPressed(RC_E))
            z_sign = 1.0f;
        else if (!checkIfRcKeyPressed(RC_Q) && checkIfRcKeyPressed(RC_E))
            z_sign = -1.0f;
    }

    float shift_speed_multiplier = 0.42f;
    if (checkIfRcKeyPressed(RC_SHIFT))
        shift_speed_multiplier = 1.0f;

    chassis->set_speed_vector[0] = rflRampCalc(chassis->speed_ramper + 0, CHASSIS_VX_MAX * 2.0f,
                                               x_sign * CHASSIS_VX_MAX / 15.0f * shift_speed_multiplier);
    chassis->set_speed_vector[1] = rflRampCalc(chassis->speed_ramper + 1, CHASSIS_VY_MAX * 2.0f,
                                               y_sign * CHASSIS_VY_MAX / 10.0f * shift_speed_multiplier);
    if (!checkIfRcKeyPressed(RC_Q) && !checkIfRcKeyPressed(RC_E))
    {
        chassis->set_speed_vector[2] = rflRampCalc(chassis->speed_ramper + 2, CHASSIS_WZ_MAX * 6.0f,
                                                   -((float)(getRcMouseX()) / 24.0f) * CHASSIS_WZ_MAX / 6.0f);
    }
    else
    {
        chassis->set_speed_vector[2] = rflRampCalc(chassis->speed_ramper + 2, CHASSIS_WZ_MAX * 6.0f,
                                                   z_sign * CHASSIS_WZ_MAX / 6.0f * shift_speed_multiplier);
    }
}

static void chassis_silver_control(engineer_chassis_s *chassis)
{
    float x_sign = 0.0f;
    float y_sign = 0.0f;
    float z_sign = 0.0f;
    if (checkIfRcKeyPressed(RC_W) && !checkIfRcKeyPressed(RC_S))
        x_sign = 1.0f;
    else if (!checkIfRcKeyPressed(RC_W) && checkIfRcKeyPressed(RC_S))
        x_sign = -1.0f;
    if (checkIfRcKeyPressed(RC_A) && !checkIfRcKeyPressed(RC_D))
        y_sign = 1.0f;
    else if (!checkIfRcKeyPressed(RC_A) && checkIfRcKeyPressed(RC_D))
        y_sign = -1.0f;
    if (!checkIfRcKeyPressed(RC_CTRL))
    {
        if (checkIfRcKeyPressed(RC_Q) && !checkIfRcKeyPressed(RC_E))
            z_sign = 1.0f;
        else if (!checkIfRcKeyPressed(RC_Q) && checkIfRcKeyPressed(RC_E))
            z_sign = -1.0f;
    }

    float shift_speed_multiplier = 0.42f;
    if (checkIfRcKeyPressed(RC_SHIFT))
        shift_speed_multiplier = 1.0f;

    float x_align_distance = (chassis->lidar_obstacle_distance - SILVER_MINING_X_DISTANCE_OFFSET);
    float yaw_align_angle = (chassis->lidar_obstacle_surface_angle - SILVER_MINING_YAW_ANGLE_OFFSET);

    // 安全启用条件
    // 强制手动控制
    float x_align_speed = (fabsf(x_align_distance) < 0.3f && !checkIfRcKeyPressed(RC_CTRL))
                              ? (x_align_distance * SILVER_MINING_X_ALIGN_KP * shift_speed_multiplier)
                              : 0.0f;
    float yaw_align_speed = (fabsf(yaw_align_angle) < 12.0f && !checkIfRcKeyPressed(RC_CTRL))
                                ? (yaw_align_angle * SILVER_MINING_YAW_ALIGN_KP * shift_speed_multiplier)
                                : 0.0f;

    chassis->set_speed_vector[0] =
        rflRampCalc(chassis->speed_ramper + 0, CHASSIS_VX_MAX * 2.0f,
                    x_sign * CHASSIS_VX_MAX / 15.0f * shift_speed_multiplier + x_align_speed);
    chassis->set_speed_vector[1] = rflRampCalc(chassis->speed_ramper + 1, CHASSIS_VY_MAX * 2.0f,
                                               y_sign * CHASSIS_VY_MAX / 10.0f * shift_speed_multiplier);
    chassis->set_speed_vector[2] = 0.0f;

    rflAngleUpdate(&chassis->set_control_angle, RFL_ANGLE_FORMAT_DEGREE,
                   rflFloatLoopConstrain(chassis->set_control_angle.deg +
                                             rflRampCalc(chassis->speed_ramper + 2, CHASSIS_WZ_MAX * 6.0f,
                                                         z_sign * CHASSIS_WZ_MAX / 6.0f * shift_speed_multiplier) *
                                                 RADIAN_TO_DEGREE_FACTOR * CHASSIS_CONTROL_TIME -
                                             yaw_align_speed,
                                         -DEG_PI, DEG_PI));
}

static void chassis_motor_0_can_rx_callback(uint8_t *rx_data)
{
    detect_hook_in_isr(CHASSIS_MOTOR_0_DH);
}
static void chassis_motor_1_can_rx_callback(uint8_t *rx_data)
{
    detect_hook_in_isr(CHASSIS_MOTOR_1_DH);
}
static void chassis_motor_2_can_rx_callback(uint8_t *rx_data)
{
    detect_hook_in_isr(CHASSIS_MOTOR_2_DH);
}
static void chassis_motor_3_can_rx_callback(uint8_t *rx_data)
{
    detect_hook_in_isr(CHASSIS_MOTOR_3_DH);
}

static void lidar_can_rx_callback(uint8_t *rx_data)
{
    memcpy(&chassis.lidar_obstacle_distance, rx_data + 0u, sizeof(float));
    memcpy(&chassis.lidar_obstacle_surface_angle, rx_data + 4u, sizeof(float));

    detect_hook_in_isr(LIDAR_DH);
}
