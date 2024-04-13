#include "stdio.h"
#include "string.h"

#include "chassis_task.h"

#include "board.h"

#include "FreeRTOS.h"
#include "task.h"

#include "drv_can.h"

#include "algo_data_limiting.h"

#include "behavior_task.h"
#include "detect_task.h"

static void chassis_init(engineer_chassis_s *chassis);
static void chassis_mode_control(engineer_chassis_s *chassis);
static void chassis_set_control(engineer_chassis_s *chassis);
static void chassis_update_and_execute(engineer_chassis_s *chassis);
static void chassis_motor_0_can_rx_callback(void);
static void chassis_motor_1_can_rx_callback(void);
static void chassis_motor_2_can_rx_callback(void);
static void chassis_motor_3_can_rx_callback(void);

engineer_chassis_s chassis;

// float feedback[4] = {0.0f};
// float wheel_set_speed[4] = {0.0f};
// rfl_angle_s forward_vector = {0};
// rfl_angle_s control_vector = {0};
// #include "kine_stable_chassis_controller.h"
// rfl_chassis_normal_pid_controller_s *controller = NULL;

void chassis_task(void *pvParameters)
{
    while (!INS_init_finished)
        vTaskDelay(2);
    vTaskDelay(60);

    chassis_init(&chassis);

    while (1)
    {
        chassis_mode_control(&chassis);

        chassis_set_control(&chassis);

        chassis_update_and_execute(&chassis);

        vTaskDelay(2);
    }
}

engineer_chassis_s *getChassisDataPointer(void)
{
    return &chassis;
}

static void chassis_mode_control(engineer_chassis_s *chassis)
{
    if (getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_MOVE)
    {
        rflChassisSetBehavior(&chassis->model, RFL_CHASSIS_BEHAVIOR_FOLLOW_CONTROL);
        for (uint8_t i = 0; i < 4; i++)
        {
            rflMotorSetMode(chassis->motor + i, RFL_MOTOR_CONTROL_MODE_SPEED);
        }
    }
    else if (getEngineerCurrentBehavior() != ENGINEER_BEHAVIOR_MOVE)
    {
        rflChassisSetBehavior(&chassis->model, RFL_CHASSIS_BEHAVIOR_NO_FORCE);
        for (uint8_t i = 0; i < 4; i++)
        {
            rflMotorSetMode(chassis->motor + i, RFL_MOTOR_CONTROL_MODE_NO_FORCE);
        }
    }
}

static void chassis_set_control(engineer_chassis_s *chassis)
{
    if (getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_MOVE)
    {
        if ((chassis->rc->dt7_dr16_data.rc.ch[0] + chassis->rc->dt7_dr16_data.rc.ch[2] +
             chassis->rc->dt7_dr16_data.rc.ch[3]) > 3)
        {
            chassis->set_speed_vector[0] =
                rlfRampCalc(chassis->speed_ramper + 0,
                            ((float)(rflDeadZoneZero(chassis->rc->dt7_dr16_data.rc.ch[3], CHASSIS_RC_DEADLINE)) /
                             660.0f * CHASSIS_VX_RC_CONTROL_MAX));
            chassis->set_speed_vector[1] =
                rlfRampCalc(chassis->speed_ramper + 1,
                            -((float)(rflDeadZoneZero(chassis->rc->dt7_dr16_data.rc.ch[2], CHASSIS_RC_DEADLINE)) /
                              660.0f * CHASSIS_VY_RC_CONTROL_MAX));
            chassis->set_speed_vector[2] =
                rlfRampCalc(chassis->speed_ramper + 2,
                            -((float)(rflDeadZoneZero(chassis->rc->dt7_dr16_data.rc.ch[0], CHASSIS_RC_DEADLINE)) /
                              660.0f * CHASSIS_WZ_RC_CONTROL_MAX));
        }
        else
        {
            float km_x = 0;
            float km_y = 0;
            if (checkIsRcKeyPressed(RC_W))
                km_x = 1.0f;
            if (checkIsRcKeyPressed(RC_S))
                km_x = -1.0f;
            if (checkIsRcKeyPressed(RC_A))
                km_y = 1.0f;
            if (checkIsRcKeyPressed(RC_D))
                km_y = -1.0f;

            chassis->set_speed_vector[0] = rlfRampCalc(chassis->speed_ramper + 0, km_x * CHASSIS_VX_RC_CONTROL_MAX);
            chassis->set_speed_vector[1] = rlfRampCalc(chassis->speed_ramper + 1, km_y * CHASSIS_VY_RC_CONTROL_MAX);
            chassis->set_speed_vector[2] =
                rlfRampCalc(chassis->speed_ramper + 2, -((float)(getRcMouseY()) / 20.0f) * CHASSIS_WZ_RC_CONTROL_MAX);
        }
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

    // 更新底盘控制量

    if (getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_MOVE)
    {
        rflAngleUpdate(&chassis->set_angle, RFL_ANGLE_FORMAT_DEGREE,
                       chassis->set_angle.deg + chassis->set_speed_vector[2] * CHASSIS_YAW_CONTROL_SEN);
        rflAngleUpdate(&chassis->set_angle, RFL_ANGLE_FORMAT_DEGREE,
                       rflFloatLoopConstrain(chassis->set_angle.deg, -DEG_PI, DEG_PI));

        rflChassisSetSpeedVector(&chassis->model, chassis->set_speed_vector[0], chassis->set_speed_vector[1], 0.0f);
    }
    else
    {
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

    if (getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_MOVE)
    {
        rflRmMotorControl(CHASSIS_MOTORS_CAN_ORDINAL, CHASSIS_MOTORS_CAN_SLAVE_ID,
                          rflMotorGetOutput(chassis->motor + 0), rflMotorGetOutput(chassis->motor + 1),
                          rflMotorGetOutput(chassis->motor + 2), rflMotorGetOutput(chassis->motor + 3));
    }
    else
    {
        rflRmMotorControl(CHASSIS_MOTORS_CAN_ORDINAL, CHASSIS_MOTORS_CAN_SLAVE_ID, 0, 0, 0, 0);
    }

    // memcpy(feedback, chassis->model.motor_feedback, 4 * sizeof(float));
    // memcpy(wheel_set_speed, chassis->model.motor_output_, 4 * sizeof(float));
    // memcpy(&forward_vector, chassis->model.forward_vector_, sizeof(rfl_angle_s));
    // memcpy(&control_vector, chassis->model.control_vector, sizeof(rfl_angle_s));
}

static void chassis_init(engineer_chassis_s *chassis)
{
    memset(chassis, 0, sizeof(engineer_chassis_s));

    // 底盘运动学模型模块
    rfl_chassis_config_s config = {0};
    rflChassisGetDefaultConfig(&config, RFL_CHASSIS_MECANUM);
    config.mecanum_length = 0.370f;
    config.mecanum_width = 0.370f;
    config.reference_frame = RFL_CHASSIS_INERTIAL_FRAME;
    rflAngleUpdate(&chassis->set_angle, RFL_ANGLE_FORMAT_DEGREE, 0.0f);
    config.control_vector = &chassis->set_angle;
    config.direction_pid_param[0] = 0.1f;
    config.direction_pid_param[1] = 0.0f;
    config.direction_pid_param[2] = 0.0f;
    config.direction_pid_param[3] = 0.0f;
    config.direction_pid_param[4] = 4.0f;
    rflChassisInit(&chassis->model, &config, &chassis->yaw, chassis->wheel_speed);
    chassis->wheel_set_speed = rflChassisGetMotorOutputArray(&chassis->model);

    // controller = (rfl_chassis_normal_pid_controller_s *)chassis->model.direction_controller;

    // 底盘姿态
    chassis->ins = get_INS_data_point();

    // 底盘控制
    chassis->rc = getRemoteControlPointer();
    rlfRampInit(chassis->speed_ramper + 0, 0.005f, CHASSIS_WZ_RC_CONTROL_MAX, -CHASSIS_WZ_RC_CONTROL_MAX);
    rlfRampInit(chassis->speed_ramper + 1, 0.005f, CHASSIS_WZ_RC_CONTROL_MAX, -CHASSIS_WZ_RC_CONTROL_MAX);
    rlfRampInit(chassis->speed_ramper + 2, 0.005f, CHASSIS_WZ_RC_CONTROL_MAX, -CHASSIS_WZ_RC_CONTROL_MAX);

    // 底盘电机
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

    // 底盘电机CAN接收回调配置
    rflCanRxMessageBoxAddRxCallbackFunc(CHASSIS_MOTORS_CAN_ORDINAL, 0x201, chassis_motor_0_can_rx_callback);
    rflCanRxMessageBoxAddRxCallbackFunc(CHASSIS_MOTORS_CAN_ORDINAL, 0x202, chassis_motor_1_can_rx_callback);
    rflCanRxMessageBoxAddRxCallbackFunc(CHASSIS_MOTORS_CAN_ORDINAL, 0x203, chassis_motor_2_can_rx_callback);
    rflCanRxMessageBoxAddRxCallbackFunc(CHASSIS_MOTORS_CAN_ORDINAL, 0x204, chassis_motor_3_can_rx_callback);
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
