#include "arm_motor.h"

#include "board.h"
#include "hpm_can_drv.h"

#include "drv_can.h"
#include "drv_delay.h"

// damiao_motor_s *motor_driver_1 = NULL;
// damiao_motor_s *motor_driver_2 = NULL;

/**
 * @brief 初始化机械臂电机
 */
void arm_motor_init(engineer_scara_arm_s *scara_arm)
{
    rfl_motor_config_s config = {0};

    /* 关节1 两个电机 */
    rflMotorGetDefaultConfig(&config, RFL_MOTOR_RM_M3508, RFL_MOTOR_CONTROLLER_PID);
    rflAngleUpdate(&config.max_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_MOTOR_JOINT_1_MAX_ANGLE);
    rflAngleUpdate(&config.min_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_MOTOR_JOINT_1_MIN_ANGLE);
    config.control_period_factor = 1.0f;
    config.max_speed = 0.1f;
    config.angle_pid_kp = ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_KP;
    config.angle_pid_ki = ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_KI;
    config.angle_pid_kd = ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_KD;
    config.angle_pid_max_iout = ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_MAX_IOUT;
    config.angle_pid_max_out = ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_MAX_OUT;
    config.speed_pid_kp = ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_KP;
    config.speed_pid_ki = ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_KI;
    config.speed_pid_kd = ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_KD;
    config.speed_pid_max_iout = ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_MAX_IOUT;
    config.speed_pid_max_out = ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_MAX_OUT;

    config.is_reversed = false;
    config.can_ordinal = ENGINEER_ARM_JOINTS_123_MOTORS_CAN_ORDINAL;
    config.master_can_id = 0x201;
    rflMotorInit(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], &config);

    config.is_reversed = true;
    config.can_ordinal = ENGINEER_ARM_JOINTS_123_MOTORS_CAN_ORDINAL;
    config.master_can_id = 0x202;
    rflMotorInit(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], &config);

    /* 关节2、3 各一个电机 */

    rflMotorGetDefaultConfig(&config, RFL_MOTOR_DM_J8009_2EC, RFL_MOTOR_CONTROLLER_DAMIAO);
    config.control_period_factor = 1.0f;

    config.is_reversed = false;
    rflAngleUpdate(&config.max_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_MOTOR_JOINT_23_BACK_MAX_ANGLE);
    rflAngleUpdate(&config.min_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_MOTOR_JOINT_23_BACK_MIN_ANGLE);
    config.can_ordinal = ENGINEER_ARM_JOINTS_123_MOTORS_CAN_ORDINAL;
    config.master_can_id = 0x02;
    config.slave_can_id = 0x00;
    rflMotorInit(&scara_arm->joints_motors[MOTOR_JOINT23_BACK], &config);

    config.is_reversed = false;
    rflAngleUpdate(&config.max_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_MOTOR_JOINT_23_FRONT_MAX_ANGLE);
    rflAngleUpdate(&config.min_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_MOTOR_JOINT_23_FRONT_MIN_ANGLE);
    config.can_ordinal = ENGINEER_ARM_JOINTS_123_MOTORS_CAN_ORDINAL;
    config.master_can_id = 0x03;
    config.slave_can_id = 0x01;
    rflMotorInit(&scara_arm->joints_motors[MOTOR_JOINT23_FRONT], &config);

    /* 关节4 一个电机 */
    rflMotorGetDefaultConfig(&config, RFL_MOTOR_RM_M3508, RFL_MOTOR_CONTROLLER_PID);
    config.control_period_factor = 10.0f;
    config.is_reversed = true;
    rflAngleUpdate(&config.max_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_MOTOR_JOINT_4_MAX_ANGLE);
    rflAngleUpdate(&config.min_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_MOTOR_JOINT_4_MIN_ANGLE);
    config.angle_pid_kp = ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_KP;
    config.angle_pid_ki = ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_KI;
    config.angle_pid_kd = ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_KD;
    config.angle_pid_max_iout = ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_MAX_IOUT;
    config.angle_pid_max_out = ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_MAX_OUT;
    config.speed_pid_kp = ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_KP;
    config.speed_pid_ki = ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_KI;
    config.speed_pid_kd = ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_KD;
    config.speed_pid_max_iout = ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_MAX_IOUT;
    config.speed_pid_max_out = ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_MAX_OUT;
    config.can_ordinal = ENGINEER_ARM_JOINTS_456_MOTORS_CAN_ORDINAL;
    config.master_can_id = 0x201;
    rflMotorInit(&scara_arm->joints_motors[MOTOR_JOINT4], &config);

    /* 关节56 共用两个电机 */
    rflMotorGetDefaultConfig(&config, RFL_MOTOR_RM_M2006, RFL_MOTOR_CONTROLLER_PID);
    config.control_period_factor = 4.0f;
    config.effector_transmission_ratio = RM_M2006_REDUCTION_RATIO * END_TRANSMISSION_GEAR_REDUCTION;
    config.angle_pid_kp = ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_KP;
    config.angle_pid_ki = ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_KI;
    config.angle_pid_kd = ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_KD;
    config.angle_pid_max_iout = ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_MAX_IOUT;
    config.angle_pid_max_out = ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_MAX_OUT;
    config.speed_pid_kp = ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_KP;
    config.speed_pid_ki = ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_KI;
    config.speed_pid_kd = ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_KD;
    config.speed_pid_max_iout = ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_MAX_IOUT;
    config.speed_pid_max_out = ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_MAX_OUT;

    rflAngleUpdate(&config.max_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_MOTOR_JOINT_56_MAX_ANGLE);
    rflAngleUpdate(&config.min_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_MOTOR_JOINT_56_MIN_ANGLE);
    config.is_reversed = false;
    config.can_ordinal = ENGINEER_ARM_JOINTS_456_MOTORS_CAN_ORDINAL;
    config.master_can_id = 0x202;
    rflMotorInit(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], &config);

    rflAngleUpdate(&config.max_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_MOTOR_JOINT_56_MAX_ANGLE);
    rflAngleUpdate(&config.min_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_MOTOR_JOINT_56_MIN_ANGLE);
    config.is_reversed = true;
    config.can_ordinal = ENGINEER_ARM_JOINTS_456_MOTORS_CAN_ORDINAL;
    config.master_can_id = 0x203;
    rflMotorInit(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], &config);
}

/**
 * @brief 更新机械臂电机状态量并执行控制
 */
void arm_motor_update_and_execute(engineer_scara_arm_s *scara_arm)
{
    // 设定电机角度

    if (scara_arm->mode != ARM_MODE_START_UP)
    {
        rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], RFL_ANGLE_FORMAT_DEGREE,
                         scara_arm->set_joints_value[JOINT_1] * LIFTER_DISTANCE_TO_DEGREE_FACTOR);
        rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], RFL_ANGLE_FORMAT_DEGREE,
                         scara_arm->set_joints_value[JOINT_1] * LIFTER_DISTANCE_TO_DEGREE_FACTOR);

        rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT23_BACK], RFL_ANGLE_FORMAT_RADIAN,
                         scara_arm->set_joints_value[JOINT_2] * JOINT2_REDUCTION);

        rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT23_FRONT], RFL_ANGLE_FORMAT_RADIAN,
                         scara_arm->set_joints_value[JOINT_2] + scara_arm->set_joints_value[JOINT_3]);

        rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT4], RFL_ANGLE_FORMAT_RADIAN,
                         scara_arm->set_joints_value[JOINT_4]);

        rflMotorSetAngle(
            &scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_RADIAN,
            ((scara_arm->set_joints_value[JOINT_5] * END_TRANSMISSION_GEAR_REDUCTION) -
             (scara_arm->set_joints_value[JOINT_6] * END_TRANSMISSION_GEAR_REDUCTION * END_BEVEL_GEAR_SET_REDUCTION)) *
                0.5f);
        rflMotorSetAngle(
            &scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_ANGLE_FORMAT_RADIAN,
            ((scara_arm->set_joints_value[JOINT_5] * END_TRANSMISSION_GEAR_REDUCTION) +
             (scara_arm->set_joints_value[JOINT_6] * END_TRANSMISSION_GEAR_REDUCTION * END_BEVEL_GEAR_SET_REDUCTION)) *
                0.5f);
    }

    // 电机更新&执行

    rflMotorUpdateStatus(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT]);
    rflMotorUpdateStatus(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT]);
    rflMotorUpdateControl(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT]);
    rflMotorUpdateControl(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT]);

    for (uint16_t i = 0; i < 1000; i++) // 检查发送缓冲区是否已满，满则循环延时
        if (!can_is_secondary_transmit_buffer_full(BOARD_CAN1))
            break;
    rflRmMotorControl(ENGINEER_ARM_JOINTS_123_MOTORS_CAN_ORDINAL, ENGINEER_ARM_JOINTS_123_RM_MOTORS_CAN_SLAVE_ID,
                      (int16_t)rflMotorGetOutput(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT]),
                      (int16_t)rflMotorGetOutput(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT]), 0, 0);

    rflMotorUpdateStatus(&scara_arm->joints_motors[MOTOR_JOINT23_BACK]);
    rflMotorUpdateControl(&scara_arm->joints_motors[MOTOR_JOINT23_BACK]);
    rflMotorExecuteControl(&scara_arm->joints_motors[MOTOR_JOINT23_BACK]);

    rflMotorUpdateStatus(&scara_arm->joints_motors[MOTOR_JOINT23_FRONT]);
    rflMotorUpdateControl(&scara_arm->joints_motors[MOTOR_JOINT23_FRONT]);
    rflMotorExecuteControl(&scara_arm->joints_motors[MOTOR_JOINT23_FRONT]);

    rflMotorUpdateStatus(&scara_arm->joints_motors[MOTOR_JOINT4]);
    rflMotorUpdateControl(&scara_arm->joints_motors[MOTOR_JOINT4]);

    rflMotorUpdateStatus(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT]);
    rflMotorUpdateStatus(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT]);
    rflMotorUpdateControl(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT]);
    rflMotorUpdateControl(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT]);

    for (uint16_t i = 0; i < 1000; i++) // 检查发送缓冲区是否已满，满则循环延时
        if (!can_is_secondary_transmit_buffer_full(BOARD_CAN2))
            break;
    rflRmMotorControl(ENGINEER_ARM_JOINTS_456_MOTORS_CAN_ORDINAL, ENGINEER_ARM_JOINTS_456_RM_MOTORS_CAN_SLAVE_ID,
                      (int16_t)rflMotorGetOutput(&scara_arm->joints_motors[MOTOR_JOINT4]),
                      (int16_t)rflMotorGetOutput(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT]),
                      (int16_t)rflMotorGetOutput(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT]), 0);
}

/**
 * @brief 设置机械臂电机控制模式
 */
void arm_motor_set_mode(engineer_scara_arm_s *scara_arm, rfl_motor_control_mode_e mode)
{
    if (mode != RFL_MOTOR_CONTROL_MODE_NO_FORCE && mode != RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE)
        return;

    rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], mode);
    rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], mode);
    rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT23_BACK], mode);
    rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT23_FRONT], mode);
    rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT4], mode);
    rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], mode);
    rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], mode);
}

/**
 * @brief 设置机械臂电机运动速度
 * @note 测试版 后续需要更新为动态速度规划
 */
void arm_motor_set_max_speed(engineer_scara_arm_s *scara_arm, engineer_scara_arm_mode_e mode, float base_speed)
{
    if (mode == ARM_MODE_START_UP)
    {
        base_speed /= 2.0f;
        rflMotorSetMaxSpeed(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], base_speed);
        rflMotorSetMaxSpeed(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], base_speed);
        rflMotorSetMaxSpeed(&scara_arm->joints_motors[MOTOR_JOINT23_BACK], base_speed);
        rflMotorSetMaxSpeed(&scara_arm->joints_motors[MOTOR_JOINT23_FRONT], base_speed * 2.0f);
        rflMotorSetMaxSpeed(&scara_arm->joints_motors[MOTOR_JOINT4], base_speed);
        rflMotorSetMaxSpeed(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], base_speed * 3.0f);
        rflMotorSetMaxSpeed(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], base_speed * 3.0f);
    }
    else
    {
        rflMotorSetMaxSpeed(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], base_speed * 3.2f);
        rflMotorSetMaxSpeed(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], base_speed * 3.2f);
        rflMotorSetMaxSpeed(&scara_arm->joints_motors[MOTOR_JOINT23_BACK], base_speed);
        rflMotorSetMaxSpeed(&scara_arm->joints_motors[MOTOR_JOINT23_FRONT], base_speed / JOINT2_REDUCTION);
        rflMotorSetMaxSpeed(&scara_arm->joints_motors[MOTOR_JOINT4], base_speed / 2.0f);
        rflMotorSetMaxSpeed(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], base_speed * 6.0f);
        rflMotorSetMaxSpeed(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], base_speed * 6.0f);
    }
}

/**
 * @brief 设置机械臂电机可达角度
 */
void arm_motor_set_angle_limit(engineer_scara_arm_s *scara_arm, engineer_scara_arm_mode_e mode)
{
    if (mode == ARM_MODE_START_UP)
    {
        rflMotorSetDegAngleLimit(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], RFL_ANGLE_FORMAT_DEGREE,
                                 ENGINEER_ARM_MOTOR_JOINT_1_INITIAL_MAX_ANGLE,
                                 ENGINEER_ARM_MOTOR_JOINT_1_INITIAL_MIN_ANGLE);
        rflMotorSetDegAngleLimit(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], RFL_ANGLE_FORMAT_DEGREE,
                                 ENGINEER_ARM_MOTOR_JOINT_1_INITIAL_MAX_ANGLE,
                                 ENGINEER_ARM_MOTOR_JOINT_1_INITIAL_MIN_ANGLE);
    }
    else if (mode != ARM_MODE_START_UP)
    {
        rflMotorSetDegAngleLimit(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], RFL_ANGLE_FORMAT_DEGREE,
                                 ENGINEER_ARM_MOTOR_JOINT_1_MAX_ANGLE, ENGINEER_ARM_MOTOR_JOINT_1_MIN_ANGLE);
        rflMotorSetDegAngleLimit(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], RFL_ANGLE_FORMAT_DEGREE,
                                 ENGINEER_ARM_MOTOR_JOINT_1_MAX_ANGLE, ENGINEER_ARM_MOTOR_JOINT_1_MIN_ANGLE);

        rflMotorSetDegAngleLimit(&scara_arm->joints_motors[MOTOR_JOINT23_BACK], RFL_ANGLE_FORMAT_DEGREE,
                                 ENGINEER_ARM_MOTOR_JOINT_23_BACK_MAX_ANGLE,
                                 ENGINEER_ARM_MOTOR_JOINT_23_BACK_MIN_ANGLE);

        rflMotorSetDegAngleLimit(&scara_arm->joints_motors[MOTOR_JOINT23_FRONT], RFL_ANGLE_FORMAT_DEGREE,
                                 ENGINEER_ARM_MOTOR_JOINT_23_FRONT_MAX_ANGLE,
                                 ENGINEER_ARM_MOTOR_JOINT_23_FRONT_MIN_ANGLE);

        rflMotorSetDegAngleLimit(&scara_arm->joints_motors[MOTOR_JOINT4], RFL_ANGLE_FORMAT_DEGREE,
                                 ENGINEER_ARM_MOTOR_JOINT_4_MAX_ANGLE, ENGINEER_ARM_MOTOR_JOINT_4_MIN_ANGLE);

        rflMotorSetDegAngleLimit(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_DEGREE,
                                 ENGINEER_ARM_MOTOR_JOINT_56_MAX_ANGLE, ENGINEER_ARM_MOTOR_JOINT_56_MIN_ANGLE);
        rflMotorSetDegAngleLimit(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_ANGLE_FORMAT_DEGREE,
                                 ENGINEER_ARM_MOTOR_JOINT_56_MAX_ANGLE, ENGINEER_ARM_MOTOR_JOINT_56_MIN_ANGLE);
    }
}
