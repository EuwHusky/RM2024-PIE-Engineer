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
    rflAngleUpdate(&config.max_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_1_INITIAL_MAX_ANGLE);
    rflAngleUpdate(&config.min_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_1_INITIAL_MIN_ANGLE);
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

    config.is_reversed = true;
    config.can_ordinal = ENGINEER_ARM_JOINTS_123_MOTORS_CAN_ORDINAL;
    config.master_can_id = 0x201;
    rflMotorInit(scara_arm->joint_1_motor + 0, &config);

    config.is_reversed = false;
    config.can_ordinal = ENGINEER_ARM_JOINTS_123_MOTORS_CAN_ORDINAL;
    config.master_can_id = 0x202;
    rflMotorInit(scara_arm->joint_1_motor + 1, &config);

    /* 关节2、3 各一个电机 */

    rflMotorGetDefaultConfig(&config, RFL_MOTOR_DM_J8009_2EC, RFL_MOTOR_CONTROLLER_DAMIAO);
    config.control_period_factor = 1.0f;

    config.is_reversed = false;
    rflAngleUpdate(&config.max_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_2_MAX_ANGLE);
    rflAngleUpdate(&config.min_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_2_MIN_ANGLE);
    config.can_ordinal = ENGINEER_ARM_JOINTS_123_MOTORS_CAN_ORDINAL;
    config.master_can_id = 0x02;
    config.slave_can_id = 0x00;
    rflMotorInit(scara_arm->joint_23_motor + 0, &config);

    config.is_reversed = true;
    rflAngleUpdate(&config.max_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_3_MAX_ANGLE);
    rflAngleUpdate(&config.min_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_3_MIN_ANGLE);
    config.can_ordinal = ENGINEER_ARM_JOINTS_123_MOTORS_CAN_ORDINAL;
    config.master_can_id = 0x03;
    config.slave_can_id = 0x01;
    rflMotorInit(scara_arm->joint_23_motor + 1, &config);

    /* 关节4 一个电机 */
    rflMotorGetDefaultConfig(&config, RFL_MOTOR_RM_M3508, RFL_MOTOR_CONTROLLER_PID);
    config.control_period_factor = 1.0f;
    config.is_reversed = true;
    rflAngleUpdate(&config.max_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_4_MAX_ANGLE);
    rflAngleUpdate(&config.min_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_4_MIN_ANGLE);
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
    rflMotorInit(scara_arm->joint_4_motor + 0, &config);

    /* 关节56 共用两个电机 */
    rflMotorGetDefaultConfig(&config, RFL_MOTOR_RM_M2006, RFL_MOTOR_CONTROLLER_PID);
    config.control_period_factor = 1.0f;
    config.effector_transmission_ratio = RM_M2006_REDUCTION_RATIO * EFFECTOR_TIMING_BELT_TRANSMISSION_RATIO;
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

    rflAngleUpdate(&config.max_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_56_MOTOR_MAX_ANGLE);
    rflAngleUpdate(&config.min_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_56_MOTOR_MIN_ANGLE);
    config.is_reversed = true;
    config.can_ordinal = ENGINEER_ARM_JOINTS_456_MOTORS_CAN_ORDINAL;
    config.master_can_id = 0x202;
    rflMotorInit(scara_arm->joint_56_motor + 0, &config);

    rflAngleUpdate(&config.max_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_56_MOTOR_MAX_ANGLE);
    rflAngleUpdate(&config.min_angle, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_56_MOTOR_MIN_ANGLE);
    config.is_reversed = false;
    config.can_ordinal = ENGINEER_ARM_JOINTS_456_MOTORS_CAN_ORDINAL;
    config.master_can_id = 0x203;
    rflMotorInit(scara_arm->joint_56_motor + 1, &config);
}

/**
 * @brief 更新机械臂电机状态量并执行控制
 */
void arm_motor_update_and_execute(engineer_scara_arm_s *scara_arm)
{
    // 设定电机角度

    if (scara_arm->mode != ARM_MODE_STARTING)
    {
        rflMotorSetAngle(scara_arm->joint_1_motor + 0, RFL_ANGLE_FORMAT_DEGREE,
                         scara_arm->set_joints_value[0] * ENGINEER_ARM_JOINT_1_DISTANCE_TO_ANGLE_FACTOR);
        rflMotorSetAngle(scara_arm->joint_1_motor + 1, RFL_ANGLE_FORMAT_DEGREE,
                         scara_arm->set_joints_value[0] * ENGINEER_ARM_JOINT_1_DISTANCE_TO_ANGLE_FACTOR);

        rflMotorSetAngle(scara_arm->joint_23_motor + 0, RFL_ANGLE_FORMAT_RADIAN, scara_arm->set_joints_value[1]);

        rflMotorSetAngle(scara_arm->joint_23_motor + 1, RFL_ANGLE_FORMAT_RADIAN, scara_arm->set_joints_value[2]);

        rflMotorSetAngle(scara_arm->joint_4_motor + 0, RFL_ANGLE_FORMAT_RADIAN, scara_arm->set_joints_value[3]);

        rflMotorSetAngle(scara_arm->joint_56_motor + 0, RFL_ANGLE_FORMAT_RADIAN,
                         ((scara_arm->set_joints_value[4] * EFFECTOR_TIMING_BELT_TRANSMISSION_RATIO) -
                          (scara_arm->set_joints_value[5] * EFFECTOR_TIMING_BELT_TRANSMISSION_RATIO *
                           EFFECTOR_CONICAL_TOOTH_PAIR_TRANSMISSION_RATIO)) *
                             0.5f);
        rflMotorSetAngle(scara_arm->joint_56_motor + 1, RFL_ANGLE_FORMAT_RADIAN,
                         ((scara_arm->set_joints_value[4] * EFFECTOR_TIMING_BELT_TRANSMISSION_RATIO) +
                          (scara_arm->set_joints_value[5] * EFFECTOR_TIMING_BELT_TRANSMISSION_RATIO *
                           EFFECTOR_CONICAL_TOOTH_PAIR_TRANSMISSION_RATIO)) *
                             0.5f);
    }

    // 电机更新&执行

    rflMotorUpdateStatus(scara_arm->joint_1_motor + 0);
    rflMotorUpdateStatus(scara_arm->joint_1_motor + 1);
    rflMotorUpdateControl(scara_arm->joint_1_motor + 0);
    rflMotorUpdateControl(scara_arm->joint_1_motor + 1);

    while (can_is_secondary_transmit_buffer_full(BOARD_CAN1)) // 检查发送缓冲区是否已满
        ;
    rflRmMotorControl(ENGINEER_ARM_JOINTS_123_MOTORS_CAN_ORDINAL, ENGINEER_ARM_JOINTS_123_RM_MOTORS_CAN_SLAVE_ID,
                      (int16_t)rflMotorGetOutput(scara_arm->joint_1_motor + 0),
                      (int16_t)rflMotorGetOutput(scara_arm->joint_1_motor + 1), 0, 0);

    rflMotorUpdateStatus(scara_arm->joint_23_motor + 0);
    rflMotorUpdateControl(scara_arm->joint_23_motor + 0);
    rflMotorExecuteControl(scara_arm->joint_23_motor + 0);

    rflMotorUpdateStatus(scara_arm->joint_23_motor + 1);
    rflMotorUpdateControl(scara_arm->joint_23_motor + 1);
    rflMotorExecuteControl(scara_arm->joint_23_motor + 1);

    // rflMotorUpdateStatus(scara_arm->joint_4_motor + 0);
    // rflMotorUpdateControl(scara_arm->joint_4_motor + 0);

    // rflMotorUpdateStatus(scara_arm->joint_56_motor + 0);
    // rflMotorUpdateStatus(scara_arm->joint_56_motor + 1);
    // rflMotorUpdateControl(scara_arm->joint_56_motor + 0);
    // rflMotorUpdateControl(scara_arm->joint_56_motor + 1);

    // while (can_is_secondary_transmit_buffer_full(BOARD_CAN2)) // 检查发送缓冲区是否已满
    //     ;
    // rflRmMotorControl(ENGINEER_ARM_JOINTS_456_MOTORS_CAN_ORDINAL, ENGINEER_ARM_JOINTS_456_RM_MOTORS_CAN_SLAVE_ID,
    //                   (int16_t)rflMotorGetOutput(scara_arm->joint_4_motor + 0),
    //                   (int16_t)rflMotorGetOutput(scara_arm->joint_56_motor + 0),
    //                   (int16_t)rflMotorGetOutput(scara_arm->joint_56_motor + 1), 0);
}

/**
 * @brief 设置机械臂电机控制模式
 */
void arm_motor_set_mode(engineer_scara_arm_s *scara_arm, rfl_motor_control_mode_e mode)
{
    if (mode != RFL_MOTOR_CONTROL_MODE_NO_FORCE && mode != RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE)
        return;

    rflMotorSetMode(scara_arm->joint_1_motor + 0, mode);
    rflMotorSetMode(scara_arm->joint_1_motor + 1, mode);
    rflMotorSetMode(scara_arm->joint_23_motor + 0, mode);
    rflMotorSetMode(scara_arm->joint_23_motor + 1, mode);
    rflMotorSetMode(scara_arm->joint_4_motor + 0, mode);
    rflMotorSetMode(scara_arm->joint_56_motor + 0, mode);
    rflMotorSetMode(scara_arm->joint_56_motor + 1, mode);
}

/**
 * @brief 设置机械臂电机运动速度
 * @note 测试版 后续需要更新为动态速度规划
 */
void arm_motor_set_max_speed(engineer_scara_arm_s *scara_arm, float max_speed)
{
    rflMotorSetMaxSpeed(scara_arm->joint_1_motor + 0, max_speed);
    rflMotorSetMaxSpeed(scara_arm->joint_1_motor + 1, max_speed);
    rflMotorSetMaxSpeed(scara_arm->joint_23_motor + 0, max_speed);
    rflMotorSetMaxSpeed(scara_arm->joint_23_motor + 1, max_speed);
    rflMotorSetMaxSpeed(scara_arm->joint_4_motor + 0, max_speed);
    rflMotorSetMaxSpeed(scara_arm->joint_56_motor + 0, max_speed);
    rflMotorSetMaxSpeed(scara_arm->joint_56_motor + 1, max_speed);
}

/**
 * @brief 设置机械臂电机可达角度
 */
void arm_motor_set_angle_limit(engineer_scara_arm_s *scara_arm, arm_mode_e mode)
{
    if (mode == ARM_MODE_STARTING)
    {
        rflMotorSetDegAngleLimit(scara_arm->joint_1_motor + 0, RFL_ANGLE_FORMAT_DEGREE,
                                 ENGINEER_ARM_JOINT_1_INITIAL_MAX_ANGLE, ENGINEER_ARM_JOINT_1_INITIAL_MIN_ANGLE);
        rflMotorSetDegAngleLimit(scara_arm->joint_1_motor + 1, RFL_ANGLE_FORMAT_DEGREE,
                                 ENGINEER_ARM_JOINT_1_INITIAL_MAX_ANGLE, ENGINEER_ARM_JOINT_1_INITIAL_MIN_ANGLE);

        // rflMotorSetDegAngleLimit(scara_arm->joint_56_motor + 0, RFL_ANGLE_FORMAT_DEGREE,
        //                          ENGINEER_ARM_JOINT_56_MOTOR_MAX_ANGLE, ENGINEER_ARM_JOINT_56_MOTOR_MIN_ANGLE);
        // rflMotorSetDegAngleLimit(scara_arm->joint_56_motor + 1, RFL_ANGLE_FORMAT_DEGREE,
        //                          ENGINEER_ARM_JOINT_56_MOTOR_MAX_ANGLE, ENGINEER_ARM_JOINT_56_MOTOR_MIN_ANGLE);
    }
    else if (mode != ARM_MODE_STARTING)
    {
        rflMotorSetDegAngleLimit(scara_arm->joint_1_motor + 0, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_1_MAX_ANGLE,
                                 ENGINEER_ARM_JOINT_1_MIN_ANGLE);
        rflMotorSetDegAngleLimit(scara_arm->joint_1_motor + 1, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_1_MAX_ANGLE,
                                 ENGINEER_ARM_JOINT_1_MIN_ANGLE);

        rflMotorSetDegAngleLimit(scara_arm->joint_23_motor + 0, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_2_MAX_ANGLE,
                                 ENGINEER_ARM_JOINT_2_MIN_ANGLE);

        rflMotorSetDegAngleLimit(scara_arm->joint_23_motor + 1, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_3_MAX_ANGLE,
                                 ENGINEER_ARM_JOINT_3_MIN_ANGLE);

        rflMotorSetDegAngleLimit(scara_arm->joint_4_motor, RFL_ANGLE_FORMAT_DEGREE, ENGINEER_ARM_JOINT_4_MAX_ANGLE,
                                 ENGINEER_ARM_JOINT_4_MIN_ANGLE);

        rflMotorSetDegAngleLimit(scara_arm->joint_56_motor + 0, RFL_ANGLE_FORMAT_DEGREE,
                                 ENGINEER_ARM_JOINT_56_MOTOR_MAX_ANGLE, ENGINEER_ARM_JOINT_56_MOTOR_MIN_ANGLE);
        rflMotorSetDegAngleLimit(scara_arm->joint_56_motor + 1, RFL_ANGLE_FORMAT_DEGREE,
                                 ENGINEER_ARM_JOINT_56_MOTOR_MAX_ANGLE, ENGINEER_ARM_JOINT_56_MOTOR_MIN_ANGLE);
    }
}
