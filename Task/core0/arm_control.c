#include "math.h"

#include "arm_control.h"

#include "arm_motor.h"

#include "algo_data_limiting.h"

static bool joint_5_is_in_home = false;

static void no_force_control(engineer_scara_arm_s *arm);
static bool starting_control(engineer_scara_arm_s *arm);
static void joints_control(engineer_scara_arm_s *arm);
static void pose_control_dbus(engineer_scara_arm_s *arm);
static void pose_control_customer(engineer_scara_arm_s *arm);

static void record_custom_rc_cmd(engineer_scara_arm_s *arm);

/**
 * @brief 获取用户输入 更新机械臂控制模式
 */
void arm_set_mode(engineer_scara_arm_s *arm)
{
    static uint32_t key_timer = 0;

    char mode_control_key_value = arm->dr16_rc->rc.s[1];
    int16_t homing_rocker_value = arm->dr16_rc->rc.ch[4];

    // 长拨重新归位
    if (arm->mode == ARM_MODE_NO_FORCE && homing_rocker_value > 600)
    {
        key_timer++;
        if (key_timer > 300)
        {
            key_timer = 0;

            arm->mode = ARM_MODE_STARTING;
            arm->is_arm_ready = false;
            for (uint8_t i = 0; i < 6; i++)
                arm->is_joints_ready[i] = false;
        }
    }

    // 无力
    if (arm->last_mode_control_key_value != ARM_NO_FORCE_MODE_RC_KEY_VALUE &&
        mode_control_key_value == ARM_NO_FORCE_MODE_RC_KEY_VALUE)
        arm->mode = ARM_MODE_NO_FORCE;

    // 工作模式 关节/位姿控制
    if (arm->is_arm_ready)
    {
        // if (arm->last_mode_control_key_value != ARM_JOINTS_MODE_RC_KEY_VALUE &&
        //     mode_control_key_value == ARM_JOINTS_MODE_RC_KEY_VALUE)
        //     arm->mode = ARM_MODE_POSE;
        // else if (arm->last_mode_control_key_value != ARM_POSE_MODE_RC_KEY_VALUE &&
        //          mode_control_key_value == ARM_POSE_MODE_RC_KEY_VALUE)
        //     arm->mode = ARM_MODE_JOINTS;

        // if (arm->last_mode_control_key_value != ARM_JOINTS_MODE_RC_KEY_VALUE &&
        //     mode_control_key_value == ARM_JOINTS_MODE_RC_KEY_VALUE)
        //     arm->mode = ARM_MODE_JOINTS;
        // else if (arm->last_mode_control_key_value != ARM_POSE_MODE_RC_KEY_VALUE &&
        //          mode_control_key_value == ARM_POSE_MODE_RC_KEY_VALUE)
        //     arm->mode = ARM_MODE_POSE;
        // else if (arm->last_mode_control_key_value != ARM_POSE_MODE_RC_KEY_VALUE &&
        //          mode_control_key_value == ARM_POSE_MODE_RC_KEY_VALUE)
        //     arm->mode = ARM_MODE_CUSTOMER;
    }

    arm->last_mode_control_key_value = mode_control_key_value;
}

/**
 * @brief 根据机械臂控制模式 配置电机控制模式 更新机械臂控制量
 */
void arm_mode_control(engineer_scara_arm_s *arm)
{
    // 模式切换时修改电机控制配置

    if (arm->last_mode != ARM_MODE_NO_FORCE && arm->mode == ARM_MODE_NO_FORCE)
    {
        arm_motor_set_mode(arm, RFL_MOTOR_CONTROL_MODE_NO_FORCE);
    }
    else if (arm->last_mode == ARM_MODE_NO_FORCE && arm->mode != ARM_MODE_NO_FORCE)
    {
        arm_motor_set_mode(arm, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        if (arm->mode == ARM_MODE_STARTING)
            arm_motor_set_max_speed(arm, 1.0f);
        else
            arm_motor_set_max_speed(arm, 3.0f);
    }

    if (arm->last_mode != ARM_MODE_STARTING && arm->mode == ARM_MODE_STARTING)
    {
        arm_motor_set_angle_limit(arm, arm->mode);
    }
    else if (arm->last_mode == ARM_MODE_STARTING && arm->mode != ARM_MODE_STARTING)
    {
        arm_motor_set_angle_limit(arm, arm->mode);
    }

    arm->last_mode = arm->mode;

    // 根据不同模式使用不同控制

    switch (arm->mode)
    {
    case ARM_MODE_NO_FORCE:
        no_force_control(arm);
        break;
    case ARM_MODE_STARTING:
        if (arm->is_arm_ready = starting_control(arm), arm->is_arm_ready)
        {
            arm->last_mode = ARM_MODE_STARTING;
            arm->mode = ARM_MODE_NO_FORCE;
            arm_motor_set_mode(arm, RFL_MOTOR_CONTROL_MODE_NO_FORCE);
        }
        break;
    case ARM_MODE_JOINTS:
        joints_control(arm);
        break;
    case ARM_MODE_POSE:
        pose_control_dbus(arm);
        break;
    case ARM_MODE_CUSTOMER:
        pose_control_customer(arm);
        break;

    default:
        break;
    }

    record_custom_rc_cmd(arm);
}

// 机械臂无力，模型的预期关节变量跟随实际关节变量
static void no_force_control(engineer_scara_arm_s *arm)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        arm->set_joints_value[i] = arm->joints_value[i];
        arm->set_pose_6d[i] = arm->pose_6d[i];
    }
}

static bool starting_control(engineer_scara_arm_s *arm)
{
    /*关节3 通过磁编获取角度直接归中后需运动到起始位置*/
    if (arm->is_joints_ready[2] == false)
    {
        // if (arm->encoder_angle[1] > ENGINEER_ARM_JOINT_3_MAX_ANGLE ||
        //     arm->encoder_angle[1] < ENGINEER_ARM_JOINT_3_MIN_ANGLE)
        //     return false;
        rflMotorResetAngle(arm->joint_23_motor + 1, RFL_ANGLE_FORMAT_DEGREE, arm->encoder_angle[1]);

        rflMotorSetMode(arm->joint_23_motor + 1, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        rflMotorSetAngle(arm->joint_23_motor + 1, RFL_ANGLE_FORMAT_DEGREE, JOINT_3_START_ANGLE);

        if (fabsf(rflMotorGetAngle(arm->joint_23_motor + 1, RFL_ANGLE_FORMAT_DEGREE) - JOINT_3_START_ANGLE) < 1.0f)
            arm->is_joints_ready[2] = true;
    }

    /*关节4 通过磁编获取角度直接归中后需运动到起始位置*/
    if (arm->is_joints_ready[3] == false)
    {
        // if (arm->encoder_angle[2] > ENGINEER_ARM_JOINT_4_MAX_ANGLE ||
        //     arm->encoder_angle[2] < ENGINEER_ARM_JOINT_4_MIN_ANGLE)
        //     return false;
        rflMotorResetAngle(arm->joint_4_motor + 0, RFL_ANGLE_FORMAT_DEGREE, arm->encoder_angle[2]);

        rflMotorSetMode(arm->joint_4_motor + 0, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        rflMotorSetAngle(arm->joint_4_motor + 0, RFL_ANGLE_FORMAT_DEGREE, JOINT_4_START_ANGLE);

        if (fabsf(rflMotorGetAngle(arm->joint_4_motor + 0, RFL_ANGLE_FORMAT_DEGREE) - JOINT_4_START_ANGLE) < 1.0f)
            arm->is_joints_ready[3] = true;
    }

    /*关节6 通过磁编获取角度直接归中后需运动到起始位置*/
    if (arm->is_joints_ready[5] == false)
    {
        // if (arm->encoder_angle[3] > ENGINEER_ARM_JOINT_6_MAX_ANGLE ||
        //     arm->encoder_angle[3] < ENGINEER_ARM_JOINT_6_MIN_ANGLE)
        //     return false;
        rflMotorResetAngle(arm->joint_56_motor + 0, RFL_ANGLE_FORMAT_DEGREE,
                           /* JOINT_5_HOMING_ANGLE - */
                           (arm->encoder_angle[3] * EFFECTOR_CONICAL_TOOTH_PAIR_TRANSMISSION_RATIO));
        rflMotorResetAngle(arm->joint_56_motor + 1, RFL_ANGLE_FORMAT_DEGREE,
                           /* JOINT_5_HOMING_ANGLE + */
                           (arm->encoder_angle[3] * EFFECTOR_CONICAL_TOOTH_PAIR_TRANSMISSION_RATIO));

        rflMotorSetMode(arm->joint_56_motor + 0, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        rflMotorSetMode(arm->joint_56_motor + 1, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        rflMotorSetAngle(arm->joint_56_motor + 0, RFL_ANGLE_FORMAT_DEGREE, JOINT_6_START_ANGLE);
        rflMotorSetAngle(arm->joint_56_motor + 1, RFL_ANGLE_FORMAT_DEGREE, JOINT_6_START_ANGLE);

        if ((fabsf(rflMotorGetAngle(arm->joint_56_motor + 0, RFL_ANGLE_FORMAT_DEGREE) - JOINT_6_START_ANGLE) < 1.0f) &&
            (fabsf(rflMotorGetAngle(arm->joint_56_motor + 1, RFL_ANGLE_FORMAT_DEGREE) - JOINT_6_START_ANGLE) < 1.0f))
            arm->is_joints_ready[5] = true;
    }

    /*关节1 向固定方向运动通过机械限位触发归中*/
    if (arm->is_joints_ready[0] == false)
    {
        if (rflMotorGetTorque(arm->joint_1_motor + 0) < -JOINT_1_HOMING_TORQUE_THRESHOLD ||
            rflMotorGetTorque(arm->joint_1_motor + 1) < -JOINT_1_HOMING_TORQUE_THRESHOLD)
        {
            rflMotorResetAngle(arm->joint_1_motor + 0, RFL_ANGLE_FORMAT_DEGREE, JOINT_1_HOMING_ANGLE);
            rflMotorResetAngle(arm->joint_1_motor + 1, RFL_ANGLE_FORMAT_DEGREE, JOINT_1_HOMING_ANGLE);
            arm->is_joints_ready[0] = true;
        }

        rflMotorSetMode(arm->joint_1_motor + 0, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        rflMotorSetMode(arm->joint_1_motor + 1, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        rflMotorSetAngle(arm->joint_1_motor + 0, RFL_ANGLE_FORMAT_DEGREE,
                         rflMotorGetAngle(arm->joint_1_motor + 0, RFL_ANGLE_FORMAT_DEGREE) - JOINT_1_HOMING_STEP_ANGLE);
        rflMotorSetAngle(arm->joint_1_motor + 1, RFL_ANGLE_FORMAT_DEGREE,
                         rflMotorGetAngle(arm->joint_1_motor + 1, RFL_ANGLE_FORMAT_DEGREE) - JOINT_1_HOMING_STEP_ANGLE);
    }

    /*关节2 向固定方向运动通过机械限位触发归中*/
    if (arm->is_joints_ready[1] == false)
    {
        if (rflMotorGetTorque(arm->joint_23_motor + 0) > JOINT_2_HOMING_TORQUE_THRESHOLD)
        {
            rflMotorResetAngle(arm->joint_23_motor + 0, RFL_ANGLE_FORMAT_DEGREE, JOINT_2_HOMING_ANGLE);
            arm->is_joints_ready[1] = true;
        }

        rflMotorSetMode(arm->joint_23_motor + 0, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        rflMotorSetAngle(arm->joint_23_motor + 0, RFL_ANGLE_FORMAT_DEGREE,
                         arm->joint_23_motor[0].set_angle_.deg + JOINT_2_HOMING_STEP_ANGLE);
    }

    /*关节5 向固定方向运动通过微动开关触发归中*/
    // if (arm->is_joints_ready[4] == false && arm->is_joints_ready[5] == true)
    // {
    //     if (joint_5_is_in_home)
    //     {
    //         rflMotorResetAngle(arm->joint_56_motor + 0, RFL_ANGLE_FORMAT_DEGREE, JOINT_5_HOMING_ANGLE);
    //         rflMotorResetAngle(arm->joint_56_motor + 1, RFL_ANGLE_FORMAT_DEGREE, JOINT_5_HOMING_ANGLE);
    arm->is_joints_ready[4] = true;
    //     }

    //     rflMotorSetMode(arm->joint_56_motor + 0, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
    //     rflMotorSetMode(arm->joint_56_motor + 1, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
    //     rflMotorSetAngle(arm->joint_56_motor + 0, RFL_ANGLE_FORMAT_DEGREE,
    //                      rflMotorGetAngle(arm->joint_56_motor + 0, RFL_ANGLE_FORMAT_DEGREE) +
    //                          JOINT_5_HOMING_STEP_ANGLE);
    //     rflMotorSetAngle(arm->joint_56_motor + 1, RFL_ANGLE_FORMAT_DEGREE,
    //                      rflMotorGetAngle(arm->joint_56_motor + 1, RFL_ANGLE_FORMAT_DEGREE) +
    //                          JOINT_5_HOMING_STEP_ANGLE);
    // }

    for (uint8_t i = 0; i < 6; i++)
    {
        arm->set_joints_value[i] = arm->joints_value[i];
        arm->set_pose_6d[i] = arm->pose_6d[i];
    }

    for (uint8_t i = 0; i < 6; i++)
        if (!arm->is_joints_ready[i])
            return false;

    return true;
}

static void joints_control(engineer_scara_arm_s *arm)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        arm->set_pose_6d[i] = arm->pose_6d[i];
    }

    // arm->set_joints_value[0] +=
    //     ((float)(rflDeadZoneZero(arm->dr16_rc->rc.ch[3], ARM_RC_DEADLINE)) / 660.0f * JOINT_1_CONTROL_SEN);

    // if (arm->dr16_rc->rc.s[0] == 2)
    //     arm->set_joints_value[1] +=
    //         ((float)(rflDeadZoneZero(arm->dr16_rc->rc.ch[2], ARM_RC_DEADLINE)) / 660.0f * -JOINT_2_CONTROL_SEN);
    // else if (arm->dr16_rc->rc.s[0] == 3)
    //     arm->set_joints_value[2] +=
    //         ((float)(rflDeadZoneZero(arm->dr16_rc->rc.ch[2], ARM_RC_DEADLINE)) / 660.0f * -JOINT_3_CONTROL_SEN);
    // else if (arm->dr16_rc->rc.s[0] == 1)
    //     arm->set_joints_value[3] +=
    //         ((float)(rflDeadZoneZero(arm->dr16_rc->rc.ch[2], ARM_RC_DEADLINE)) / 660.0f * -JOINT_4_CONTROL_SEN);

    // arm->set_joints_value[4] +=
    //     ((float)(rflDeadZoneZero(arm->dr16_rc->rc.ch[1], ARM_RC_DEADLINE)) / 660.0f * -JOINT_5_CONTROL_SEN);

    // arm->set_joints_value[5] +=
    //     ((float)(rflDeadZoneZero(arm->dr16_rc->rc.ch[0], ARM_RC_DEADLINE)) / 660.0f * JOINT_6_CONTROL_SEN);
}

static void pose_control_dbus(engineer_scara_arm_s *arm)
{
    arm->set_pose_6d[0] +=
        ((float)(rflDeadZoneZero(arm->dr16_rc->rc.ch[1], ARM_RC_DEADLINE)) / 660.0f * POSE_X_CONTROL_SEN);

    arm->set_pose_6d[1] +=
        ((float)(rflDeadZoneZero(arm->dr16_rc->rc.ch[2], ARM_RC_DEADLINE)) / 660.0f * -POSE_Y_CONTROL_SEN);

    arm->set_pose_6d[2] +=
        ((float)(rflDeadZoneZero(arm->dr16_rc->rc.ch[3], ARM_RC_DEADLINE)) / 660.0f * POSE_Z_CONTROL_SEN);

    if (arm->dr16_rc->rc.s[0] == 1)
        arm->set_pose_6d[3] +=
            ((float)(rflDeadZoneZero(arm->dr16_rc->rc.ch[0], ARM_RC_DEADLINE)) / 660.0f * -POSE_AY_CONTROL_SEN);
    else if (arm->dr16_rc->rc.s[0] == 3)
        arm->set_pose_6d[4] +=
            ((float)(rflDeadZoneZero(arm->dr16_rc->rc.ch[0], ARM_RC_DEADLINE)) / 660.0f * -POSE_AP_CONTROL_SEN);
    else if (arm->dr16_rc->rc.s[0] == 2)
        arm->set_pose_6d[5] +=
            ((float)(rflDeadZoneZero(arm->dr16_rc->rc.ch[0], ARM_RC_DEADLINE)) / 660.0f * POSE_AR_CONTROL_SEN);
}

static void pose_control_customer(engineer_scara_arm_s *arm)
{
    arm->set_pose_6d[0] += ((arm->custom_cmd->x - arm->last_custom_rc_cmd[0]) * CUSTOMER_X_CONTROL_SEN);

    arm->set_pose_6d[1] += ((arm->custom_cmd->y - arm->last_custom_rc_cmd[1]) * CUSTOMER_Y_CONTROL_SEN);

    arm->set_pose_6d[2] += ((arm->custom_cmd->z - arm->last_custom_rc_cmd[2]) * CUSTOMER_Z_CONTROL_SEN);

    arm->set_pose_6d[3] += (rflFloatLoopConstrain(arm->custom_cmd->yaw - arm->last_custom_rc_cmd[3], -RAD_PI, RAD_PI) *
                            CUSTOMER_AY_CONTROL_SEN);
    arm->set_pose_6d[4] +=
        (rflFloatLoopConstrain(arm->custom_cmd->pitch - arm->last_custom_rc_cmd[4], -RAD_PI, RAD_PI) *
         CUSTOMER_AP_CONTROL_SEN);
    arm->set_pose_6d[5] += (rflFloatLoopConstrain(arm->custom_cmd->roll - arm->last_custom_rc_cmd[5], -RAD_PI, RAD_PI) *
                            CUSTOMER_AR_CONTROL_SEN);
}

static void record_custom_rc_cmd(engineer_scara_arm_s *arm)
{
    arm->last_custom_rc_cmd[0] = arm->custom_cmd->x;
    arm->last_custom_rc_cmd[1] = arm->custom_cmd->y;
    arm->last_custom_rc_cmd[2] = arm->custom_cmd->z;
    arm->last_custom_rc_cmd[3] = arm->custom_cmd->yaw;
    arm->last_custom_rc_cmd[4] = arm->custom_cmd->pitch;
    arm->last_custom_rc_cmd[5] = arm->custom_cmd->roll;
}
