#include "math.h"

#include "arm_control.h"

#include "board.h"

#include "arm_motor.h"

#include "algo_data_limiting.h"

static void no_force_control(engineer_scara_arm_s *scara_arm);
static bool starting_control(engineer_scara_arm_s *scara_arm);
static void joints_control(engineer_scara_arm_s *scara_arm);
static void pose_control_dbus(engineer_scara_arm_s *scara_arm);
static void pose_control_customer(engineer_scara_arm_s *scara_arm);

/**
 * @brief 获取用户输入 更新机械臂控制模式
 */
void arm_set_mode(engineer_scara_arm_s *scara_arm)
{
    static uint32_t key_timer = 0;

    char mode_control_key_value = scara_arm->dbus_rc->rc.s[1];
    int16_t homing_rocker_value = scara_arm->dbus_rc->rc.ch[4];

    // 长拨重新归位
    if (scara_arm->mode == ARM_MODE_NO_FORCE && homing_rocker_value > 600)
    {
        key_timer++;
        if (key_timer > 300)
        {
            key_timer = 0;

            scara_arm->mode = ARM_MODE_STARTING;
            scara_arm->is_arm_ready = false;
            for (uint8_t i = 0; i < 6; i++)
                scara_arm->is_joints_ready[i] = false;
        }
    }

    // 无力
    if (scara_arm->last_mode_control_key_value != ARM_NO_FORCE_MODE_RC_KEY_VALUE &&
        mode_control_key_value == ARM_NO_FORCE_MODE_RC_KEY_VALUE)
        scara_arm->mode = ARM_MODE_NO_FORCE;

    // 工作模式 关节/位姿控制
    if (scara_arm->is_arm_ready)
    {
        if (scara_arm->last_mode_control_key_value != ARM_JOINTS_MODE_RC_KEY_VALUE &&
            mode_control_key_value == ARM_JOINTS_MODE_RC_KEY_VALUE)
            scara_arm->mode = ARM_MODE_JOINTS;

        // if (scara_arm->last_mode_control_key_value != ARM_JOINTS_MODE_RC_KEY_VALUE &&
        //     mode_control_key_value == ARM_JOINTS_MODE_RC_KEY_VALUE)
        //     scara_arm->mode = ARM_MODE_POSE;
        // else if (scara_arm->last_mode_control_key_value != ARM_POSE_MODE_RC_KEY_VALUE &&
        //          mode_control_key_value == ARM_POSE_MODE_RC_KEY_VALUE)
        //     scara_arm->mode = ARM_MODE_JOINTS;

        // if (scara_arm->last_mode_control_key_value != ARM_JOINTS_MODE_RC_KEY_VALUE &&
        //     mode_control_key_value == ARM_JOINTS_MODE_RC_KEY_VALUE)
        //     scara_arm->mode = ARM_MODE_JOINTS;
        // else if (scara_arm->last_mode_control_key_value != ARM_POSE_MODE_RC_KEY_VALUE &&
        //          mode_control_key_value == ARM_POSE_MODE_RC_KEY_VALUE)
        //     scara_arm->mode = ARM_MODE_POSE;
        // else if (scara_arm->last_mode_control_key_value != ARM_POSE_MODE_RC_KEY_VALUE &&
        //          mode_control_key_value == ARM_POSE_MODE_RC_KEY_VALUE)
        //     scara_arm->mode = ARM_MODE_CUSTOMER;
    }

    scara_arm->last_mode_control_key_value = mode_control_key_value;
}

/**
 * @brief 根据机械臂控制模式 配置电机控制模式 更新机械臂控制量
 */
void arm_mode_control(engineer_scara_arm_s *scara_arm)
{
    // 模式切换时修改电机控制配置

    if (scara_arm->last_mode != ARM_MODE_NO_FORCE && scara_arm->mode == ARM_MODE_NO_FORCE)
    {
        arm_motor_set_mode(scara_arm, RFL_MOTOR_CONTROL_MODE_NO_FORCE);
    }
    else if (scara_arm->last_mode == ARM_MODE_NO_FORCE && scara_arm->mode != ARM_MODE_NO_FORCE)
    {
        arm_motor_set_mode(scara_arm, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        if (scara_arm->mode == ARM_MODE_STARTING)
            arm_motor_set_max_speed(scara_arm, 3.0f);
        else
            arm_motor_set_max_speed(scara_arm, 6.0f);
    }

    if (scara_arm->last_mode != ARM_MODE_STARTING && scara_arm->mode == ARM_MODE_STARTING)
    {
        arm_motor_set_angle_limit(scara_arm, scara_arm->mode);

        board_write_led_r(LED_ON);
    }
    else if (scara_arm->last_mode == ARM_MODE_STARTING && scara_arm->mode != ARM_MODE_STARTING)
    {
        arm_motor_set_angle_limit(scara_arm, scara_arm->mode);

        board_write_led_r(LED_OFF);
    }

    scara_arm->last_mode = scara_arm->mode;

    // 根据不同模式使用不同控制

    switch (scara_arm->mode)
    {
    case ARM_MODE_NO_FORCE:
        no_force_control(scara_arm);
        break;
    case ARM_MODE_STARTING:
        if (scara_arm->is_arm_ready = starting_control(scara_arm), scara_arm->is_arm_ready)
        {
            scara_arm->last_mode = ARM_MODE_STARTING;
            scara_arm->mode = ARM_MODE_NO_FORCE;
            arm_motor_set_mode(scara_arm, RFL_MOTOR_CONTROL_MODE_NO_FORCE);
        }
        break;
    case ARM_MODE_JOINTS:
        joints_control(scara_arm);
        break;
    case ARM_MODE_POSE:
        pose_control_dbus(scara_arm);
        break;
    case ARM_MODE_CUSTOMER:
        pose_control_customer(scara_arm);
        break;

    default:
        break;
    }
}

// 机械臂无力，模型的预期关节变量跟随实际关节变量
static void no_force_control(engineer_scara_arm_s *scara_arm)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        scara_arm->set_joints_value[i] = scara_arm->joints_value[i];
        scara_arm->set_pose_6d[i] = scara_arm->pose_6d[i];
    }
}

static bool starting_control(engineer_scara_arm_s *scara_arm)
{
    // /*关节2 运动到起始位置*/
    // if (scara_arm->is_joints_ready[1] == false)
    // {
    //     rflMotorSetMode(scara_arm->joint_23_motor + 0, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
    //     rflMotorSetAngle(scara_arm->joint_23_motor + 0, RFL_ANGLE_FORMAT_DEGREE, JOINT_2_START_ANGLE);

    //     if (fabsf(rflMotorGetAngle(scara_arm->joint_23_motor + 0, RFL_ANGLE_FORMAT_DEGREE) - JOINT_2_START_ANGLE) <
    //         1.0f)
    scara_arm->is_joints_ready[1] = true;
    // }

    // /*关节3 运动到起始位置*/
    // if (scara_arm->is_joints_ready[2] == false)
    // {
    //     rflMotorSetMode(scara_arm->joint_23_motor + 1, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
    //     rflMotorSetAngle(scara_arm->joint_23_motor + 1, RFL_ANGLE_FORMAT_DEGREE, JOINT_3_START_ANGLE);

    //     if (fabsf(rflMotorGetAngle(scara_arm->joint_23_motor + 1, RFL_ANGLE_FORMAT_DEGREE) - JOINT_3_START_ANGLE) <
    //         1.0f)
    scara_arm->is_joints_ready[2] = true;
    // }

    // /*关节4 通过磁编获取角度归中后运动到起始位置*/
    // if (scara_arm->is_joints_ready[3] == false)
    // {
    //     // if (scara_arm->encoder_angle[2] > ENGINEER_ARM_JOINT_4_MAX_ANGLE ||
    //     //     scara_arm->encoder_angle[2] < ENGINEER_ARM_JOINT_4_MIN_ANGLE)
    //     //     return false;
    //     rflMotorResetAngle(scara_arm->joint_4_motor + 0, RFL_ANGLE_FORMAT_DEGREE, scara_arm->encoder_angle[2]);

    //     rflMotorSetMode(scara_arm->joint_4_motor + 0, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
    //     rflMotorSetAngle(scara_arm->joint_4_motor + 0, RFL_ANGLE_FORMAT_DEGREE, JOINT_4_START_ANGLE);

    //     if (fabsf(rflMotorGetAngle(scara_arm->joint_4_motor + 0, RFL_ANGLE_FORMAT_DEGREE) - JOINT_4_START_ANGLE)
    //     < 1.0f)
    scara_arm->is_joints_ready[3] = true;
    // }

    // /*关节6 通过磁编获取角度归中后运动到起始位置*/
    // if (scara_arm->is_joints_ready[5] == false)
    // {
    //     // if (scara_arm->encoder_angle[3] > ENGINEER_ARM_JOINT_6_MAX_ANGLE ||
    //     //     scara_arm->encoder_angle[3] < ENGINEER_ARM_JOINT_6_MIN_ANGLE)
    //     //     return false;
    //     rflMotorResetAngle(scara_arm->joint_56_motor + 0, RFL_ANGLE_FORMAT_DEGREE,
    //                        /* JOINT_5_HOMING_ANGLE - */
    //                        (scara_arm->encoder_angle[3] * EFFECTOR_CONICAL_TOOTH_PAIR_TRANSMISSION_RATIO));
    //     rflMotorResetAngle(scara_arm->joint_56_motor + 1, RFL_ANGLE_FORMAT_DEGREE,
    //                        /* JOINT_5_HOMING_ANGLE + */
    //                        (scara_arm->encoder_angle[3] * EFFECTOR_CONICAL_TOOTH_PAIR_TRANSMISSION_RATIO));

    //     rflMotorSetMode(scara_arm->joint_56_motor + 0, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
    //     rflMotorSetMode(scara_arm->joint_56_motor + 1, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
    //     rflMotorSetAngle(scara_arm->joint_56_motor + 0, RFL_ANGLE_FORMAT_DEGREE, JOINT_6_START_ANGLE);
    //     rflMotorSetAngle(scara_arm->joint_56_motor + 1, RFL_ANGLE_FORMAT_DEGREE, JOINT_6_START_ANGLE);

    //     if ((fabsf(rflMotorGetAngle(scara_arm->joint_56_motor + 0, RFL_ANGLE_FORMAT_DEGREE) - JOINT_6_START_ANGLE)
    //     < 1.0f)
    //     &&
    //         (fabsf(rflMotorGetAngle(scara_arm->joint_56_motor + 1, RFL_ANGLE_FORMAT_DEGREE) - JOINT_6_START_ANGLE)
    //         < 1.0f))
    scara_arm->is_joints_ready[5] = true;
    // }

    // /*关节1 向固定方向运动通过机械限位触发归中*/
    // if (scara_arm->is_joints_ready[0] == false)
    // {
    //     if (rflMotorGetTorque(scara_arm->joint_1_motor + 0) < -JOINT_1_HOMING_TORQUE_THRESHOLD ||
    //         rflMotorGetTorque(scara_arm->joint_1_motor + 1) < -JOINT_1_HOMING_TORQUE_THRESHOLD)
    //     {
    //         rflMotorResetAngle(scara_arm->joint_1_motor + 0, RFL_ANGLE_FORMAT_DEGREE, JOINT_1_HOMING_ANGLE);
    //         rflMotorResetAngle(scara_arm->joint_1_motor + 1, RFL_ANGLE_FORMAT_DEGREE, JOINT_1_HOMING_ANGLE);
    scara_arm->is_joints_ready[0] = true;
    //     }

    //     rflMotorSetMode(scara_arm->joint_1_motor + 0, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
    //     rflMotorSetMode(scara_arm->joint_1_motor + 1, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
    //     rflMotorSetAngle(scara_arm->joint_1_motor + 0, RFL_ANGLE_FORMAT_DEGREE,
    //                      rflMotorGetAngle(scara_arm->joint_1_motor + 0, RFL_ANGLE_FORMAT_DEGREE) -
    //                          JOINT_1_HOMING_STEP_ANGLE);
    //     rflMotorSetAngle(scara_arm->joint_1_motor + 1, RFL_ANGLE_FORMAT_DEGREE,
    //                      rflMotorGetAngle(scara_arm->joint_1_motor + 1, RFL_ANGLE_FORMAT_DEGREE) -
    //                          JOINT_1_HOMING_STEP_ANGLE);
    // }

    // /*关节5 向固定方向运动通过微动开关触发归中*/
    // if (scara_arm->is_joints_ready[4] == false && scara_arm->is_joints_ready[5] == true)
    // {
    //     if (joint_5_is_in_home)
    //     {
    //         rflMotorResetAngle(scara_arm->joint_56_motor + 0, RFL_ANGLE_FORMAT_DEGREE, JOINT_5_HOMING_ANGLE);
    //         rflMotorResetAngle(scara_arm->joint_56_motor + 1, RFL_ANGLE_FORMAT_DEGREE, JOINT_5_HOMING_ANGLE);
    scara_arm->is_joints_ready[4] = true;
    //     }

    //     rflMotorSetMode(scara_arm->joint_56_motor + 0, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
    //     rflMotorSetMode(scara_arm->joint_56_motor + 1, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
    //     rflMotorSetAngle(scara_arm->joint_56_motor + 0, RFL_ANGLE_FORMAT_DEGREE,
    //                      rflMotorGetAngle(scara_arm->joint_56_motor + 0, RFL_ANGLE_FORMAT_DEGREE) +
    //                          JOINT_5_HOMING_STEP_ANGLE);
    //     rflMotorSetAngle(scara_arm->joint_56_motor + 1, RFL_ANGLE_FORMAT_DEGREE,
    //                      rflMotorGetAngle(scara_arm->joint_56_motor + 1, RFL_ANGLE_FORMAT_DEGREE) +
    //                          JOINT_5_HOMING_STEP_ANGLE);
    // }

    for (uint8_t i = 0; i < 6; i++)
    {
        scara_arm->set_joints_value[i] = scara_arm->joints_value[i];
        scara_arm->set_pose_6d[i] = scara_arm->pose_6d[i];
    }

    for (uint8_t i = 0; i < 6; i++)
        if (!scara_arm->is_joints_ready[i])
            return false;

    return true;
}

static void joints_control(engineer_scara_arm_s *scara_arm)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        scara_arm->set_pose_6d[i] = scara_arm->pose_6d[i];
    }

    scara_arm->set_joints_value[0] +=
        ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[3], ARM_RC_DEADLINE)) / 660.0f * JOINT_1_CONTROL_SEN);

    if (scara_arm->dbus_rc->rc.s[0] == 2)
        scara_arm->set_joints_value[1] +=
            ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[2], ARM_RC_DEADLINE)) / 660.0f * -JOINT_2_CONTROL_SEN);
    else if (scara_arm->dbus_rc->rc.s[0] == 3)
        scara_arm->set_joints_value[2] +=
            ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[2], ARM_RC_DEADLINE)) / 660.0f * -JOINT_3_CONTROL_SEN);
    else if (scara_arm->dbus_rc->rc.s[0] == 1)
        scara_arm->set_joints_value[3] +=
            ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[2], ARM_RC_DEADLINE)) / 660.0f * -JOINT_4_CONTROL_SEN);

    scara_arm->set_joints_value[4] +=
        ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[1], ARM_RC_DEADLINE)) / 660.0f * -JOINT_5_CONTROL_SEN);

    scara_arm->set_joints_value[5] +=
        ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[0], ARM_RC_DEADLINE)) / 660.0f * JOINT_6_CONTROL_SEN);
}

static void pose_control_dbus(engineer_scara_arm_s *scara_arm)
{
    scara_arm->set_pose_6d[0] +=
        ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[1], ARM_RC_DEADLINE)) / 660.0f * POSE_X_CONTROL_SEN);

    scara_arm->set_pose_6d[1] +=
        ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[2], ARM_RC_DEADLINE)) / 660.0f * -POSE_Y_CONTROL_SEN);

    scara_arm->set_pose_6d[2] +=
        ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[3], ARM_RC_DEADLINE)) / 660.0f * POSE_Z_CONTROL_SEN);

    if (scara_arm->dbus_rc->rc.s[0] == 1)
        scara_arm->set_pose_6d[3] +=
            ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[0], ARM_RC_DEADLINE)) / 660.0f * -POSE_AY_CONTROL_SEN);
    else if (scara_arm->dbus_rc->rc.s[0] == 3)
        scara_arm->set_pose_6d[4] +=
            ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[0], ARM_RC_DEADLINE)) / 660.0f * -POSE_AP_CONTROL_SEN);
    else if (scara_arm->dbus_rc->rc.s[0] == 2)
        scara_arm->set_pose_6d[5] +=
            ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[0], ARM_RC_DEADLINE)) / 660.0f * POSE_AR_CONTROL_SEN);
}

static void pose_control_customer(engineer_scara_arm_s *scara_arm)
{
    scara_arm->set_pose_6d[0] += (scara_arm->vt_customer_rc->x * CUSTOMER_X_CONTROL_SEN);

    scara_arm->set_pose_6d[1] += (scara_arm->vt_customer_rc->y * CUSTOMER_Y_CONTROL_SEN);

    scara_arm->set_pose_6d[2] += (scara_arm->vt_customer_rc->z * CUSTOMER_Z_CONTROL_SEN);

    scara_arm->set_pose_6d[3] +=
        (rflFloatLoopConstrain(scara_arm->vt_customer_rc->yaw, -RAD_PI, RAD_PI) * CUSTOMER_AY_CONTROL_SEN);
    scara_arm->set_pose_6d[4] +=
        (rflFloatLoopConstrain(scara_arm->vt_customer_rc->pitch, -RAD_PI, RAD_PI) * CUSTOMER_AP_CONTROL_SEN);
    scara_arm->set_pose_6d[5] +=
        (rflFloatLoopConstrain(scara_arm->vt_customer_rc->roll, -RAD_PI, RAD_PI) * CUSTOMER_AR_CONTROL_SEN);
}
