#include "math.h"

#include "arm_control.h"

#include "board.h"

#include "arm_motor.h"

#include "algo_data_limiting.h"

static void no_force_control(engineer_scara_arm_s *scara_arm);
static void starting_control(engineer_scara_arm_s *scara_arm);
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
        if (key_timer == 240)
        {
            scara_arm->mode = ARM_MODE_START_UP;
            resetArmStartUpStatus(scara_arm->start_up_status);
        }
    }
    else
    {
        key_timer = 0;
    }

    // 无力
    if (scara_arm->last_mode_control_key_value != ARM_NO_FORCE_MODE_RC_KEY_VALUE &&
        mode_control_key_value == ARM_NO_FORCE_MODE_RC_KEY_VALUE)
        scara_arm->mode = ARM_MODE_NO_FORCE;

    // 工作模式 关节/位姿控制
    if (scara_arm->start_up_status == ARM_START_UP_OK)
    {
        if (scara_arm->last_mode_control_key_value != ARM_WORK_MODE_RC_KEY_VALUE &&
            mode_control_key_value == ARM_WORK_MODE_RC_KEY_VALUE)
            scara_arm->mode = ARM_MODE_JOINTS;
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
        if (scara_arm->mode == ARM_MODE_START_UP)
            arm_motor_set_max_speed(scara_arm, 1.0f);
        else
            arm_motor_set_max_speed(scara_arm, 2.0f);
    }

    if (scara_arm->last_mode != ARM_MODE_START_UP && scara_arm->mode == ARM_MODE_START_UP)
    {
        arm_motor_set_angle_limit(scara_arm, scara_arm->mode);

        board_write_led_r(LED_ON);
    }
    else if (scara_arm->last_mode == ARM_MODE_START_UP && scara_arm->mode != ARM_MODE_START_UP)
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
    case ARM_MODE_START_UP:
        if (starting_control(scara_arm), scara_arm->start_up_status == ARM_START_UP_OK)
        {
            // 启动完自动退出
            scara_arm->last_mode = ARM_MODE_START_UP;
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

static void starting_control(engineer_scara_arm_s *scara_arm)
{
    /* 关节1 向固定方向运动通过机械限位触发角度获取 */
    // setJointStartUpStateOk(JOINT_1, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    if (checkIfJointNotStartUp(JOINT_1, scara_arm->start_up_status))
    {
        rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], RFL_ANGLE_FORMAT_DEGREE,
                         rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], RFL_ANGLE_FORMAT_DEGREE) -
                             JOINT_1_HOMING_STEP_ANGLE);
        rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], RFL_ANGLE_FORMAT_DEGREE,
                         rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], RFL_ANGLE_FORMAT_DEGREE) -
                             JOINT_1_HOMING_STEP_ANGLE);

        if (rflMotorGetTorque(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT]) < -JOINT_1_HOMING_TORQUE_THRESHOLD ||
            rflMotorGetTorque(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT]) < -JOINT_1_HOMING_TORQUE_THRESHOLD)
        {
            rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], RFL_ANGLE_FORMAT_DEGREE,
                               JOINT_1_HOMING_ANGLE);
            rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], RFL_ANGLE_FORMAT_DEGREE,
                               JOINT_1_HOMING_ANGLE);
            setJointStartUpStateOk(JOINT_1, scara_arm->start_up_status);
            rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], RFL_MOTOR_CONTROL_MODE_NO_FORCE);
            rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], RFL_MOTOR_CONTROL_MODE_NO_FORCE);
        }
    }

    /* 关节2 运动到起始位置 */
    // setJointStartUpStateOk(JOINT_2, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    if (checkIfJointNotStartUp(JOINT_2, scara_arm->start_up_status))
    {
        rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT23_BACK], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT23_BACK], RFL_ANGLE_FORMAT_DEGREE, JOINT_2_START_ANGLE);

        if (fabsf(rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT23_BACK], RFL_ANGLE_FORMAT_DEGREE) -
                  JOINT_2_START_ANGLE) < 1.0f)
        {
            setJointStartUpStateOk(JOINT_2, scara_arm->start_up_status);
            rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT23_BACK], RFL_MOTOR_CONTROL_MODE_NO_FORCE);
        }
    }

    /* 关节3 运动到起始位置 */
    // setJointStartUpStateOk(JOINT_3, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    if (checkIfJointNotStartUp(JOINT_3, scara_arm->start_up_status))
    {
        rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT23_FRONT], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT23_FRONT], RFL_ANGLE_FORMAT_DEGREE, JOINT_3_START_ANGLE);

        if (fabsf(rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT23_FRONT], RFL_ANGLE_FORMAT_DEGREE) -
                  JOINT_3_START_ANGLE) < 1.0f)
        {
            setJointStartUpStateOk(JOINT_3, scara_arm->start_up_status);
            rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT23_FRONT], RFL_MOTOR_CONTROL_MODE_NO_FORCE);
        }
    }

    /* 关节4 向固定方向运动通过机械限位触发角度获取后运动到起始位置 */
    setJointStartUpStateOk(JOINT_4, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    setJointStartUpStateOk(JOINT_4 + JOINT45_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status);
    if (checkIfJointNotStartUp(JOINT_4, scara_arm->start_up_status))
    {
        if (checkIfJointNotStartUp(JOINT_4 + JOINT45_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status))
        {
            rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT4], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
            rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT4], RFL_ANGLE_FORMAT_DEGREE,
                             rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT4], RFL_ANGLE_FORMAT_DEGREE) -
                                 JOINT_4_HOMING_STEP_ANGLE);

            if (rflMotorGetTorque(&scara_arm->joints_motors[MOTOR_JOINT4]) < -JOINT_4_HOMING_TORQUE_THRESHOLD)
            {
                rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT4], RFL_ANGLE_FORMAT_DEGREE,
                                   JOINT_4_HOMING_ANGLE);
                setJointStartUpStateOk(JOINT_4 + JOINT45_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status);
            }
        }
        else
        {
            rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT4], RFL_ANGLE_FORMAT_DEGREE, JOINT_4_START_ANGLE);

            if (fabsf(rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT4], RFL_ANGLE_FORMAT_DEGREE) -
                      JOINT_4_START_ANGLE) < 1.0f)
            {
                setJointStartUpStateOk(JOINT_4, scara_arm->start_up_status);
                rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT4], RFL_MOTOR_CONTROL_MODE_NO_FORCE);
            }
        }
    }

    /* 关节6 通过磁编获取角度归中后运动到起始位置 */
    setJointStartUpStateOk(JOINT_6, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    if (checkIfJointNotStartUp(JOINT_6, scara_arm->start_up_status))
    {
        rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_DEGREE,
                           /* JOINT_5_HOMING_ANGLE - */
                           (scara_arm->joint_6_encoder_angle * END_BEVEL_GEAR_SET_REDUCTION));
        rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_ANGLE_FORMAT_DEGREE,
                           /* JOINT_5_HOMING_ANGLE + */
                           (scara_arm->joint_6_encoder_angle * END_BEVEL_GEAR_SET_REDUCTION));

        rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_DEGREE, JOINT_6_START_ANGLE);
        rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_ANGLE_FORMAT_DEGREE, JOINT_6_START_ANGLE);

        if ((fabsf(rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_DEGREE) -
                   JOINT_6_START_ANGLE) < 1.0f) &&
            (fabsf(rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_ANGLE_FORMAT_DEGREE) -
                   JOINT_6_START_ANGLE) < 1.0f))
        {
            setJointStartUpStateOk(JOINT_6, scara_arm->start_up_status);
            rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_MOTOR_CONTROL_MODE_NO_FORCE);
            rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_MOTOR_CONTROL_MODE_NO_FORCE);
        }
    }

    /* 关节5 当关节6启动完毕后 向固定方向运动通过机械限位触发角度获取后运动到起始位置 */
    setJointStartUpStateOk(JOINT_5, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    setJointStartUpStateOk(JOINT_5 + JOINT45_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status);
    if (!checkIfJointNotStartUp(JOINT_6, scara_arm->start_up_status) &&
        checkIfJointNotStartUp(JOINT_5, scara_arm->start_up_status))
    {
        if (checkIfJointNotStartUp(JOINT_5 + JOINT45_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status))
        {
            rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
            rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
            rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_DEGREE,
                             rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_DEGREE) +
                                 JOINT_5_HOMING_STEP_ANGLE);
            rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_ANGLE_FORMAT_DEGREE,
                             rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_ANGLE_FORMAT_DEGREE) +
                                 JOINT_5_HOMING_STEP_ANGLE);

            if (rflMotorGetTorque(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT]) > JOINT_1_HOMING_TORQUE_THRESHOLD ||
                rflMotorGetTorque(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT]) > JOINT_1_HOMING_TORQUE_THRESHOLD)
            {
                rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_DEGREE,
                                   JOINT_5_HOMING_ANGLE);
                rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_ANGLE_FORMAT_DEGREE,
                                   JOINT_5_HOMING_ANGLE);
                setJointStartUpStateOk(JOINT_5 + JOINT45_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status);
            }
        }
        else
        {
            rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_DEGREE,
                             JOINT_5_START_ANGLE);
            rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_ANGLE_FORMAT_DEGREE,
                             JOINT_5_START_ANGLE);

            if ((fabsf(rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_DEGREE) -
                       JOINT_5_START_ANGLE) < 1.0f) &&
                (fabsf(rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_ANGLE_FORMAT_DEGREE) -
                       JOINT_5_START_ANGLE) < 1.0f))
            {
                setJointStartUpStateOk(JOINT_5, scara_arm->start_up_status);
                rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_MOTOR_CONTROL_MODE_NO_FORCE);
                rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_MOTOR_CONTROL_MODE_NO_FORCE);
            }
        }
    }

    for (uint8_t i = 0; i < 6; i++)
    {
        scara_arm->set_joints_value[i] = scara_arm->joints_value[i];
        scara_arm->set_pose_6d[i] = scara_arm->pose_6d[i];
    }
}

static void joints_control(engineer_scara_arm_s *scara_arm)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        scara_arm->set_pose_6d[i] = scara_arm->pose_6d[i];
    }

    scara_arm->set_joints_value[JOINT_1] +=
        ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[3], ARM_RC_DEADLINE)) / 660.0f * JOINT_1_CONTROL_SEN);

    if (scara_arm->dbus_rc->rc.s[0] == 2)
        scara_arm->set_joints_value[JOINT_2] +=
            ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[2], ARM_RC_DEADLINE)) / 660.0f * -JOINT_2_CONTROL_SEN);
    else if (scara_arm->dbus_rc->rc.s[0] == 3)
        scara_arm->set_joints_value[JOINT_3] +=
            ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[2], ARM_RC_DEADLINE)) / 660.0f * -JOINT_3_CONTROL_SEN);
    else if (scara_arm->dbus_rc->rc.s[0] == 1)
        scara_arm->set_joints_value[JOINT_4] +=
            ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[2], ARM_RC_DEADLINE)) / 660.0f * -JOINT_4_CONTROL_SEN);

    scara_arm->set_joints_value[JOINT_5] +=
        ((float)(rflDeadZoneZero(scara_arm->dbus_rc->rc.ch[1], ARM_RC_DEADLINE)) / 660.0f * -JOINT_5_CONTROL_SEN);

    scara_arm->set_joints_value[JOINT_6] +=
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
