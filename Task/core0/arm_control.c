#include "math.h"

#include "arm_control.h"

#include "board.h"

#include "arm_motor.h"

#include "algo_data_limiting.h"

#include "remote_control.h"

#include "storage_task.h"

static void control_value_process(engineer_scara_arm_s *scara_arm);

static void no_force_control(engineer_scara_arm_s *scara_arm);
static void starting_control(engineer_scara_arm_s *scara_arm);
#if USE_JOINTS_CONTROL
static void joints_control(engineer_scara_arm_s *scara_arm);
#endif
static void pose_control(engineer_scara_arm_s *scara_arm);
static void move_homing_control(engineer_scara_arm_s *scara_arm);
static void operation_homing_control(engineer_scara_arm_s *scara_arm);
static void silver_mining_control(engineer_scara_arm_s *scara_arm);
static void gold_mining_control(engineer_scara_arm_s *scara_arm);
static void storage_push_control(engineer_scara_arm_s *scara_arm);
static void storage_pop_control(engineer_scara_arm_s *scara_arm);

/**
 * @brief 根据机械臂控制模式 配置电机控制模式 更新机械臂控制量
 */
void arm_mode_control(engineer_scara_arm_s *scara_arm)
{
    // 处理控制量

    control_value_process(scara_arm);

    // 模式切换时修改机械臂设定

    scara_arm->last_behavior = scara_arm->behavior;
    scara_arm->behavior = getEngineerCurrentBehavior();

    if (scara_arm->last_behavior != scara_arm->behavior)
    {
        if (scara_arm->behavior == ENGINEER_BEHAVIOR_DISABLE)
        {
            arm_motor_set_mode(scara_arm, RFL_MOTOR_CONTROL_MODE_NO_FORCE);
            setArmGrabMode(false);
        }
        else if (scara_arm->behavior != ENGINEER_BEHAVIOR_DISABLE)
        {
            arm_motor_set_mode(scara_arm, RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        }

        if (scara_arm->behavior == ENGINEER_BEHAVIOR_RESET)
        {
            resetArmStartUpStatus(scara_arm->start_up_status);
            arm_motor_set_angle_limit(scara_arm, true);
        }
        else if (getEngineerLastBehavior() == ENGINEER_BEHAVIOR_RESET && scara_arm->behavior != ENGINEER_BEHAVIOR_RESET)
        {
            arm_motor_set_angle_limit(scara_arm, false);
        }
    }

    // 关节速度控制

    arm_motor_set_max_speed(scara_arm);

    // 根据不同模式使用不同控制

    switch (scara_arm->behavior)
    {
    case ENGINEER_BEHAVIOR_DISABLE:
        no_force_control(scara_arm);
        break;
    case ENGINEER_BEHAVIOR_RESET:
        starting_control(scara_arm);
        break;
    case ENGINEER_BEHAVIOR_MANUAL_OPERATION:
#if USE_JOINTS_CONTROL
        joints_control(scara_arm);
#else
        pose_control(scara_arm);
#endif
        break;
    case ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING:
        move_homing_control(scara_arm);
        break;
    case ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING:
        operation_homing_control(scara_arm);
        break;
    case ENGINEER_BEHAVIOR_AUTO_SILVER_MINING:
        silver_mining_control(scara_arm);
        break;
    case ENGINEER_BEHAVIOR_AUTO_GOLD_MINING:
        gold_mining_control(scara_arm);
        break;
    case ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH:
        storage_push_control(scara_arm);
        break;
    case ENGINEER_BEHAVIOR_AUTO_STORAGE_POP:
        storage_pop_control(scara_arm);
        break;

    default:
        // 其他模式什么都不做 保持不动
        break;
    }
}

static void control_value_process(engineer_scara_arm_s *scara_arm)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        if (i < 3)
            rflFirstOrderFilterCali(&scara_arm->cc_pose_filter[i], scara_arm->customer_controller->pose[i]);
        else
            rflFirstOrderFilterCali(&scara_arm->cc_pose_filter[i],
                                    rflFloatLoopConstrain(scara_arm->customer_controller->pose[i], -RAD_PI, RAD_PI));

        scara_arm->cc_pose_6d[i] = scara_arm->cc_pose_filter[i].out;
    }

    if (!checkIfCustomerControllerKeyPressed(scara_arm->customer_controller->key, CC_ROUGHLY))
    {
        for (uint8_t i = 0; i < 3; i++)
        {
            scara_arm->local_pos_memory[i] = scara_arm->pose_6d[i];
            scara_arm->cc_pos_memory[i] = scara_arm->cc_pose_6d[i];
        }
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
        // scara_arm->set_joints_value[0] -= JOINT_1_HOMING_STEP_ANGLE;

        // if ((rflMotorGetTorque(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT]) < -JOINT_1_HOMING_TORQUE_THRESHOLD ||
        //      rflMotorGetTorque(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT]) < -JOINT_1_HOMING_TORQUE_THRESHOLD) &&
        //     ((fabsf(rflMotorGetSpeed(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT])) < 0.1f) ||
        //      (fabsf(rflMotorGetSpeed(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT])) < 0.1f)))

        rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], RFL_MOTOR_CONTROL_MODE_NO_FORCE);
        rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], RFL_MOTOR_CONTROL_MODE_NO_FORCE);

        if (fabsf(rflMotorGetSpeed(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT])) < 0.02f)
        {
            scara_arm->joint_1_homing_timer =
                scara_arm->joint_1_homing_timer > 254 ? 255 : scara_arm->joint_1_homing_timer + 1;

            if (scara_arm->joint_1_homing_timer == 255)
            {
                rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], RFL_ANGLE_FORMAT_DEGREE,
                                   JOINT_1_HOMING_ANGLE, false);
                rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], RFL_ANGLE_FORMAT_DEGREE,
                                   JOINT_1_HOMING_ANGLE, false);
                rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
                rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
                scara_arm->joint_1_homing_timer = 0;
                setJointStartUpStateOk(JOINT_1, scara_arm->start_up_status);
            }
        }
        else
        {
            scara_arm->joint_1_homing_timer = 0;
        }
    }

    /* 关节2 运动到起始位置 */
    // setJointStartUpStateOk(JOINT_2, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    if (checkIfJointNotStartUp(JOINT_2, scara_arm->start_up_status))
    {
        scara_arm->set_joints_value[JOINT_2] = JOINT_2_START_ANGLE * DEGREE_TO_RADIAN_FACTOR;

        if (fabsf(scara_arm->joints_value[JOINT_2] - JOINT_2_START_ANGLE * DEGREE_TO_RADIAN_FACTOR) < 0.02f)
        {
            setJointStartUpStateOk(JOINT_2, scara_arm->start_up_status);
        }
    }

    /* 关节3 运动到起始位置 */
    // setJointStartUpStateOk(JOINT_3, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    if (checkIfJointNotStartUp(JOINT_3, scara_arm->start_up_status))
    {
        // scara_arm->set_joints_value[JOINT_3] =
        //     (JOINT_3_START_ANGLE -
        //      (((JOINT_2_START_ANGLE * JOINT2_REDUCTION) -
        //        rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT23_BACK], RFL_ANGLE_FORMAT_DEGREE)) /
        //       (ENGINEER_ARM_MOTOR_JOINT_23_BACK_MAX_ANGLE - ENGINEER_ARM_MOTOR_JOINT_23_BACK_MIN_ANGLE) *
        //       (JOINT_3_START_ANGLE * 2.0f))) *
        //     DEGREE_TO_RADIAN_FACTOR;
        scara_arm->set_joints_value[JOINT_3] = JOINT_3_START_ANGLE * DEGREE_TO_RADIAN_FACTOR;

        if (fabsf(scara_arm->joints_value[JOINT_3] - JOINT_3_START_ANGLE * DEGREE_TO_RADIAN_FACTOR) < 0.02f)
        {
            setJointStartUpStateOk(JOINT_3, scara_arm->start_up_status);
        }
    }

    /* 关节4 向固定方向运动通过机械限位触发角度获取后运动到起始位置 */
    // setJointStartUpStateOk(JOINT_4, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    // setJointStartUpStateOk(JOINT_4 + JOINT45_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status);
    if (checkIfJointNotStartUp(JOINT_4, scara_arm->start_up_status))
    {
        if (checkIfJointNotStartUp(JOINT_4 + JOINT45_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status))
        {
            scara_arm->set_joints_value[JOINT_4] -= JOINT_4_HOMING_STEP_ANGLE;

            if (rflMotorGetTorque(&scara_arm->joints_motors[MOTOR_JOINT4]) < -JOINT_4_HOMING_TORQUE_THRESHOLD &&
                fabsf(rflMotorGetSpeed(&scara_arm->joints_motors[MOTOR_JOINT4])) < 0.2f)
            {
                rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT4], RFL_ANGLE_FORMAT_DEGREE,
                                   JOINT_4_HOMING_ANGLE, false);
                setJointStartUpStateOk(JOINT_4 + JOINT45_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status);
            }
        }
        else
        {
            scara_arm->set_joints_value[JOINT_4] = JOINT_4_START_ANGLE * DEGREE_TO_RADIAN_FACTOR;

            if (fabsf(scara_arm->joints_value[JOINT_4] - JOINT_4_START_ANGLE * DEGREE_TO_RADIAN_FACTOR) < 0.02f)
            {
                setJointStartUpStateOk(JOINT_4, scara_arm->start_up_status);
            }
        }
    }

    // 也许roll归中可以不考虑了

    /* 关节6 通过磁编获取角度归中后运动到起始位置 */
    setJointStartUpStateOk(JOINT_6, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    // if (checkIfJointNotStartUp(JOINT_6, scara_arm->start_up_status))
    // {
    //     rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_DEGREE,
    //                        /* JOINT_5_HOMING_ANGLE - */
    //                        (scara_arm->joint_6_encoder_angle * END_BEVEL_GEAR_SET_REDUCTION), true);
    //     rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_ANGLE_FORMAT_DEGREE,
    //                        /* JOINT_5_HOMING_ANGLE + */
    //                        (scara_arm->joint_6_encoder_angle * END_BEVEL_GEAR_SET_REDUCTION), true);

    //     rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
    //     rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
    //     rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_DEGREE,
    //     JOINT_6_START_ANGLE); rflMotorSetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT],
    //     RFL_ANGLE_FORMAT_DEGREE, JOINT_6_START_ANGLE);

    //     if ((fabsf(rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_DEGREE) -
    //                JOINT_6_START_ANGLE) < 1.0f) &&
    //         (fabsf(rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_ANGLE_FORMAT_DEGREE) -
    //                JOINT_6_START_ANGLE) < 1.0f))
    //     {
    //         setJointStartUpStateOk(JOINT_6, scara_arm->start_up_status);
    //         rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_MOTOR_CONTROL_MODE_NO_FORCE);
    //         rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_MOTOR_CONTROL_MODE_NO_FORCE);
    //     }
    // }

    /* 关节5 当关节6启动完毕后 向固定方向运动通过机械限位触发角度获取后运动到起始位置 */
    // setJointStartUpStateOk(JOINT_5, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    // setJointStartUpStateOk(JOINT_5 + JOINT45_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status);
    if (/* !checkIfJointNotStartUp(JOINT_6, scara_arm->start_up_status) &&
         */
        checkIfJointNotStartUp(JOINT_5, scara_arm->start_up_status))
    {
        if (checkIfJointNotStartUp(JOINT_5 + JOINT45_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status))
        {
            scara_arm->set_joints_value[JOINT_5] -= JOINT_5_HOMING_STEP_ANGLE;

            if ((rflMotorGetTorque(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT]) < -JOINT_5_HOMING_TORQUE_THRESHOLD ||
                 rflMotorGetTorque(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT]) <
                     -JOINT_5_HOMING_TORQUE_THRESHOLD) &&
                ((fabsf(rflMotorGetSpeed(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT])) < 0.1f) ||
                 (fabsf(rflMotorGetSpeed(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT])) < 0.1f)))
            {
                rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_DEGREE,
                                   JOINT_5_HOMING_ANGLE, false);
                rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_ANGLE_FORMAT_DEGREE,
                                   JOINT_5_HOMING_ANGLE, false);
                setJointStartUpStateOk(JOINT_5 + JOINT45_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status);
            }
        }
        else
        {
            scara_arm->set_joints_value[JOINT_5] = JOINT_5_START_ANGLE * DEGREE_TO_RADIAN_FACTOR;

            if (fabsf(scara_arm->joints_value[JOINT_5] - JOINT_5_START_ANGLE * DEGREE_TO_RADIAN_FACTOR) < 0.04f)
            {
                setJointStartUpStateOk(JOINT_5, scara_arm->start_up_status);
            }
        }
    }

    scara_arm->reset_success = (scara_arm->start_up_status == ARM_START_UP_OK);

    for (uint8_t i = 0; i < 6; i++)
    {
        scara_arm->set_pose_6d[i] = scara_arm->pose_6d[i];
    }
}

#if USE_JOINTS_CONTROL
static void joints_control(engineer_scara_arm_s *scara_arm)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        scara_arm->set_pose_6d[i] = scara_arm->pose_6d[i];
    }

    scara_arm->set_joints_value[JOINT_1] +=
        ((float)(rflDeadZoneZero(scara_arm->rc->dt7_dr16_data.rc.ch[3], RC_DT7_ROCKER_DEADLINE)) / 660.0f *
         JOINT_1_CONTROL_SEN);

    if (scara_arm->rc->dt7_dr16_data.rc.s[0] == 2)
        scara_arm->set_joints_value[JOINT_2] +=
            ((float)(rflDeadZoneZero(scara_arm->rc->dt7_dr16_data.rc.ch[2], RC_DT7_ROCKER_DEADLINE)) / 660.0f *
             -JOINT_2_CONTROL_SEN);
    else if (scara_arm->rc->dt7_dr16_data.rc.s[0] == 3)
        scara_arm->set_joints_value[JOINT_3] +=
            ((float)(rflDeadZoneZero(scara_arm->rc->dt7_dr16_data.rc.ch[2], RC_DT7_ROCKER_DEADLINE)) / 660.0f *
             -JOINT_3_CONTROL_SEN);
    else if (scara_arm->rc->dt7_dr16_data.rc.s[0] == 1)
        scara_arm->set_joints_value[JOINT_4] +=
            ((float)(rflDeadZoneZero(scara_arm->rc->dt7_dr16_data.rc.ch[2], RC_DT7_ROCKER_DEADLINE)) / 660.0f *
             -JOINT_4_CONTROL_SEN);

    scara_arm->set_joints_value[JOINT_5] +=
        ((float)(rflDeadZoneZero(scara_arm->rc->dt7_dr16_data.rc.ch[1], RC_DT7_ROCKER_DEADLINE)) / 660.0f *
         -JOINT_5_CONTROL_SEN);

    scara_arm->set_joints_value[JOINT_6] +=
        ((float)(rflDeadZoneZero(scara_arm->rc->dt7_dr16_data.rc.ch[0], RC_DT7_ROCKER_DEADLINE)) / 660.0f *
         JOINT_6_CONTROL_SEN);
}
#endif

static void pose_control(engineer_scara_arm_s *scara_arm)
{
    // DT7控制
    scara_arm->set_pose_6d[0] +=
        ((float)(rflDeadZoneZero(scara_arm->rc->dt7_dr16_data.rc.ch[3], RC_DT7_ROCKER_DEADLINE)) / 660.0f *
         POSE_X_CONTROL_SEN);

    scara_arm->set_pose_6d[1] +=
        ((float)(rflDeadZoneZero(scara_arm->rc->dt7_dr16_data.rc.ch[2], RC_DT7_ROCKER_DEADLINE)) / 660.0f *
         -POSE_Y_CONTROL_SEN);

    scara_arm->set_pose_6d[2] +=
        ((float)(rflDeadZoneZero(scara_arm->rc->dt7_dr16_data.rc.ch[1], RC_DT7_ROCKER_DEADLINE)) / 660.0f *
         POSE_Z_CONTROL_SEN);

    if (scara_arm->rc->dt7_dr16_data.rc.s[0] == 1)
        scara_arm->set_pose_6d[3] +=
            ((float)(rflDeadZoneZero(scara_arm->rc->dt7_dr16_data.rc.ch[0], RC_DT7_ROCKER_DEADLINE)) / 660.0f *
             -POSE_AY_CONTROL_SEN);
    else if (scara_arm->rc->dt7_dr16_data.rc.s[0] == 3)
        scara_arm->set_pose_6d[4] +=
            ((float)(rflDeadZoneZero(scara_arm->rc->dt7_dr16_data.rc.ch[0], RC_DT7_ROCKER_DEADLINE)) / 660.0f *
             -POSE_AP_CONTROL_SEN);
    else if (scara_arm->rc->dt7_dr16_data.rc.s[0] == 2)
        scara_arm->set_pose_6d[5] +=
            ((float)(rflDeadZoneZero(scara_arm->rc->dt7_dr16_data.rc.ch[0], RC_DT7_ROCKER_DEADLINE)) / 660.0f *
             POSE_AR_CONTROL_SEN);

    // 自定义控制器控制
    if (checkIfCustomerControllerKeyPressed(scara_arm->customer_controller->key, CC_ROUGHLY))
    {
        for (uint8_t i = 0; i < 6; i++)
        {
            if (i < 3)
                scara_arm->set_pose_6d[i] =
                    scara_arm->local_pos_memory[i] + (scara_arm->cc_pose_6d[i] - scara_arm->cc_pos_memory[i]);
            else
                scara_arm->set_pose_6d[i] = scara_arm->cc_pose_6d[i];
        }
    }
}

static void move_homing_control(engineer_scara_arm_s *scara_arm)
{
    scara_arm->set_pose_6d[0] = 0.122f;
    scara_arm->set_pose_6d[1] = -0.267f;
    scara_arm->set_pose_6d[2] = 0.0f;
    scara_arm->set_pose_6d[3] = -90.0f * DEGREE_TO_RADIAN_FACTOR;
    scara_arm->set_pose_6d[4] = 0.0f;
    scara_arm->set_pose_6d[5] = 0.0f;

    if (fabsf(scara_arm->pose_6d[0] - scara_arm->set_pose_6d[0]) < TOLERABLE_DISTANCE_DEVIATION &&
        fabsf(scara_arm->pose_6d[1] - scara_arm->set_pose_6d[1]) < TOLERABLE_DISTANCE_DEVIATION &&
        fabsf(scara_arm->pose_6d[2] - scara_arm->set_pose_6d[2]) < TOLERABLE_DISTANCE_DEVIATION &&
        fabsf(scara_arm->pose_6d[3] - scara_arm->set_pose_6d[3]) < TOLERABLE_ANGLE_DEVIATION &&
        fabsf(scara_arm->pose_6d[4] - scara_arm->set_pose_6d[4]) < TOLERABLE_ANGLE_DEVIATION &&
        fabsf(scara_arm->pose_6d[5] - scara_arm->set_pose_6d[5]) < TOLERABLE_ANGLE_DEVIATION)
        scara_arm->move_homing_success = true;
}

static void operation_homing_control(engineer_scara_arm_s *scara_arm)
{
    scara_arm->set_pose_6d[0] = 0.45f;
    scara_arm->set_pose_6d[1] = 0.0f;
    scara_arm->set_pose_6d[2] = OPERATION_MODE_DEFAULT_Z;
    scara_arm->set_pose_6d[3] = 0.0f;
    scara_arm->set_pose_6d[4] = 0.0f;
    scara_arm->set_pose_6d[5] = 0.0f;

    if (fabsf(scara_arm->pose_6d[0] - scara_arm->set_pose_6d[0]) < TOLERABLE_DISTANCE_DEVIATION &&
        fabsf(scara_arm->pose_6d[1] - scara_arm->set_pose_6d[1]) < TOLERABLE_DISTANCE_DEVIATION &&
        fabsf(scara_arm->pose_6d[2] - scara_arm->set_pose_6d[2]) < TOLERABLE_DISTANCE_DEVIATION &&
        fabsf(scara_arm->pose_6d[3] - scara_arm->set_pose_6d[3]) < TOLERABLE_ANGLE_DEVIATION &&
        fabsf(scara_arm->pose_6d[4] - scara_arm->set_pose_6d[4]) < TOLERABLE_ANGLE_DEVIATION &&
        fabsf(scara_arm->pose_6d[5] - scara_arm->set_pose_6d[5]) < TOLERABLE_ANGLE_DEVIATION)
        scara_arm->operation_homing_success = true;
}

/**
 * @brief 取银矿控制
 * @note 分为两种情况，即完整的自动取矿，和已经手动取矿后的上提并收回
 *
 * @param scara_arm
 */
static void silver_mining_control(engineer_scara_arm_s *scara_arm)
{
    if (scara_arm->silver_mining_success == true)
        return;

    if (scara_arm->silver_mining_step == SILVER_MINING_STEP_INIT) // 对位
    {
        scara_arm->silver_mining_grab_detect_timer = 0;
        scara_arm->silver_mining_grab_end_timer = 0;

        // 清除历史误操作
        if (checkIfRcKeyFallingEdgeDetected(RC_LEFT))
            ;
        if (checkIfRcKeyFallingEdgeDetected(RC_RIGHT))
            ;

        scara_arm->set_pose_6d[0] = 0.45f;
        scara_arm->set_pose_6d[1] = 0.0f;
        scara_arm->set_pose_6d[2] = 0.45f;
        scara_arm->set_pose_6d[4] = 0.0f;
        for (uint8_t i = 0; i < 6; i++)
        {
            scara_arm->silver_mining_pose_memory[i] = scara_arm->set_pose_6d[i];
        }

        // 若已经取到矿，则直接上提
        if (scara_arm->grabbed)
        {
            scara_arm->set_pose_6d[2] += 0.15f;
            scara_arm->silver_mining_step = SILVER_MINING_STEP_WAIT;
        }

        // 普通完整流程
        if (fabsf(scara_arm->pose_6d[0] - scara_arm->set_pose_6d[0]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[1] - scara_arm->set_pose_6d[1]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[2] - scara_arm->set_pose_6d[2]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[4] - scara_arm->set_pose_6d[4]) < TOLERABLE_ANGLE_DEVIATION)
        {
            setArmGrabMode(true);
            scara_arm->set_pose_6d[0] = 0.75f;
            scara_arm->silver_mining_step = SILVER_MINING_STEP_GRAB;
        }
    }
    else if (scara_arm->silver_mining_step == SILVER_MINING_STEP_GRAB)
    {
        // 判断前伸是否因为碰到矿石而无法前进
        if (fabsf(scara_arm->pose_6d[0] - scara_arm->last_pose_6d[0]) < 0.002f)
            scara_arm->silver_mining_grab_detect_timer++;
        else
            scara_arm->silver_mining_grab_detect_timer = 0;

        scara_arm->silver_mining_grab_end_timer++;

        if (scara_arm->silver_mining_grab_detect_timer >= 200 || scara_arm->silver_mining_grab_end_timer >= 2500)
        {
            scara_arm->silver_mining_grab_detect_timer = 0;
            scara_arm->silver_mining_grab_end_timer = 0;
            scara_arm->set_pose_6d[0] = scara_arm->pose_6d[0] - 0.05f;
            scara_arm->set_pose_6d[2] = scara_arm->pose_6d[0] + 0.15f;
            scara_arm->set_pose_6d[4] = scara_arm->pose_6d[4] + 10.0f * DEGREE_TO_RADIAN_FACTOR;
            scara_arm->silver_mining_step = SILVER_MINING_STEP_WAIT;
        }
    }
    else if (scara_arm->silver_mining_step == SILVER_MINING_STEP_WAIT)
    {
        if (checkIfRcKeyFallingEdgeDetected(RC_LEFT) &&
            fabsf(scara_arm->pose_6d[2] - scara_arm->set_pose_6d[2]) < TOLERABLE_DISTANCE_DEVIATION)
        {
            scara_arm->silver_mining_step = SILVER_MINING_STEP_OK;
        }
        if (checkIfRcKeyFallingEdgeDetected(RC_RIGHT) &&
            fabsf(scara_arm->pose_6d[2] - scara_arm->set_pose_6d[2]) < TOLERABLE_DISTANCE_DEVIATION)
        {
            for (uint8_t i = 0; i < 6; i++)
            {
                scara_arm->set_pose_6d[i] = scara_arm->silver_mining_pose_memory[i];
            }
            scara_arm->silver_mining_step = SILVER_MINING_STEP_READY;
        }
    }
    else if (scara_arm->silver_mining_step == SILVER_MINING_STEP_READY)
    {
        float y_sign = 0.0f;
        float z_sign = 0.0f;
        float yaw_sign = 0.0f;
        float roll_sign = 0.0f;

        if (checkIsRcKeyPressed(RC_A) && !checkIsRcKeyPressed(RC_D))
            y_sign = 1.0f;
        else if (!checkIsRcKeyPressed(RC_A) && checkIsRcKeyPressed(RC_D))
            y_sign = -1.0f;
        else
            y_sign = 0.0f;

        if (checkIsRcKeyPressed(RC_W) && !checkIsRcKeyPressed(RC_S))
            z_sign = 1.0f;
        else if (!checkIsRcKeyPressed(RC_W) && checkIsRcKeyPressed(RC_S))
            z_sign = -1.0f;
        else
            z_sign = 0.0f;

        if (checkIsRcKeyPressed(RC_Q) && !checkIsRcKeyPressed(RC_E))
            yaw_sign = 1.0f;
        else if (!checkIsRcKeyPressed(RC_Q) && checkIsRcKeyPressed(RC_E))
            yaw_sign = -1.0f;
        else
            yaw_sign = 0.0f;

        if (checkIsRcKeyPressed(RC_Z) && !checkIsRcKeyPressed(RC_X))
            roll_sign = 1.0f;
        else if (!checkIsRcKeyPressed(RC_Z) && checkIsRcKeyPressed(RC_X))
            roll_sign = -1.0f;
        else
            roll_sign = 0.0f;

        scara_arm->set_pose_6d[1] += y_sign * SILVER_MINING_Y_CONTROL_SEN;
        scara_arm->set_pose_6d[2] += z_sign * SILVER_MINING_Z_CONTROL_SEN;
        scara_arm->set_pose_6d[3] += yaw_sign * SILVER_MINING_AY_CONTROL_SEN;
        scara_arm->set_pose_6d[5] += roll_sign * SILVER_MINING_AR_CONTROL_SEN;
        for (uint8_t i = 0; i < 6; i++)
        {
            scara_arm->silver_mining_pose_memory[i] = scara_arm->set_pose_6d[i];
        }

        if (checkIfRcKeyFallingEdgeDetected(RC_LEFT))
        {
            scara_arm->set_pose_6d[0] = 0.75f;
            scara_arm->silver_mining_step = SILVER_MINING_STEP_GRAB;
        }
    }
    else if (scara_arm->silver_mining_step == SILVER_MINING_STEP_OK)
    {
        scara_arm->set_pose_6d[0] = 0.4f;
        scara_arm->set_pose_6d[1] = 0.0f;

        if (fabsf(scara_arm->pose_6d[0] - scara_arm->set_pose_6d[0]) < TOLERABLE_DISTANCE_DEVIATION)
        {
            scara_arm->set_pose_6d[2] = 0.05f;
            if (fabsf(scara_arm->pose_6d[2] - scara_arm->set_pose_6d[2]) < TOLERABLE_DISTANCE_DEVIATION)
                scara_arm->silver_mining_success = true;

            scara_arm->silver_mining_step = SILVER_MINING_STEP_INIT;
        }
    }
}

static void gold_mining_control(engineer_scara_arm_s *scara_arm)
{
    if (scara_arm->gold_mining_success == true)
        return;

    scara_arm->gold_mining_success = true;
}

static void storage_push_control(engineer_scara_arm_s *scara_arm)
{
    if (scara_arm->storage_push_success == true)
        return;

    if (scara_arm->storage_push_step == STORAGE_PUSH_STEP_INIT)
    {
        if (getStoragePushInAvailableSlot() >= STORAGE_NULL)
        {
            StorageCancelPushIn();
            scara_arm->storage_push_success = true;
        }

        scara_arm->solution = JOINT_3_ON_THE_LEFT;

        scara_arm->storage_push_step = STORAGE_PUSH_STEP_START;
    }
    else if (scara_arm->storage_push_step == STORAGE_PUSH_STEP_START)
    {
        scara_arm->set_pose_6d[0] = 0.15f;
        scara_arm->set_pose_6d[1] = -0.4f;
        scara_arm->set_pose_6d[2] = 0.04f;
        scara_arm->set_pose_6d[3] = -135.0f * DEGREE_TO_RADIAN_FACTOR;
        scara_arm->set_pose_6d[4] = 12.5f * DEGREE_TO_RADIAN_FACTOR;

        if (fabsf(scara_arm->pose_6d[0] - scara_arm->set_pose_6d[0]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[1] - scara_arm->set_pose_6d[1]) < TOLERABLE_DISTANCE_DEVIATION)
            scara_arm->storage_push_step = STORAGE_PUSH_STEP_SLOT;
    }
    else if (scara_arm->storage_push_step == STORAGE_PUSH_STEP_SLOT)
    {
        scara_arm->set_pose_6d[1] = -0.4f;
        scara_arm->set_pose_6d[2] = 0.04f;
        scara_arm->set_pose_6d[3] = -182.0f * DEGREE_TO_RADIAN_FACTOR;
        scara_arm->set_pose_6d[4] = 12.5f * DEGREE_TO_RADIAN_FACTOR;

        if (getStorageCurrentTargetSlot() == STORAGE_BACK)
            scara_arm->set_pose_6d[0] = -0.355f;
        else if (getStorageCurrentTargetSlot() == STORAGE_FRONT)
            scara_arm->set_pose_6d[0] = -0.02f;

        if (fabsf(scara_arm->pose_6d[0] - scara_arm->set_pose_6d[0]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[1] - scara_arm->set_pose_6d[1]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[2] - scara_arm->set_pose_6d[2]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[3] - scara_arm->set_pose_6d[3]) < TOLERABLE_ANGLE_DEVIATION &&
            fabsf(scara_arm->pose_6d[4] - scara_arm->set_pose_6d[4]) < TOLERABLE_ANGLE_DEVIATION)
            scara_arm->storage_push_step = STORAGE_PUSH_STEP_PUSH_IN;
    }
    else if (scara_arm->storage_push_step == STORAGE_PUSH_STEP_PUSH_IN)
    {
        if (getStorageCurrentTargetSlot() == STORAGE_BACK)
        {
            scara_arm->set_pose_6d[0] = -0.355f;

            scara_arm->set_pose_6d[1] += 0.0005f;
        }
        else if (getStorageCurrentTargetSlot() == STORAGE_FRONT)
        {
            scara_arm->set_pose_6d[0] = -0.025f;

            scara_arm->set_pose_6d[1] += 0.0003f;
        }

        if (scara_arm->set_pose_6d[1] > -0.27)
            scara_arm->set_pose_6d[1] = -0.27;
    }

    if (getStorageSlotStatus(getStorageCurrentTargetSlot()) == STORAGE_SLOT_USED)
    {
        setArmGrabMode(false);
        scara_arm->set_pose_6d[0] = scara_arm->pose_6d[0] + 0.1f;
        scara_arm->set_pose_6d[1] = scara_arm->pose_6d[1] - 0.1f;
        scara_arm->storage_push_success = true;

        scara_arm->storage_push_step = STORAGE_PUSH_STEP_INIT;
    }
}

static void storage_pop_control(engineer_scara_arm_s *scara_arm)
{
    if (scara_arm->storage_pop_success == true)
        return;

    if (scara_arm->storage_pop_step == STORAGE_POP_STEP_INIT)
    {
        if (getStoragePopOutAvailableSlot() >= STORAGE_NULL)
        {
            scara_arm->storage_pop_success = true;
        }

        scara_arm->solution = JOINT_3_ON_THE_LEFT;

        scara_arm->storage_pop_step = STORAGE_POP_STEP_START;
    }
    else if (scara_arm->storage_pop_step == STORAGE_POP_STEP_START)
    {
        scara_arm->set_pose_6d[0] = 0.06f;
        scara_arm->set_pose_6d[1] = -0.3f;
        scara_arm->set_pose_6d[2] = 0.0f;
        scara_arm->set_pose_6d[3] = -185.0f * DEGREE_TO_RADIAN_FACTOR;
        scara_arm->set_pose_6d[4] = 0.0f;

        if (fabsf(scara_arm->pose_6d[1] - scara_arm->set_pose_6d[1]) < TOLERABLE_DISTANCE_DEVIATION)
            scara_arm->storage_pop_step = STORAGE_POP_STEP_SLOT;
    }
    else if (scara_arm->storage_pop_step == STORAGE_POP_STEP_SLOT)
    {
        scara_arm->set_pose_6d[1] = -0.3f;
        scara_arm->set_pose_6d[2] = 0.0f;
        scara_arm->set_pose_6d[3] = -185.0f * DEGREE_TO_RADIAN_FACTOR;
        scara_arm->set_pose_6d[4] = 0.0f;

        if (getStorageCurrentTargetSlot() == STORAGE_BACK)
            scara_arm->set_pose_6d[0] = -0.315f;
        else if (getStorageCurrentTargetSlot() == STORAGE_FRONT)
            scara_arm->set_pose_6d[0] = 0.015f;

        if (fabsf(scara_arm->pose_6d[0] - scara_arm->set_pose_6d[0]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[1] - scara_arm->set_pose_6d[1]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[2] - scara_arm->set_pose_6d[2]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[3] - scara_arm->set_pose_6d[3]) < TOLERABLE_ANGLE_DEVIATION &&
            fabsf(scara_arm->pose_6d[4] - scara_arm->set_pose_6d[4]) < TOLERABLE_ANGLE_DEVIATION)
        {
            setArmGrabMode(true);
            scara_arm->storage_pop_step = STORAGE_POP_STEP_POP_OUT;
        }
    }
    else if (scara_arm->storage_pop_step == STORAGE_POP_STEP_POP_OUT)
    {
        scara_arm->set_pose_6d[0] -= 0.00025f;
        if (getStorageCurrentTargetSlot() == STORAGE_BACK)
        {
            if (scara_arm->set_pose_6d[0] < -0.4f)
                scara_arm->set_pose_6d[0] = -0.4f;
        }
        else if (getStorageCurrentTargetSlot() == STORAGE_FRONT)
        {
            if (scara_arm->set_pose_6d[0] < -0.07f)
                scara_arm->set_pose_6d[0] = -0.07f;
        }

        scara_arm->set_pose_6d[1] = -0.3f;
    }

    if (checkIfArmGrabbed())
    {
        StorageConfirmPopOut();
        scara_arm->set_pose_6d[0] = scara_arm->pose_6d[0] + 0.1f;
        scara_arm->set_pose_6d[1] = scara_arm->pose_6d[1] - 0.1f;
        scara_arm->storage_pop_success = true;

        scara_arm->storage_pop_step = STORAGE_POP_STEP_INIT;
    }
}
