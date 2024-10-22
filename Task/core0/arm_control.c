#include "math.h"

#include "arm_control.h"

#include "board.h"

#include "arm_motor.h"

#include "algo_data_limiting.h"

#include "remote_control.h"

#include "detect_task.h"
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
static void emergency_move_control(engineer_scara_arm_s *scara_arm);

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
        arm_motor_set_mode(scara_arm);

        // if (scara_arm->behavior == ENGINEER_BEHAVIOR_DISABLE)
        //     setArmGrabMode(false);

        if (scara_arm->behavior == ENGINEER_BEHAVIOR_RESET && !scara_arm->grabbed)
        {
            resetArmStartUpStatus(scara_arm->start_up_status);
            scara_arm->joint_6_homing_timer = 0;
            arm_motor_set_angle_limit(scara_arm, true);
        }
        else if (getEngineerLastBehavior() == ENGINEER_BEHAVIOR_RESET && scara_arm->behavior != ENGINEER_BEHAVIOR_RESET)
        {
            arm_motor_set_angle_limit(scara_arm, false);
        }

        if (scara_arm->behavior == ENGINEER_BEHAVIOR_AUTO_SILVER_MINING)
        {
            scara_arm->silver_mining_step = SILVER_MINING_STEP_INIT;
        }
        if (scara_arm->behavior == ENGINEER_BEHAVIOR_AUTO_GOLD_MINING)
        {
            scara_arm->gold_mining_step = GOLD_MINING_STEP_INIT;
        }

        if (scara_arm->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH)
        {
            scara_arm->storage_push_step = STORAGE_PUSH_STEP_INIT;
        }
        if (scara_arm->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_POP)
        {
            scara_arm->storage_pop_step = STORAGE_POP_STEP_INIT;
        }

        if (checkIfIsEmergencyMoving())
        {
            rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
            rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
            arm_motor_set_angle_limit(scara_arm, true);
            rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], RFL_ANGLE_FORMAT_DEGREE, 0.0f, false);
            rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], RFL_ANGLE_FORMAT_DEGREE, 0.0f, false);
            scara_arm->set_joints_value[JOINT_1] = 0.0f;
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
        // 若需紧急机动，有力抬升并允许对抬升层进行控制操作
        if (checkIfIsEmergencyMoving())
            emergency_move_control(scara_arm);
        break;
    }
}

static void control_value_process(engineer_scara_arm_s *scara_arm)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        if (i < 3)
            rflFirstOrderFilterCali(&scara_arm->cc_pose_filter[i], getCcPose(i));
        else
            rflFirstOrderFilterCali(&scara_arm->cc_pose_filter[i],
                                    rflFloatLoopConstrain(getCcPose(i), -RAD_PI, RAD_PI));

        scara_arm->cc_pose_6d[i] = scara_arm->cc_pose_filter[i].out;
    }

    if (!checkIsCcKeyPressed(CC_TRIGGER))
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
    /* 关节1 */
    // setJointStartUpStateOk(JOINT_1, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    if (checkIfJointNotStartUp(JOINT_1, scara_arm->start_up_status))
    {
        scara_arm->set_joints_value[JOINT_1] -= JOINT_1_HOMING_STEP_DISTANCE;

        if (rflMotorGetTorque(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT]) < -JOINT_1_HOMING_TORQUE_THRESHOLD &&
            rflMotorGetTorque(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT]) < -JOINT_1_HOMING_TORQUE_THRESHOLD &&
            (fabsf(rflMotorGetSpeed(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT])) < 0.1f) &&
            (fabsf(rflMotorGetSpeed(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT])) < 0.1f))
        {
            rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT1_LEFT], RFL_ANGLE_FORMAT_DEGREE,
                               JOINT_1_HOMING_ANGLE, false);
            rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT1_RIGHT], RFL_ANGLE_FORMAT_DEGREE,
                               JOINT_1_HOMING_ANGLE, false);

            scara_arm->set_joints_value[JOINT_1] = 0.0f;

            setJointStartUpStateOk(JOINT_1, scara_arm->start_up_status);
        }
    }

    /* 关节2 运动到起始位置 */
    // setJointStartUpStateOk(JOINT_2, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    if (checkIfJointNotStartUp(JOINT_2, scara_arm->start_up_status) && !detect_error(ARM_JOINT_2_DH) &&
        !detect_error(ARM_JOINT_3_DH))
    {
        if (rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT23_FRONT], RFL_ANGLE_FORMAT_RADIAN) *
                RADIAN_TO_DEGREE_FACTOR <
            -160.0f)
        {
            scara_arm->set_joints_value[JOINT_2] = scara_arm->joints_value[JOINT_2];
        }
        else if (checkIfJointNotStartUp(JOINT_4, scara_arm->start_up_status))
        {
            scara_arm->set_joints_value[JOINT_2] = JOINT_2_START_WAIT_ANGLE * DEGREE_TO_RADIAN_FACTOR;
        }
        else
        {
            scara_arm->set_joints_value[JOINT_2] = JOINT_2_START_ANGLE * DEGREE_TO_RADIAN_FACTOR;

            if (fabsf(scara_arm->joints_value[JOINT_2] - JOINT_2_START_ANGLE * DEGREE_TO_RADIAN_FACTOR) <
                TOLERABLE_ANGLE_DEVIATION)
            {
                setJointStartUpStateOk(JOINT_2, scara_arm->start_up_status);
            }
        }
    }

    /* 关节3 运动到起始位置 */
    // setJointStartUpStateOk(JOINT_3, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    setJointStartUpStateOk(JOINT_3 + JOINTS_START_UP_STEP1_BIT_OFFSET,
                           scara_arm->start_up_status); // 以前有用现在没用了 懒得删了
    if (checkIfJointNotStartUp(JOINT_3, scara_arm->start_up_status) && !detect_error(ARM_JOINT_2_DH) &&
        !detect_error(ARM_JOINT_3_DH))
    {
        if (rflMotorGetAngle(&scara_arm->joints_motors[MOTOR_JOINT23_FRONT], RFL_ANGLE_FORMAT_RADIAN) *
                RADIAN_TO_DEGREE_FACTOR <
            -160.0f)
            scara_arm->set_joints_value[JOINT_3] = scara_arm->joints_value[JOINT_3] + 0.2f;
        else if (checkIfJointNotStartUp(JOINT_4, scara_arm->start_up_status))
            scara_arm->set_joints_value[JOINT_3] = JOINT_3_START_WAIT_ANGLE * DEGREE_TO_RADIAN_FACTOR;
        else
        {
            scara_arm->set_joints_value[JOINT_3] = JOINT_3_START_ANGLE * DEGREE_TO_RADIAN_FACTOR;

            if (fabsf(scara_arm->joints_value[JOINT_3] - JOINT_3_START_ANGLE * DEGREE_TO_RADIAN_FACTOR) <
                TOLERABLE_ANGLE_DEVIATION)
            {
                setJointStartUpStateOk(JOINT_3, scara_arm->start_up_status);
            }
        }
    }

    /* 关节4 向固定方向运动通过机械限位触发角度获取后运动到起始位置 */
    // setJointStartUpStateOk(JOINT_4, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    // setJointStartUpStateOk(JOINT_4 + JOINTS_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status);
    if (checkIfJointNotStartUp(JOINT_4, scara_arm->start_up_status))
    {
        if (checkIfJointNotStartUp(JOINT_4 + JOINTS_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status))
        {
            scara_arm->set_joints_value[JOINT_4] += JOINT_4_HOMING_STEP_ANGLE;

            if (rflMotorGetTorque(&scara_arm->joints_motors[MOTOR_JOINT4]) > JOINT_4_HOMING_TORQUE_THRESHOLD &&
                fabsf(rflMotorGetSpeed(&scara_arm->joints_motors[MOTOR_JOINT4])) < 0.2f &&
                fabsf(scara_arm->joints_value[JOINT_4] - scara_arm->set_joints_value[JOINT_4]) >
                    JOINT_4_HOMING_ANGLE_THRESHOLD)
            {
                rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT4], RFL_ANGLE_FORMAT_DEGREE,
                                   JOINT_4_HOMING_ANGLE, false);
                setJointStartUpStateOk(JOINT_4 + JOINTS_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status);
            }
        }
        else
        {
            scara_arm->set_joints_value[JOINT_4] = JOINT_4_START_ANGLE * DEGREE_TO_RADIAN_FACTOR;

            if (fabsf(scara_arm->joints_value[JOINT_4] - JOINT_4_START_ANGLE * DEGREE_TO_RADIAN_FACTOR) <
                TOLERABLE_ANGLE_DEVIATION)
            {
                setJointStartUpStateOk(JOINT_4, scara_arm->start_up_status);
            }
        }
    }

    /* 关节6 通过磁编获取角度归中后运动到起始位置 */
    // setJointStartUpStateOk(JOINT_6, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    if (checkIfJointNotStartUp(JOINT_6, scara_arm->start_up_status))
    {
        scara_arm->set_joints_value[JOINT_6] += JOINT_6_HOMING_STEP_ANGLE;

        scara_arm->joint_6_homing_timer++;

        if ((fabsf(scara_arm->joint_6_encoder_angle - JOINT_6_START_DETECT_ANGLE_0) < 2.4f) ||
            (fabsf(scara_arm->joint_6_encoder_angle - JOINT_6_START_DETECT_ANGLE_1) < 2.4f) ||
            scara_arm->joint_6_homing_timer > 1000)
        {
            rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_LEFT], RFL_ANGLE_FORMAT_DEGREE, 0.0f, true);
            rflMotorResetAngle(&scara_arm->joints_motors[MOTOR_JOINT56_RIGHT], RFL_ANGLE_FORMAT_DEGREE, 0.0f, true);

            scara_arm->set_joints_value[JOINT_6] = 0.0f;

            setJointStartUpStateOk(JOINT_6, scara_arm->start_up_status);
        }
    }

    /* 关节5 当关节6启动完毕后 向固定方向运动通过机械限位触发角度获取后运动到起始位置 */
    // setJointStartUpStateOk(JOINT_5, scara_arm->start_up_status); // 只是测试时用来关掉这个关节的归中
    // setJointStartUpStateOk(JOINT_5 + JOINTS_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status);
    if (!checkIfJointNotStartUp(JOINT_6, scara_arm->start_up_status) &&
        checkIfJointNotStartUp(JOINT_5, scara_arm->start_up_status))
    {
        if (checkIfJointNotStartUp(JOINT_5 + JOINTS_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status))
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
                setJointStartUpStateOk(JOINT_5 + JOINTS_START_UP_STEP1_BIT_OFFSET, scara_arm->start_up_status);
            }
        }
        else
        {
            scara_arm->set_joints_value[JOINT_5] = JOINT_5_START_ANGLE * DEGREE_TO_RADIAN_FACTOR;

            if (fabsf(scara_arm->joints_value[JOINT_5] - JOINT_5_START_ANGLE * DEGREE_TO_RADIAN_FACTOR) <
                TOLERABLE_ANGLE_DEVIATION)
            {
                setJointStartUpStateOk(JOINT_5, scara_arm->start_up_status);
            }
        }
    }

    scara_arm->reset_success = (scara_arm->start_up_status == ARM_START_UP_OK);
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
        ((float)(rflDeadZoneZero(getDt7RockerPosition(DT7_ROCKER_LEFT_VERTICAL), RC_DT7_ROCKER_DEADLINE)) / 660.0f *
         POSE_X_CONTROL_SEN);

    scara_arm->set_pose_6d[1] +=
        ((float)(rflDeadZoneZero(getDt7RockerPosition(DT7_ROCKER_LEFT_HORIZONTAL), RC_DT7_ROCKER_DEADLINE)) / 660.0f *
         -POSE_Y_CONTROL_SEN);

    scara_arm->set_pose_6d[2] +=
        ((float)(rflDeadZoneZero(getDt7RockerPosition(DT7_ROCKER_RIGHT_VERTICAL), RC_DT7_ROCKER_DEADLINE)) / 660.0f *
         POSE_Z_CONTROL_SEN);

    if (getDt7ToggleSwitchPosition(DT7_SWITCH_RIGHT) == DT7_SWITCH_UP)
        scara_arm->set_pose_6d[3] +=
            ((float)(rflDeadZoneZero(getDt7RockerPosition(DT7_ROCKER_RIGHT_HORIZONTAL), RC_DT7_ROCKER_DEADLINE)) /
             660.0f * -POSE_AY_CONTROL_SEN);
    else if (getDt7ToggleSwitchPosition(DT7_SWITCH_RIGHT) == DT7_SWITCH_MID)
        scara_arm->set_pose_6d[4] +=
            ((float)(rflDeadZoneZero(getDt7RockerPosition(DT7_ROCKER_RIGHT_HORIZONTAL), RC_DT7_ROCKER_DEADLINE)) /
             660.0f * -POSE_AP_CONTROL_SEN);
    else if (getDt7ToggleSwitchPosition(DT7_SWITCH_RIGHT) == DT7_SWITCH_DOWN)
        scara_arm->set_pose_6d[5] +=
            ((float)(rflDeadZoneZero(getDt7RockerPosition(DT7_ROCKER_RIGHT_HORIZONTAL), RC_DT7_ROCKER_DEADLINE)) /
             660.0f * POSE_AR_CONTROL_SEN);

    // 键鼠控制
    scara_arm->set_pose_6d[POSE_Z] += ((float)getRcMouseZ() * 0.0002);

    // 自定义控制器控制
    if (checkIsCcKeyPressed(CC_TRIGGER))
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
    if (scara_arm->grabbed)
    {
        scara_arm->set_pose_6d[POSE_X] = 0.232f;
        scara_arm->set_pose_6d[POSE_Y] = -0.281f;
        scara_arm->set_pose_6d[POSE_Z] = 0.0f;
        scara_arm->set_pose_6d[POSE_YAW] = -90.0f * DEGREE_TO_RADIAN_FACTOR;
        scara_arm->set_pose_6d[POSE_PITCH] = 0.0f;
        scara_arm->set_pose_6d[POSE_ROLL] = 90.0f * DEGREE_TO_RADIAN_FACTOR;
    }
    else
    {
        scara_arm->set_pose_6d[POSE_X] = 0.122f;
        scara_arm->set_pose_6d[POSE_Y] = -0.267f;
        scara_arm->set_pose_6d[POSE_Z] = 0.0f;
        scara_arm->set_pose_6d[POSE_YAW] = -90.0f * DEGREE_TO_RADIAN_FACTOR;
        scara_arm->set_pose_6d[POSE_PITCH] = 0.0f;
        scara_arm->set_pose_6d[POSE_ROLL] = 0.0f;
    }

    if (fabsf(scara_arm->pose_6d[POSE_X] - scara_arm->set_pose_6d[POSE_X]) < TOLERABLE_DISTANCE_DEVIATION &&
        fabsf(scara_arm->pose_6d[POSE_Y] - scara_arm->set_pose_6d[POSE_Y]) < TOLERABLE_DISTANCE_DEVIATION &&
        fabsf(scara_arm->pose_6d[POSE_Z] - scara_arm->set_pose_6d[POSE_Z]) < TOLERABLE_DISTANCE_DEVIATION &&
        fabsf(scara_arm->pose_6d[POSE_YAW] - scara_arm->set_pose_6d[POSE_YAW]) < TOLERABLE_ANGLE_DEVIATION &&
        fabsf(scara_arm->pose_6d[POSE_PITCH] - scara_arm->set_pose_6d[POSE_PITCH]) < TOLERABLE_ANGLE_DEVIATION &&
        fabsf(scara_arm->pose_6d[POSE_ROLL] - scara_arm->set_pose_6d[POSE_ROLL]) < TOLERABLE_ANGLE_DEVIATION)
        scara_arm->move_homing_success = true;
}

static void operation_homing_control(engineer_scara_arm_s *scara_arm)
{
    scara_arm->set_pose_6d[POSE_X] = OPERATION_MODE_DEFAULT_X;
    scara_arm->set_pose_6d[POSE_Y] = OPERATION_MODE_DEFAULT_Y;
    scara_arm->set_pose_6d[POSE_Z] = OPERATION_MODE_DEFAULT_Z;
    scara_arm->set_pose_6d[POSE_YAW] = OPERATION_MODE_DEFAULT_YAW;
    scara_arm->set_pose_6d[POSE_PITCH] = OPERATION_MODE_DEFAULT_PITCH;
    scara_arm->set_pose_6d[POSE_ROLL] = OPERATION_MODE_DEFAULT_ROLL;

    if (fabsf(scara_arm->pose_6d[POSE_X] - scara_arm->set_pose_6d[POSE_X]) < TOLERABLE_DISTANCE_DEVIATION &&
        fabsf(scara_arm->pose_6d[POSE_Y] - scara_arm->set_pose_6d[POSE_Y]) < TOLERABLE_DISTANCE_DEVIATION &&
        fabsf(scara_arm->pose_6d[POSE_Z] - scara_arm->set_pose_6d[POSE_Z]) < TOLERABLE_DISTANCE_DEVIATION &&
        fabsf(scara_arm->pose_6d[POSE_YAW] - scara_arm->set_pose_6d[POSE_YAW]) < TOLERABLE_ANGLE_DEVIATION &&
        fabsf(scara_arm->pose_6d[POSE_PITCH] - scara_arm->set_pose_6d[POSE_PITCH]) < TOLERABLE_ANGLE_DEVIATION &&
        fabsf(scara_arm->pose_6d[POSE_ROLL] - scara_arm->set_pose_6d[POSE_ROLL]) < TOLERABLE_ANGLE_DEVIATION)
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

    if (scara_arm->silver_mining_step == SILVER_MINING_STEP_INIT)
    {
        scara_arm->auto_operation_process = 0;

        setGrabNuggetType(SILVER_NUGGET);

        scara_arm->silver_mining_grab_end_timer = 0;

        // 若已经取到矿，则直接上提
        if (scara_arm->grabbed)
        {
            scara_arm->silver_mining_step = SILVER_MINING_STEP_LIFT;
        }
        // 普通完整流程
        else
        {
            scara_arm->silver_mining_step = SILVER_MINING_STEP_START;
        }
    }
    else if (scara_arm->silver_mining_step == SILVER_MINING_STEP_START)
    {
        scara_arm->auto_operation_process = 0;

        scara_arm->set_pose_6d[POSE_X] = 0.25f;
        scara_arm->set_pose_6d[POSE_Y] = 0.0f;
        scara_arm->set_pose_6d[POSE_Z] = SILVER_MINING_POSE_Z;
        scara_arm->set_pose_6d[POSE_YAW] = 0.0f;
        scara_arm->set_pose_6d[POSE_PITCH] = -90.0f * DEGREE_TO_RADIAN_FACTOR;
        scara_arm->set_pose_6d[POSE_ROLL] = 0.0f;

        if (fabsf(scara_arm->pose_6d[POSE_X] - scara_arm->set_pose_6d[POSE_X]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[POSE_Y] - scara_arm->set_pose_6d[POSE_Y]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[POSE_Z] - scara_arm->set_pose_6d[POSE_Z]) < TOLERABLE_DISTANCE_DEVIATION)
        {
            scara_arm->silver_mining_step = SILVER_MINING_STEP_MOVE;
        }
    }
    else if (scara_arm->silver_mining_step == SILVER_MINING_STEP_MOVE)
    {
        scara_arm->auto_operation_process = 0;

        scara_arm->set_pose_6d[POSE_X] = 0.556f;

        if (getSilverTarget() == SILVER_MID)
            scara_arm->set_pose_6d[POSE_Y] = 0.0f;
        else if (getSilverTarget() == SILVER_LEFT)
            scara_arm->set_pose_6d[POSE_Y] = SILVER_MINING_POSE_Y_OFFSET;
        else if (getSilverTarget() == SILVER_RIGHT)
            scara_arm->set_pose_6d[POSE_Y] = -SILVER_MINING_POSE_Y_OFFSET;

        if (fabsf(scara_arm->pose_6d[POSE_X] - scara_arm->set_pose_6d[POSE_X]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[POSE_Y] - scara_arm->set_pose_6d[POSE_Y]) < TOLERABLE_DISTANCE_DEVIATION)
        {
            // 清除历史误操作
            checkIfRcKeyFallingEdgeDetected(RC_LEFT);
            checkIfRcKeyFallingEdgeDetected(RC_RIGHT);

            scara_arm->last_silver_target = getSilverTarget();
            scara_arm->silver_mining_step = SILVER_MINING_STEP_READY;
        }
    }
    else if (scara_arm->silver_mining_step == SILVER_MINING_STEP_READY)
    {
        scara_arm->auto_operation_process = 1;

        if (getSilverTarget() != scara_arm->last_silver_target)
        {
            scara_arm->silver_mining_step = SILVER_MINING_STEP_MOVE;
        }

        scara_arm->set_pose_6d[POSE_Z] += ((float)getRcMouseZ() * 0.00004);

        if (checkIfRcKeyFallingEdgeDetected(RC_LEFT))
        {
            setArmGrabMode(true);

            scara_arm->silver_mining_step = SILVER_MINING_STEP_GRAB;
        }
        if (checkIfRcKeyFallingEdgeDetected(RC_RIGHT))
        {
            scara_arm->silver_mining_step = SILVER_MINING_STEP_OK;
        }
    }
    else if (scara_arm->silver_mining_step == SILVER_MINING_STEP_GRAB)
    {
        scara_arm->auto_operation_process = 1;

        scara_arm->silver_mining_grab_end_timer++;

        scara_arm->set_pose_6d[POSE_Z] -= 0.0005f;

        if (checkIfArmGrabbed() || scara_arm->silver_mining_grab_end_timer >= 500)
        {
            scara_arm->silver_mining_grab_end_timer = 0;

            scara_arm->set_pose_6d[POSE_Z] = scara_arm->pose_6d[POSE_Z] + 0.01f;

            scara_arm->grab_top = true;

            scara_arm->silver_mining_step = SILVER_MINING_STEP_LIFT;
        }
    }
    else if (scara_arm->silver_mining_step == SILVER_MINING_STEP_LIFT)
    {
        scara_arm->auto_operation_process = 1;

        scara_arm->set_pose_6d[POSE_Z] += 0.0005f;

        if (scara_arm->pose_6d[POSE_Z] > 0.4f)
        {
            // 清除历史误操作
            checkIfRcKeyFallingEdgeDetected(RC_LEFT);
            checkIfRcKeyFallingEdgeDetected(RC_RIGHT);

            scara_arm->silver_mining_step = SILVER_MINING_STEP_WAIT;
        }
    }
    else if (scara_arm->silver_mining_step == SILVER_MINING_STEP_WAIT)
    {
        scara_arm->auto_operation_process = 1;

        if (checkIfRcKeyFallingEdgeDetected(RC_LEFT))
        {
            scara_arm->silver_mining_step = SILVER_MINING_STEP_OK;
        }
        if (checkIfRcKeyFallingEdgeDetected(RC_RIGHT) && !checkIfArmGrabbed())
        {
            setArmGrabMode(false);

            scara_arm->set_pose_6d[POSE_Z] = SILVER_MINING_POSE_Z;
            scara_arm->set_pose_6d[POSE_YAW] = 0.0f;
            scara_arm->set_pose_6d[POSE_PITCH] = -90.0f * DEGREE_TO_RADIAN_FACTOR;
            scara_arm->set_pose_6d[POSE_ROLL] = 0.0f;

            scara_arm->silver_mining_step = SILVER_MINING_STEP_MOVE;
        }
    }
    else if (scara_arm->silver_mining_step == SILVER_MINING_STEP_OK)
    {
        scara_arm->auto_operation_process = 2;

        if (scara_arm->grab_top)
            scara_arm->set_pose_6d[POSE_X] = 0.24f;
        else
        {
            scara_arm->set_pose_6d[POSE_X] = 0.36f;
            scara_arm->set_pose_6d[POSE_Z] = 0.4255f;
            scara_arm->set_pose_6d[POSE_YAW] = 0.0f;
            scara_arm->set_pose_6d[POSE_PITCH] = 0.0f;
            scara_arm->set_pose_6d[POSE_ROLL] = 0.0f;
        }

        scara_arm->set_pose_6d[POSE_Y] = 0.0f;

        if (fabsf(scara_arm->pose_6d[POSE_X] - scara_arm->set_pose_6d[POSE_X]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[POSE_Y] - scara_arm->set_pose_6d[POSE_Y]) < TOLERABLE_DISTANCE_DEVIATION)
        {
            scara_arm->set_pose_6d[POSE_Z] = OPERATION_MODE_DEFAULT_Z;
            if (fabsf(scara_arm->pose_6d[POSE_Z] - scara_arm->set_pose_6d[POSE_Z]) < TOLERABLE_DISTANCE_DEVIATION)
            {
                scara_arm->grab_top = true;

                scara_arm->silver_mining_success = true;
            }
        }
    }
}

static void gold_mining_control(engineer_scara_arm_s *scara_arm)
{
    if (scara_arm->gold_mining_step == GOLD_MINING_STEP_INIT)
    {
        scara_arm->auto_operation_process = 0;

        setGrabNuggetType(GOLD_NUGGET);

        // 清除历史误操作
        if (checkIfRcKeyFallingEdgeDetected(RC_RIGHT))
            ;

        // 若已经取到矿，则直接上提
        if (scara_arm->grabbed)
        {
            scara_arm->set_joints_value[JOINT_1] += 0.065f;
            scara_arm->set_joints_value[JOINT_5] = NUGGET_PITCH_LEVELLING_OFFSET * DEGREE_TO_RADIAN_FACTOR;

            scara_arm->gold_mining_step = GOLD_MINING_STEP_PULL_OUT;
        }
        // 普通完整流程
        else
        {
            scara_arm->set_joints_value[JOINT_1] = 0.07f;
            scara_arm->set_joints_value[JOINT_2] = 85.0f * DEGREE_TO_RADIAN_FACTOR;
            scara_arm->set_joints_value[JOINT_3] = 5.0f * DEGREE_TO_RADIAN_FACTOR;
            scara_arm->set_joints_value[JOINT_4] = 0.0f;
            scara_arm->set_joints_value[JOINT_5] = 0.0f;
            scara_arm->set_joints_value[JOINT_6] = 0.0f;

            scara_arm->gold_mining_step = GOLD_MINING_STEP_START;
        }
    }
    else if (scara_arm->gold_mining_step == GOLD_MINING_STEP_START)
    {
        scara_arm->auto_operation_process = 0;

        if (fabsf(scara_arm->joints_value[JOINT_1] - scara_arm->set_joints_value[JOINT_1]) <
                TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->joints_value[JOINT_2] - scara_arm->set_joints_value[JOINT_2]) <
                TOLERABLE_ANGLE_DEVIATION &&
            fabsf(scara_arm->joints_value[JOINT_3] - scara_arm->set_joints_value[JOINT_3]) <
                TOLERABLE_ANGLE_DEVIATION &&
            fabsf(scara_arm->joints_value[JOINT_4] - scara_arm->set_joints_value[JOINT_4]) <
                TOLERABLE_ANGLE_DEVIATION &&
            fabsf(scara_arm->joints_value[JOINT_5] - scara_arm->set_joints_value[JOINT_5]) <
                TOLERABLE_ANGLE_DEVIATION &&
            fabsf(scara_arm->joints_value[JOINT_6] - scara_arm->set_joints_value[JOINT_6]) < TOLERABLE_ANGLE_DEVIATION)
        {
            setArmGrabMode(true);

            scara_arm->gold_mining_step = GOLD_MINING_STEP_OPERATION;
        }
    }
    else if (scara_arm->gold_mining_step == GOLD_MINING_STEP_OPERATION)
    {
        scara_arm->auto_operation_process = 1;

        scara_arm->set_joints_value[JOINT_1] += ((float)getRcMouseZ() * 0.00004);

        if (scara_arm->grabbed)
        {
            scara_arm->set_joints_value[JOINT_1] += 0.065f;
            scara_arm->set_joints_value[JOINT_5] = NUGGET_PITCH_LEVELLING_OFFSET * DEGREE_TO_RADIAN_FACTOR;

            scara_arm->gold_mining_step = GOLD_MINING_STEP_PULL_OUT;
        }
    }
    else if (scara_arm->gold_mining_step == GOLD_MINING_STEP_PULL_OUT)
    {
        scara_arm->auto_operation_process = 2;

        // 无力末端yaw轴以防矿石与隧道挤压
        rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT4], RFL_MOTOR_CONTROL_MODE_NO_FORCE);

        scara_arm->set_joints_value[JOINT_1] += ((float)getRcMouseZ() * 0.00004);
    }

    if (checkIfRcKeyFallingEdgeDetected(RC_RIGHT))
    {
        rflMotorSetMode(&scara_arm->joints_motors[MOTOR_JOINT4], RFL_MOTOR_CONTROL_MODE_SPEED_ANGLE);
        setArmGrabMode(false);

        scara_arm->gold_mining_step = GOLD_MINING_STEP_INIT;
    }
}

static void storage_push_control(engineer_scara_arm_s *scara_arm)
{
    if (scara_arm->storage_push_success == true)
        return;

    if (scara_arm->storage_push_step == STORAGE_PUSH_STEP_INIT)
    {
        scara_arm->auto_operation_process = 0;

        if (getStoragePushInAvailableSlot() >= STORAGE_NULL)
        {
            StorageCancelOperation(STORAGE_PUSH_IN);
            scara_arm->storage_push_success = true;
        }

        scara_arm->storage_push_end_timer = 0;
        scara_arm->storage_push_overtime_timer = 0;

        scara_arm->solution = JOINT_3_ON_THE_LEFT;

        scara_arm->storage_push_step = STORAGE_PUSH_STEP_START;
    }
    else if (scara_arm->storage_push_step == STORAGE_PUSH_STEP_START)
    {
        scara_arm->auto_operation_process = 1;

        scara_arm->set_pose_6d[POSE_X] = -0.02f;
        scara_arm->set_pose_6d[POSE_Y] = -0.425f;
        scara_arm->set_pose_6d[POSE_Z] = 0.0f;
        scara_arm->set_pose_6d[POSE_YAW] = -145.0f * DEGREE_TO_RADIAN_FACTOR;
        scara_arm->set_pose_6d[POSE_PITCH] = -90.0f * DEGREE_TO_RADIAN_FACTOR;

        if (fabsf(scara_arm->pose_6d[POSE_X] - scara_arm->set_pose_6d[POSE_X]) < TOLERABLE_DISTANCE_DEVIATION_IN_MOVE &&
            fabsf(scara_arm->pose_6d[POSE_Y] - scara_arm->set_pose_6d[POSE_Y]) < TOLERABLE_DISTANCE_DEVIATION_IN_MOVE &&
            fabsf(scara_arm->pose_6d[POSE_Z] - scara_arm->set_pose_6d[POSE_Z]) < TOLERABLE_DISTANCE_DEVIATION_IN_MOVE &&
            fabsf(scara_arm->pose_6d[POSE_YAW] - scara_arm->set_pose_6d[POSE_YAW]) <
                TOLERABLE_ANGLE_DEVIATION_IN_MOVE &&
            fabsf(scara_arm->pose_6d[POSE_PITCH] - scara_arm->set_pose_6d[POSE_PITCH]) <
                TOLERABLE_ANGLE_DEVIATION_IN_MOVE)
            scara_arm->storage_push_step = STORAGE_PUSH_STEP_SLOT;
    }
    else if (scara_arm->storage_push_step == STORAGE_PUSH_STEP_SLOT)
    {
        scara_arm->auto_operation_process = 1;

        scara_arm->set_pose_6d[POSE_YAW] = -185.0f * DEGREE_TO_RADIAN_FACTOR;

        if (fabsf(scara_arm->pose_6d[POSE_X] - scara_arm->set_pose_6d[POSE_X]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[POSE_Y] - scara_arm->set_pose_6d[POSE_Y]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[POSE_Z] - scara_arm->set_pose_6d[POSE_Z]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[POSE_YAW] - scara_arm->set_pose_6d[POSE_YAW]) < TOLERABLE_ANGLE_DEVIATION &&
            fabsf(scara_arm->pose_6d[POSE_PITCH] - scara_arm->set_pose_6d[POSE_PITCH]) < TOLERABLE_ANGLE_DEVIATION)
        {
            StorageConfirmOperation(STORAGE_PUSH_IN);

            scara_arm->storage_push_step = STORAGE_PUSH_STEP_PUSH_IN;
        }

        if (getStorageCurrentTargetSlot() == STORAGE_BACK)
        {
            scara_arm->set_pose_6d[POSE_X] = -0.37f;
        }
        else if (getStorageCurrentTargetSlot() == STORAGE_FRONT)
        {
            scara_arm->set_pose_6d[POSE_X] = -0.02f;
        }
        scara_arm->set_pose_6d[POSE_Y] = -0.4f;
    }
    else if (scara_arm->storage_push_step == STORAGE_PUSH_STEP_PUSH_IN)
    {
        scara_arm->auto_operation_process = 2;

        scara_arm->storage_push_overtime_timer++;

        if (getStorageCurrentTargetSlot() == STORAGE_BACK)
        {
            scara_arm->set_pose_6d[POSE_X] = -0.37f;
        }
        else if (getStorageCurrentTargetSlot() == STORAGE_FRONT)
        {
            scara_arm->set_pose_6d[POSE_X] = -0.02f;
        }

        scara_arm->set_pose_6d[POSE_Y] += 0.00015f;
        if (scara_arm->set_pose_6d[POSE_Y] > -0.26f)
            scara_arm->set_pose_6d[POSE_Y] = -0.26f;

        scara_arm->set_pose_6d[POSE_ROLL] += ((float)getRcMouseZ() * 0.0005f);

        if (getStorageSlotStatus(getStorageCurrentTargetSlot()) == STORAGE_SLOT_USED)
        {
            setArmGrabMode(false);

            scara_arm->set_pose_6d[POSE_X] = scara_arm->pose_6d[POSE_X] + 0.1f;
            scara_arm->set_pose_6d[POSE_Y] = scara_arm->pose_6d[POSE_Y] - 0.1f;
            scara_arm->set_pose_6d[POSE_Z] = scara_arm->pose_6d[POSE_Z] + 0.1f;

            scara_arm->storage_push_step = STORAGE_PUSH_STEP_END;
        }
    }
    else if (scara_arm->storage_push_step == STORAGE_PUSH_STEP_END)
    {
        scara_arm->auto_operation_process = 3;

        if (fabsf(scara_arm->pose_6d[POSE_X] - scara_arm->set_pose_6d[POSE_X]) < TOLERABLE_DISTANCE_DEVIATION_IN_MOVE &&
            fabsf(scara_arm->pose_6d[POSE_Y] - scara_arm->set_pose_6d[POSE_Y]) < TOLERABLE_DISTANCE_DEVIATION_IN_MOVE &&
            fabsf(scara_arm->pose_6d[POSE_Z] - scara_arm->set_pose_6d[POSE_Z]) < TOLERABLE_DISTANCE_DEVIATION_IN_MOVE)
        {
            scara_arm->storage_push_overtime_timer = 0;

            if (scara_arm->grab_top)
            {
                setPushInNuggetToward(getStorageCurrentTargetSlot(), NUGGET_FACING_FORWARD);
            }
            else
            {
                setPushInNuggetToward(getStorageCurrentTargetSlot(), NUGGET_FACING_UPWARD);
            }

            scara_arm->storage_push_step = STORAGE_PUSH_STEP_SAFE_FOLD;
        }
    }
    else if (scara_arm->storage_push_step == STORAGE_PUSH_STEP_SAFE_FOLD)
    {
        scara_arm->auto_operation_process = 3;

        scara_arm->set_pose_6d[POSE_X] = 0.122f;
        scara_arm->set_pose_6d[POSE_Y] = -0.267f;
        scara_arm->set_pose_6d[POSE_YAW] = -135.0f * DEGREE_TO_RADIAN_FACTOR;
        scara_arm->set_pose_6d[POSE_PITCH] = OPERATION_MODE_DEFAULT_PITCH;
        scara_arm->set_pose_6d[POSE_ROLL] = OPERATION_MODE_DEFAULT_ROLL;

        if (fabsf(scara_arm->pose_6d[POSE_X] - scara_arm->set_pose_6d[POSE_X]) < TOLERABLE_DISTANCE_DEVIATION_IN_MOVE &&
            fabsf(scara_arm->pose_6d[POSE_Y] - scara_arm->set_pose_6d[POSE_Y]) < TOLERABLE_DISTANCE_DEVIATION_IN_MOVE &&
            fabsf(scara_arm->pose_6d[POSE_YAW] - scara_arm->set_pose_6d[POSE_YAW]) < TOLERABLE_ANGLE_DEVIATION_IN_MOVE)
        {
            scara_arm->storage_push_success = true;
        }
    }
}

static void storage_pop_control(engineer_scara_arm_s *scara_arm)
{
    if (scara_arm->storage_pop_success == true)
        return;

    if (scara_arm->storage_pop_step == STORAGE_POP_STEP_INIT)
    {
        scara_arm->auto_operation_process = 0;

        if (getStoragePopOutAvailableSlot() >= STORAGE_NULL)
        {
            scara_arm->storage_pop_success = true;
        }

        scara_arm->solution = JOINT_3_ON_THE_LEFT;

        scara_arm->storage_pop_step = STORAGE_POP_STEP_START;
    }
    else if (scara_arm->storage_pop_step == STORAGE_POP_STEP_START)
    {
        scara_arm->auto_operation_process = 1;

        if (getNuggetPopOutToward(getStorageCurrentTargetSlot()) == NUGGET_FACING_UPWARD)
        {
            scara_arm->set_pose_6d[POSE_X] = 0.0f;
            scara_arm->set_pose_6d[POSE_Y] = -0.313f;
            scara_arm->set_pose_6d[POSE_Z] = 0.05f;
            scara_arm->set_pose_6d[POSE_YAW] = -180.0f * DEGREE_TO_RADIAN_FACTOR;
            scara_arm->set_pose_6d[POSE_PITCH] = -90.0f * DEGREE_TO_RADIAN_FACTOR;
        }
        else if (getNuggetPopOutToward(getStorageCurrentTargetSlot()) == NUGGET_FACING_FORWARD)
        {
            scara_arm->set_pose_6d[POSE_X] = 0.05f;
            scara_arm->set_pose_6d[POSE_Y] = -0.28f;
            scara_arm->set_pose_6d[POSE_Z] = 0.0f;
            scara_arm->set_pose_6d[POSE_YAW] = -190.0f * DEGREE_TO_RADIAN_FACTOR;
            scara_arm->set_pose_6d[POSE_PITCH] = 0.0f;
        }

        if (fabsf(scara_arm->pose_6d[POSE_X] - scara_arm->set_pose_6d[POSE_X]) < TOLERABLE_DISTANCE_DEVIATION_IN_MOVE &&
            fabsf(scara_arm->pose_6d[POSE_Y] - scara_arm->set_pose_6d[POSE_Y]) < TOLERABLE_DISTANCE_DEVIATION_IN_MOVE &&
            fabsf(scara_arm->pose_6d[POSE_Z] - scara_arm->set_pose_6d[POSE_Z]) < TOLERABLE_DISTANCE_DEVIATION_IN_MOVE &&
            fabsf(scara_arm->pose_6d[POSE_YAW] - scara_arm->set_pose_6d[POSE_YAW]) <
                TOLERABLE_ANGLE_DEVIATION_IN_MOVE &&
            fabsf(scara_arm->pose_6d[POSE_PITCH] - scara_arm->set_pose_6d[POSE_PITCH]) <
                TOLERABLE_ANGLE_DEVIATION_IN_MOVE)
            scara_arm->storage_pop_step = STORAGE_POP_STEP_SLOT;
    }
    else if (scara_arm->storage_pop_step == STORAGE_POP_STEP_SLOT)
    {
        scara_arm->auto_operation_process = 1;

        if (getNuggetPopOutToward(getStorageCurrentTargetSlot()) == NUGGET_FACING_UPWARD)
        {
            scara_arm->set_pose_6d[POSE_Y] = -0.313f;

            if (getStorageCurrentTargetSlot() == STORAGE_BACK)
            {
                scara_arm->set_pose_6d[POSE_X] = -0.365f;
            }
            else if (getStorageCurrentTargetSlot() == STORAGE_FRONT)
            {
                scara_arm->set_pose_6d[POSE_X] = 0.0f;
            }
        }
        else if (getNuggetPopOutToward(getStorageCurrentTargetSlot()) == NUGGET_FACING_FORWARD)
        {
            if (getStorageCurrentTargetSlot() == STORAGE_BACK)
            {
                scara_arm->set_pose_6d[POSE_X] = -0.327f;
                scara_arm->set_pose_6d[POSE_Y] = -0.3f;
            }
            else if (getStorageCurrentTargetSlot() == STORAGE_FRONT)
            {
                scara_arm->set_pose_6d[POSE_X] = 0.05f;
                scara_arm->set_pose_6d[POSE_Y] = -0.28f;
            }
        }

        if (fabsf(scara_arm->pose_6d[POSE_X] - scara_arm->set_pose_6d[POSE_X]) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm->pose_6d[POSE_Y] - scara_arm->set_pose_6d[POSE_Y]) < TOLERABLE_DISTANCE_DEVIATION)
        {
            setArmGrabMode(true);

            scara_arm->storage_pop_step = STORAGE_POP_STEP_POP_OUT;
        }
    }
    else if (scara_arm->storage_pop_step == STORAGE_POP_STEP_POP_OUT)
    {
        scara_arm->auto_operation_process = 2;

        if (getNuggetPopOutToward(getStorageCurrentTargetSlot()) == NUGGET_FACING_UPWARD)
        {
            scara_arm->set_pose_6d[POSE_Z] -= 0.0002;
            if (scara_arm->set_pose_6d[POSE_Z] < -0.05f)
                scara_arm->set_pose_6d[POSE_Z] = -0.05f;
        }
        else if (getNuggetPopOutToward(getStorageCurrentTargetSlot()) == NUGGET_FACING_FORWARD)
        {
            scara_arm->set_pose_6d[POSE_X] -= 0.0002f;

            if (getStorageCurrentTargetSlot() == STORAGE_BACK)
            {
                if (scara_arm->set_pose_6d[POSE_X] < -0.4f)
                    scara_arm->set_pose_6d[POSE_X] = -0.4f;

                scara_arm->set_pose_6d[POSE_Y] = -0.3f;
            }
            else if (getStorageCurrentTargetSlot() == STORAGE_FRONT)
            {
                if (scara_arm->set_pose_6d[POSE_X] < -0.09f)
                    scara_arm->set_pose_6d[POSE_X] = -0.09f;

                scara_arm->set_pose_6d[POSE_Y] = -0.28f;
            }
        }

        if (checkIfArmGrabbed())
        {
            StorageConfirmOperation(STORAGE_POP_OUT);

            if (getStorageCurrentTargetSlot() == STORAGE_FRONT)
            {
                scara_arm->set_pose_6d[POSE_Y] = scara_arm->pose_6d[POSE_X] + 0.06f;
            }
            scara_arm->set_pose_6d[POSE_Y] = scara_arm->pose_6d[POSE_Y] - 0.1f;
            scara_arm->set_pose_6d[POSE_Z] = scara_arm->pose_6d[POSE_Z] + 0.06f;

            scara_arm->storage_pop_step = STORAGE_POP_STEP_END;
        }
    }
    else if (scara_arm->storage_pop_step == STORAGE_POP_STEP_END)
    {
        scara_arm->auto_operation_process = 3;

        if (fabsf(scara_arm->pose_6d[POSE_Y] - scara_arm->set_pose_6d[POSE_Y]) < TOLERABLE_DISTANCE_DEVIATION_IN_MOVE &&
            fabsf(scara_arm->pose_6d[POSE_Z] - scara_arm->set_pose_6d[POSE_Z]) < TOLERABLE_DISTANCE_DEVIATION_IN_MOVE)
        {
            scara_arm->storage_pop_time_node = xTaskGetTickCount();
            scara_arm->storage_pop_step = STORAGE_POP_STEP_SAFE_FOLD;
        }
    }
    else if (scara_arm->storage_pop_step == STORAGE_POP_STEP_SAFE_FOLD)
    {
        scara_arm->auto_operation_process = 3;

        if (xTaskGetTickCount() - scara_arm->storage_pop_time_node < 600u)
        {
            scara_arm->set_pose_6d[POSE_X] = -0.02f;
            scara_arm->set_pose_6d[POSE_Y] = -0.425f;
            scara_arm->set_pose_6d[POSE_Z] = 0.0f;
            scara_arm->set_pose_6d[POSE_YAW] = -145.0f * DEGREE_TO_RADIAN_FACTOR;
            scara_arm->set_pose_6d[POSE_ROLL] = 0.0f;
            if (getNuggetPopOutToward(getStorageCurrentTargetSlot()) == NUGGET_FACING_UPWARD)
            {
                scara_arm->set_pose_6d[POSE_PITCH] = -90.0f * DEGREE_TO_RADIAN_FACTOR;
            }
            else if (getNuggetPopOutToward(getStorageCurrentTargetSlot()) == NUGGET_FACING_FORWARD)
            {
                scara_arm->set_pose_6d[POSE_PITCH] = 0.0f;
            }
        }
        else
        {
            scara_arm->set_pose_6d[POSE_X] = 0.232f;
            scara_arm->set_pose_6d[POSE_Y] = -0.281f;
            scara_arm->set_pose_6d[POSE_Z] = 0.0f;
            scara_arm->set_pose_6d[POSE_YAW] = -90.0f * DEGREE_TO_RADIAN_FACTOR;
            scara_arm->set_pose_6d[POSE_PITCH] = 0.0f;
            scara_arm->set_pose_6d[POSE_ROLL] = 0.0f;

            if (fabsf(scara_arm->pose_6d[POSE_X] - scara_arm->set_pose_6d[POSE_X]) <
                    TOLERABLE_DISTANCE_DEVIATION_IN_MOVE &&
                fabsf(scara_arm->pose_6d[POSE_Y] - scara_arm->set_pose_6d[POSE_Y]) <
                    TOLERABLE_DISTANCE_DEVIATION_IN_MOVE &&
                fabsf(scara_arm->pose_6d[POSE_Z] - scara_arm->set_pose_6d[POSE_Z]) <
                    TOLERABLE_DISTANCE_DEVIATION_IN_MOVE &&
                fabsf(scara_arm->pose_6d[POSE_YAW] - scara_arm->set_pose_6d[POSE_YAW]) <
                    TOLERABLE_ANGLE_DEVIATION_IN_MOVE &&
                fabsf(scara_arm->pose_6d[POSE_PITCH] - scara_arm->set_pose_6d[POSE_PITCH]) <
                    TOLERABLE_ANGLE_DEVIATION_IN_MOVE)
                scara_arm->storage_pop_success = true;
        }
    }
}

static void emergency_move_control(engineer_scara_arm_s *scara_arm)
{
    scara_arm->set_joints_value[JOINT_1] += (checkIfRcKeyPressed(RC_LEFT)    ? 0.0005f
                                             : checkIfRcKeyPressed(RC_RIGHT) ? -0.0005f
                                                                             : 0.0f);
}
