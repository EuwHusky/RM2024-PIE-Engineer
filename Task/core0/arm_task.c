#include "stdio.h"
#include "string.h"

#include "arm_task.h"

#include "arm_control.h"
#include "arm_kinematics.h"
#include "arm_motor.h"

#include "drv_delay.h"

#include "bsp_ma600.h"

#include "algo_data_limiting.h"

#include "INS_task.h"
#include "detect_task.h"

static void arm_init(engineer_scara_arm_s *scara_arm);
// static void update_mag_encoder_ma600_feedback(engineer_scara_arm_s *scara_arm);

static engineer_scara_arm_s scara_arm;

void arm_task(void *pvParameters)
{
    rflOsDelayMs(800);

    while (!INS_init_finished)
        rflOsDelayMs(2);
    rflOsDelayMs(30);

    arm_init(&scara_arm);

    while (1)
    {
        // 接收行为控制任务的指令 控制机械臂
        arm_mode_control(&scara_arm);

        // // 更新磁编码器反馈
        // update_mag_encoder_ma600_feedback(&scara_arm);

        // 机械臂模型更新
        arm_model_update_status(&scara_arm);
        arm_model_update_control(&scara_arm);

        // 机械臂电机更新与执行
        arm_motor_update_and_execute(&scara_arm);

        rflOsDelayMs(1);
    }
}

engineer_scara_arm_s *getArmDataPointer(void)
{
    return &scara_arm;
}

bool *getArmResetStatus(void)
{
    return &scara_arm.reset_success;
}

bool *getArmMoveHomingStatue(void)
{
    return &scara_arm.move_homing_success;
}

bool *getArmOperationHomingStatus(void)
{
    return &scara_arm.operation_homing_success;
}

static void arm_init(engineer_scara_arm_s *scara_arm)
{
    memset(scara_arm, 0, sizeof(engineer_scara_arm_s));

    scara_arm->behavior = ENGINEER_BEHAVIOR_DISABLE;
    scara_arm->last_behavior = ENGINEER_BEHAVIOR_DISABLE;

    arm_model_init(scara_arm);

    scara_arm->joint_1_homing_timer = 0;

    arm_rm_motor_can_init();
    while (detect_error(ARM_JOINT_1_L_DH) || detect_error(ARM_JOINT_1_R_DH) || detect_error(ARM_JOINT_4_DH) ||
           detect_error(ARM_JOINT_56_L_DH) || detect_error(ARM_JOINT_56_R_DH))
        rflOsDelayMs(10);
    arm_motor_init(scara_arm);

    // MA600_init();

    resetArmStartUpStatus(scara_arm->start_up_status);
    scara_arm->move_homing_success = false;
    scara_arm->operation_homing_success = false;

    scara_arm->rc = getRemoteControlPointer();

    scara_arm->customer_controller = getCustomerControllerData();
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[0], 0.15f, 0.85f);
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[1], 0.15f, 0.85f);
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[2], 0.01f, 0.99f);
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[3], 0.15f, 0.85f);
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[4], 0.15f, 0.85f);
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[5], 0.15f, 0.85f);

    // rflSlidingWindowFilterInit(&scara_arm->joint_6_encoder_angle_filter, 14, 2);
}

// static void update_mag_encoder_ma600_feedback(engineer_scara_arm_s *scara_arm)
// {
//     bool is_error = false;
//     float angle_offset = ENGINEER_ARM_JOINT_6_ENCODER_ANGLE_OFFSET;

//     if (scara_arm->joint_6_encoder_value = MA600_read_with_check(&is_error), is_error == false)
//     {
//         scara_arm->joint_6_encoder_angle = rflSlidingWindowFilterCalc(
//             &scara_arm->joint_6_encoder_angle_filter,
//             rflFloatLoopConstrain(((float)scara_arm->joint_6_encoder_value * 0.005493248f) - 180.0f - angle_offset,
//                                   -DEG_PI, DEG_PI));
//     }

//     scara_arm->joint_6_encoder_angle = 0.0f; // 还没装磁编，暂时这样，记得删
// }
