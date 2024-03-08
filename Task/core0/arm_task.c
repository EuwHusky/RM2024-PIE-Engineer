#include "stdio.h"
#include "string.h"

#include "arm_task.h"

#include "arm_control.h"
#include "arm_kinematics.h"
#include "arm_motor.h"

#include "FreeRTOS.h"
#include "task.h"

#include "bsp_ma600.h"

#include "algo_data_limiting.h"

#include "INS_task.h"
#include "detect_task.h"

engineer_scara_arm_s scara_arm;

static void arm_init(engineer_scara_arm_s *scara_arm);
static void update_mag_encoder_ma600_feedback(engineer_scara_arm_s *scara_arm);

void arm_task(void *pvParameters)
{
    while (!INS_init_finished)
        vTaskDelay(2);
    vTaskDelay(30);

    arm_init(&scara_arm);

    while (1)
    {
        // 接收用户输入的模式控制指令 设定机械臂工作模式
        arm_set_mode(&scara_arm);
        // 根据工作模式 接收用户输入的关节/位姿操控指令 控制机械臂
        arm_mode_control(&scara_arm);

        // 更新磁编码器反馈
        // update_mag_encoder_ma600_feedback(&scara_arm);

        // 机械臂模型更新
        arm_model_update_status(&scara_arm);
        arm_model_update_control(&scara_arm);

        // 机械臂电机更新与执行
        arm_motor_update_and_execute(&scara_arm);

        vTaskDelay(2);
    }
}

// const engineer_scara_arm_s *scara_data_p;
// void print_task(void *pvParameters)
// {
//     while (!INS_init_finished)
//         vTaskDelay(8);
//     vTaskDelay(300);

//     scara_data_p = getArmDataPointer();
//     while (true)
//     {
//         printf("%d\n", scara_data_p->test_num);
//         vTaskDelay(500);
//     }
// }

static void arm_init(engineer_scara_arm_s *scara_arm)
{
    memset(scara_arm, 0, sizeof(engineer_scara_arm_s));

    arm_model_init(scara_arm);

    arm_motor_init(scara_arm);

    scara_arm->mode = ARM_MODE_NO_FORCE;
    scara_arm->last_mode = ARM_MODE_NO_FORCE;
    scara_arm->is_arm_ready = false;
    for (uint8_t i = 0; i < 6; i++)
        scara_arm->is_joints_ready[i] = false;
    scara_arm->last_mode_control_key_value = 1;

    scara_arm->dr16_rc = get_remote_control_point();
    scara_arm->custom_cmd = getCustomerControllerData();
    scara_arm->custom_mk = getRemoteControlData();

    scara_arm->last_custom_rc_cmd[0] = scara_arm->custom_cmd->x;
    scara_arm->last_custom_rc_cmd[1] = scara_arm->custom_cmd->y;
    scara_arm->last_custom_rc_cmd[2] = scara_arm->custom_cmd->z;
    scara_arm->last_custom_rc_cmd[3] = scara_arm->custom_cmd->yaw;
    scara_arm->last_custom_rc_cmd[4] = scara_arm->custom_cmd->pitch;
    scara_arm->last_custom_rc_cmd[5] = scara_arm->custom_cmd->roll;

    for (uint8_t i = 0; i < 2; i++)
        rlfSlidingWindowFilterInit(scara_arm->encoder_angle_filter + i, 11, 2);
}

static void update_mag_encoder_ma600_feedback(engineer_scara_arm_s *scara_arm)
{
    bool is_error = false;
    float angle_offset[2] = {ENGINEER_ARM_JOINT_4_ENCODER_ANGLE_OFFSET, ENGINEER_ARM_JOINT_6_ENCODER_ANGLE_OFFSET};
    for (uint8_t i = 0; i < 2; i++)
    {
        if (scara_arm->encoder_value[i] = MA600_read_with_check(&is_error, i), is_error == false)
        {
            if (i == 2)
                scara_arm->encoder_value[i] = 65535 - scara_arm->encoder_value[i];

            scara_arm->encoder_angle[i] = rlfSlidingWindowFilterCalc(
                scara_arm->encoder_angle_filter + i,
                rflFloatLoopConstrain(((float)scara_arm->encoder_value[i] * 0.005493248f) - 180.0f - angle_offset[i],
                                      -DEG_PI, DEG_PI));
        }
    }

    scara_arm->encoder_angle[3] = 0.0f; // 因为关节6磁编失效临时写的 记得删
}

engineer_scara_arm_s *getArmDataPointer(void)
{
    return &scara_arm;
}
