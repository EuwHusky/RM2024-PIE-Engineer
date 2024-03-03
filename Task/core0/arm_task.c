#include "stdio.h"
#include "string.h"

#include "arm_task.h"

#include "FreeRTOS.h"
#include "task.h"

#include "arm_control.h"
#include "arm_kinematics.h"
#include "arm_motor.h"

// #include "board.h"

#include "drv_can.h"

#include "bsp_ma600.h"

#include "dev_motor.h"

#include "algo_data_limiting.h"

#include "INS_task.h"
#include "detect_task.h"

engineer_scara_arm_s arm = {0};

static void arm_init(engineer_scara_arm_s *arm);
static void update_mag_encoder_ma600_feedback(engineer_scara_arm_s *arm);

void arm_task(void *pvParameters)
{
    while (!INS_init_finished)
        vTaskDelay(2);
    vTaskDelay(30);

    arm_init(&arm);

    while (1)
    {
        // 接收用户输入的模式控制指令 设定机械臂工作模式
        arm_set_mode(&arm);
        // 根据工作模式 接收用户输入的关节/位姿操控指令 控制机械臂
        arm_mode_control(&arm);

        // 更新磁编码器反馈
        update_mag_encoder_ma600_feedback(&arm);

        // 机械臂模型更新
        arm_model_update_status(&arm);
        arm_model_update_control(&arm);

        // 机械臂电机更新与执行
        arm_motor_update_and_execute(&arm);

        vTaskDelay(2); // 在达妙电机CAN发送后还各有一个1ms延迟，所以总共有4ms周期
    }
}

static void arm_init(engineer_scara_arm_s *arm)
{
    memset(arm, 0, sizeof(engineer_scara_arm_s));

    arm_model_init(arm);

    arm_motor_init(arm);

    arm->mode = ARM_MODE_NO_FORCE;
    arm->last_mode = ARM_MODE_NO_FORCE;
    arm->is_arm_ready = false;
    for (uint8_t i = 0; i < 6; i++)
        arm->is_joints_ready[i] = false;
    arm->last_mode_control_key_value = 1;

    arm->dr16_rc = get_remote_control_point();
    arm->custom_cmd = getCustomerControllerData();
    arm->custom_mk = getRemoteControlData();

    arm->last_custom_rc_cmd[0] = arm->custom_cmd->x;
    arm->last_custom_rc_cmd[1] = arm->custom_cmd->y;
    arm->last_custom_rc_cmd[2] = arm->custom_cmd->z;
    arm->last_custom_rc_cmd[3] = arm->custom_cmd->yaw;
    arm->last_custom_rc_cmd[4] = arm->custom_cmd->pitch;
    arm->last_custom_rc_cmd[5] = arm->custom_cmd->roll;

    for (uint8_t i = 0; i < 2; i++)
        rlfSlidingWindowFilterInit(arm->encoder_angle_filter + i, 11, 2);
}

static void update_mag_encoder_ma600_feedback(engineer_scara_arm_s *arm)
{
    bool is_error = false;
    float angle_offset[2] = {ENGINEER_ARM_JOINT_4_ENCODER_ANGLE_OFFSET, ENGINEER_ARM_JOINT_6_ENCODER_ANGLE_OFFSET};
    for (uint8_t i = 0; i < 2; i++)
    {
        if (arm->encoder_value[i] = MA600_read_with_check(&is_error, i), is_error == false)
        {
            if (i == 2)
                arm->encoder_value[i] = 65535 - arm->encoder_value[i];

            arm->encoder_angle[i] = rlfSlidingWindowFilterCalc(
                arm->encoder_angle_filter + i,
                rflFloatLoopConstrain(((float)arm->encoder_value[i] * 0.005493248f) - 180.0f - angle_offset[i], -DEG_PI,
                                      DEG_PI));
        }
    }

    arm->encoder_angle[3] = 0.0f; // 因为关节6磁编失效临时写的 记得删
}
