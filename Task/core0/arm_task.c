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
        update_mag_encoder_ma600_feedback(&scara_arm);

        // 机械臂模型更新
        arm_model_update_status(&scara_arm);
        arm_model_update_control(&scara_arm);

        // 机械臂电机更新与执行
        arm_motor_update_and_execute(&scara_arm);

        vTaskDelay(2);
    }
}

static void arm_init(engineer_scara_arm_s *scara_arm)
{
    memset(scara_arm, 0, sizeof(engineer_scara_arm_s));

    arm_model_init(scara_arm);

    arm_motor_init(scara_arm);

    MA600_init();

    scara_arm->mode = ARM_MODE_NO_FORCE;
    scara_arm->last_mode = ARM_MODE_NO_FORCE;
    scara_arm->is_arm_ready = false;
    for (uint8_t i = 0; i < 6; i++)
        scara_arm->is_joints_ready[i] = false;
    scara_arm->last_mode_control_key_value = 1;

    scara_arm->dbus_rc = get_remote_control_point();
    scara_arm->vt_customer_rc = getCustomerControllerData();
    scara_arm->vt_mk = getRemoteControlData();

    for (uint8_t i = 0; i < 2; i++)
        rlfSlidingWindowFilterInit(&scara_arm->encoder_angle_filter, 11, 2);
}

static void update_mag_encoder_ma600_feedback(engineer_scara_arm_s *scara_arm)
{
    bool is_error = false;
    float angle_offset = ENGINEER_ARM_JOINT_6_ENCODER_ANGLE_OFFSET;

    if (scara_arm->encoder_value = MA600_read_with_check(&is_error), is_error == false)
    {
        scara_arm->encoder_angle = rlfSlidingWindowFilterCalc(
            &scara_arm->encoder_angle_filter,
            rflFloatLoopConstrain(((float)scara_arm->encoder_value * 0.005493248f) - 180.0f - angle_offset, -DEG_PI,
                                  DEG_PI));
    }
}

engineer_scara_arm_s *getArmDataPointer(void)
{
    return &scara_arm;
}
