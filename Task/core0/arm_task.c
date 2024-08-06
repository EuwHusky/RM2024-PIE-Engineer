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
static void update_and_execute_grabber(engineer_scara_arm_s *scara_arm);
static void update_mag_encoder_ma600_feedback(engineer_scara_arm_s *scara_arm);

static engineer_scara_arm_s scara_arm;

void arm_task(void *pvParameters)
{
    rflOsDelayMs(800);

    while (!INS_init_finished)
        rflOsDelayMs(2);
    rflOsDelayMs(30);

    scara_arm.started = false;

    arm_init(&scara_arm);

    scara_arm.started = true;

    while (1)
    {
        // 接收行为控制任务的指令 控制机械臂
        arm_mode_control(&scara_arm);

        // 抓取机构更新与执行
        update_and_execute_grabber(&scara_arm);

        // 更新磁编码器反馈
        update_mag_encoder_ma600_feedback(&scara_arm);

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

bool *getArmStartedFlag(void)
{
    return &scara_arm.started;
}

float getArmTargetDirection(void)
{
    return atan2f(scara_arm.pose_6d[POSE_Y], scara_arm.pose_6d[POSE_X]);
}

float getArmPose(engineer_scara_arm_pose_e pose)
{
    return scara_arm.pose_6d[pose];
}

float getArmJointsValue(engineer_scara_arm_joints_e joint)
{
    return scara_arm.joints_value[joint];
}

float getArmSetJointsValue(engineer_scara_arm_joints_e joint)
{
    return scara_arm.set_joints_value[joint];
}

float getArmMotorTemperature(engineer_scara_arm_joints_motors_index_e motor_index)
{
    return rflMotorGetTemperature(&scara_arm.joints_motors[motor_index]);
}

bool checkIfArmInDefaultPose(void)
{
    return (fabsf(scara_arm.pose_6d[POSE_X] - OPERATION_MODE_DEFAULT_X) < 0.1f &&
            fabsf(scara_arm.pose_6d[POSE_Y] - OPERATION_MODE_DEFAULT_Y) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm.pose_6d[POSE_Z] - OPERATION_MODE_DEFAULT_Z) < TOLERABLE_DISTANCE_DEVIATION &&
            fabsf(scara_arm.pose_6d[POSE_YAW] - OPERATION_MODE_DEFAULT_YAW) < TOLERABLE_ANGLE_DEVIATION &&
            fabsf(scara_arm.pose_6d[POSE_PITCH] - OPERATION_MODE_DEFAULT_PITCH) < TOLERABLE_ANGLE_DEVIATION &&
            fabsf(scara_arm.pose_6d[POSE_ROLL] - OPERATION_MODE_DEFAULT_ROLL) < TOLERABLE_ANGLE_DEVIATION);
}

bool checkIfArmGrabbed(void)
{
    return scara_arm.grabbed;
}

bool checkIfLifterMotorOverheat(void)
{
    if (rflMotorGetTemperature(&scara_arm.joints_motors[MOTOR_JOINT1_LEFT]) > 65 ||
        rflMotorGetTemperature(&scara_arm.joints_motors[MOTOR_JOINT1_RIGHT]) > 65)
        return true;

    return false;
}

bool checkIfStoragePushInOverTime(void)
{
    return scara_arm.storage_push_overtime_timer > 250;
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

bool *getSilverMiningStatus(void)
{
    return &scara_arm.silver_mining_success;
}

bool *getStoragePushStatus(void)
{
    return &scara_arm.storage_push_success;
}

bool *getStoragePopStatus(void)
{
    return &scara_arm.storage_pop_success;
}

uint8_t *getArmAutoOperationProcess(void)
{
    return &scara_arm.auto_operation_process;
}

void resetArmPose(void)
{
    for (uint8_t i = 0; i < 6; i++)
        scara_arm.set_pose_6d[i] = scara_arm.pose_6d[i];
}

void ArmReadyToExchangePose(engineer_scara_arm_solution_e solution)
{

    if (scara_arm.pose_6d[POSE_Z] > 0.2f)
    {
        scara_arm.solution = solution;
        scara_arm.set_pose_6d[POSE_X] = 0.65f;
    }
    scara_arm.set_pose_6d[POSE_Z] = ENGINEER_ARM_Z_MAX_DISTANCE / 2.0f;
}

static void arm_init(engineer_scara_arm_s *scara_arm)
{
    memset(scara_arm, 0, sizeof(engineer_scara_arm_s));

    scara_arm->behavior = ENGINEER_BEHAVIOR_DISABLE;
    scara_arm->last_behavior = ENGINEER_BEHAVIOR_DISABLE;

    scara_arm->grabbed = false;
    scara_arm->grab_top = false;

    resetArmStartUpStatus(scara_arm->start_up_status);
    scara_arm->reset_success = false;
    scara_arm->move_homing_success = false;
    scara_arm->operation_homing_success = false;
    scara_arm->silver_mining_success = false;
    scara_arm->storage_push_success = false;
    scara_arm->storage_pop_success = false;

    scara_arm->joint_6_homing_timer = 0;
    scara_arm->silver_mining_step = 0;
    scara_arm->silver_mining_grab_end_timer = 0;
    scara_arm->last_silver_target = SILVER_MID;
    scara_arm->gold_mining_step = 0;
    scara_arm->storage_push_step = 0;
    scara_arm->storage_push_end_timer = 0;
    scara_arm->storage_push_overtime_timer = 0;
    scara_arm->storage_pop_step = 0;
    scara_arm->storage_pop_time_node = 0;

    scara_arm->auto_operation_process = 0;

    arm_model_init(scara_arm);

    // 气泵
    HPM_IOC->PAD[IOC_PAD_PA25].FUNC_CTL = IOC_PA25_FUNC_CTL_GPIO_A_25;
    gpio_set_pin_output_with_initial(HPM_GPIO0, ENGINEER_ARM_PUMP_GPIO_PORT, ENGINEER_ARM_PUMP_GPIO_PIN, 0);
    // 卸力阀
    HPM_IOC->PAD[IOC_PAD_PA31].FUNC_CTL = IOC_PA31_FUNC_CTL_GPIO_A_31;
    gpio_set_pin_output_with_initial(HPM_GPIO0, ENGINEER_ARM_VALVE_GPIO_PORT, ENGINEER_ARM_VALVE_GPIO_PIN, 1);
    // 气压传感器
    HPM_IOC->PAD[IOC_PAD_PA05].FUNC_CTL = IOC_PA05_FUNC_CTL_GPIO_A_05;
    gpio_set_pin_input(HPM_GPIO0, ENGINEER_ARM_SENSOR_GPIO_PORT, ENGINEER_ARM_SENSOR_GPIO_PIN);

    MA600_init();
    rflSlidingWindowFilterInit(&scara_arm->joint_6_encoder_angle_filter, 14, 2);

    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[0], 0.02f, 0.98f);
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[1], 0.02f, 0.98f);
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[2], 0.01f, 0.99f);
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[3], 0.03f, 0.97f);
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[4], 0.03f, 0.97f);
    rflFirstOrderFilterInit(&scara_arm->cc_pose_filter[5], 0.03f, 0.97f);

    arm_rm_motor_can_init();
    while (detect_error(ARM_JOINT_1_L_DH) || detect_error(ARM_JOINT_1_R_DH) || /* detect_error(ARM_JOINT_4_DH) || */
           detect_error(ARM_JOINT_56_L_DH) || detect_error(ARM_JOINT_56_R_DH))
        rflOsDelayMs(10);
    arm_motor_init(scara_arm);
}

static void update_and_execute_grabber(engineer_scara_arm_s *scara_arm)
{
    // 更新

    scara_arm->grabbed = (gpio_read_pin(HPM_GPIO0, ENGINEER_ARM_SENSOR_GPIO_PORT, ENGINEER_ARM_SENSOR_GPIO_PIN) == 0);

    if (!scara_arm->grabbed && scara_arm->behavior != ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH)
        scara_arm->grab_top = false;

    // 执行

    if (scara_arm->behavior != ENGINEER_BEHAVIOR_DISABLE)
    {
        gpio_write_pin(HPM_GPIO0, ENGINEER_ARM_PUMP_GPIO_PORT, ENGINEER_ARM_PUMP_GPIO_PIN,
                       getArmGrabMode() ? 1 : 0); // 气泵
        gpio_write_pin(HPM_GPIO0, ENGINEER_ARM_VALVE_GPIO_PORT, ENGINEER_ARM_VALVE_GPIO_PIN,
                       getArmGrabMode() ? 0 : 1); // 电磁阀
    }
}

static void update_mag_encoder_ma600_feedback(engineer_scara_arm_s *scara_arm)
{
    bool is_error = false;

    if (scara_arm->joint_6_encoder_value = MA600_read_with_check(&is_error), is_error == false)
    {
        scara_arm->joint_6_encoder_angle = (float)scara_arm->joint_6_encoder_value * 0.005493248f;
    }
}
