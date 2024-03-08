#ifndef _ARM_CONTROL_H__
#define _ARM_CONTROL_H__

#include "stdint.h"

#include "arm_task.h"

/**
 * @brief 获取用户输入 更新机械臂控制模式
 */
extern void arm_set_mode(engineer_scara_arm_s *scara_arm);

/**
 * @brief 根据机械臂控制模式 配置电机控制模式 更新机械臂控制量
 */
extern void arm_mode_control(engineer_scara_arm_s *scara_arm);

#define JOINT_1_HOMING_STEP_ANGLE (0.5f)
#define JOINT_1_HOMING_TORQUE_THRESHOLD (1.0f)
#define JOINT_1_HOMING_ANGLE (0.0f)
#define JOINT_1_START_ANGLE (0.0f)

#define JOINT_2_HOMING_STEP_ANGLE (0.05f)
#define JOINT_2_HOMING_TORQUE_THRESHOLD (0.5f)
#define JOINT_2_HOMING_ANGLE (98.1f)
#define JOINT_2_START_ANGLE (90.0f)

#define JOINT_3_START_ANGLE (90.0f)

#define JOINT_4_START_ANGLE (0.0f)

#define JOINT_5_HOMING_STEP_ANGLE (0.5f)
#define JOINT_5_HOMING_ANGLE (80.0f)

#define JOINT_6_START_ANGLE (0.0f)

#define ARM_NO_FORCE_MODE_RC_KEY_VALUE (1)
#define ARM_JOINTS_MODE_RC_KEY_VALUE (3)
#define ARM_POSE_MODE_RC_KEY_VALUE (2)

#define ARM_RC_DEADLINE (3)

#define JOINT_1_CONTROL_SEN (0.0003f)
#define JOINT_2_CONTROL_SEN (0.04f)
#define JOINT_3_CONTROL_SEN (0.04f)
#define JOINT_4_CONTROL_SEN (0.04f)
#define JOINT_5_CONTROL_SEN (0.04f)
#define JOINT_6_CONTROL_SEN (0.04f)

#define POSE_X_CONTROL_SEN (0.001f)
#define POSE_Y_CONTROL_SEN (0.001f)
#define POSE_Z_CONTROL_SEN (0.001f)
#define POSE_AY_CONTROL_SEN (0.005f)
#define POSE_AP_CONTROL_SEN (0.005f)
#define POSE_AR_CONTROL_SEN (0.005f)

#define CUSTOMER_X_CONTROL_SEN (1.0f)
#define CUSTOMER_Y_CONTROL_SEN (1.0f)
#define CUSTOMER_Z_CONTROL_SEN (1.0f)
#define CUSTOMER_AY_CONTROL_SEN (1.0f)
#define CUSTOMER_AP_CONTROL_SEN (1.0f)
#define CUSTOMER_AR_CONTROL_SEN (1.0f)

#endif /* _ARM_CONTROL_H__ */
