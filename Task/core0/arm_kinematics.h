#ifndef _ARM_KINEMATICS_H__
#define _ARM_KINEMATICS_H__

#include "stdint.h"

#include "arm_task.h"

/**
 * @brief 初始化机械臂模型
 */
extern void arm_model_init(engineer_scara_arm_s *scara_arm);

/**
 * @brief 更新机械臂模型状态量 正运动学分析
 */
extern void arm_model_update_status(engineer_scara_arm_s *scara_arm);

/**
 * @brief 更新机械臂模型控制量 逆运动学解算
 */
extern void arm_model_update_control(engineer_scara_arm_s *scara_arm);

#define ARM_JI_TO_BASE_X (0.0f)
#define ARM_JI_TO_BASE_Y (0.0f)
#define ARM_JI_TO_BASE_Z (0.0f)
#define ARM_JI_TO_BASE_ANGLE_Z (0.0f)
#define ARM_JI_TO_BASE_ANGLE_Y (0.0f)
#define ARM_JI_TO_BASE_ANGLE_X (0.0f)

#define ARM_TOOL_TO_J6_X (0.0f)
#define ARM_TOOL_TO_J6_Y (0.0f)
#define ARM_TOOL_TO_J6_Z (0.0f)
#define ARM_TOOL_TO_J6_ANGLE_Z (RAD_PI)
#define ARM_TOOL_TO_J6_ANGLE_Y (0.0f)
#define ARM_TOOL_TO_J6_ANGLE_X (0.0f)

#define ARM_JOINT_1_DH_ALPHA (0.0f)
#define ARM_JOINT_1_DH_A (0.0f)
#define ARM_JOINT_1_DH_D (0.0f)
#define ARM_JOINT_1_DH_THETA (0.0f)
#define ARM_JOINT_1_DH_THETA_OFFSET (0.0f)

#define ARM_JOINT_2_DH_ALPHA (0.0f)
#define ARM_JOINT_2_DH_A (0.0f)
#define ARM_JOINT_2_DH_D (0.0f)
#define ARM_JOINT_2_DH_THETA (0.0f)
#define ARM_JOINT_2_DH_THETA_OFFSET (0.0f)

#define ARM_JOINT_3_DH_ALPHA (0.0f)
#define ARM_JOINT_3_DH_A (ENGINEER_ARM_1_LENGTH)
#define ARM_JOINT_3_DH_D (0.0f)
#define ARM_JOINT_3_DH_THETA (0.0f)
#define ARM_JOINT_3_DH_THETA_OFFSET (0.0f)

#define ARM_JOINT_4_DH_ALPHA (0.0f)
#define ARM_JOINT_4_DH_A (ENGINEER_ARM_2_LENGTH)
#define ARM_JOINT_4_DH_D (0.0f)
#define ARM_JOINT_4_DH_THETA (0.0f)
#define ARM_JOINT_4_DH_THETA_OFFSET (0.0f)

#define ARM_JOINT_5_DH_ALPHA (RAD_PI / 2.0f)
#define ARM_JOINT_5_DH_A (ENGINEER_ARM_3_LENGTH)
#define ARM_JOINT_5_DH_D (0.0f)
#define ARM_JOINT_5_DH_THETA (0.0f)
#define ARM_JOINT_5_DH_THETA_OFFSET (RAD_PI / 2.0f)

#define ARM_JOINT_6_DH_ALPHA (-RAD_PI / 2.0f)
#define ARM_JOINT_6_DH_A (0.0f)
#define ARM_JOINT_6_DH_D (ENGINEER_ARM_4_LENGTH)
#define ARM_JOINT_6_DH_THETA (0.0f)
#define ARM_JOINT_6_DH_THETA_OFFSET (0.0f)

#define PREVENT_DISTANCE (0.002f)

#endif /* _ARM_KINEMATICS_H__ */
