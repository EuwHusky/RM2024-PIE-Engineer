#ifndef _ARM_CONTROL_H__
#define _ARM_CONTROL_H__

#include "stdint.h"

#include "arm_task.h"

#define JOINT_1_HOMING_STEP_ANGLE (0.01f)
#define JOINT_1_HOMING_TORQUE_THRESHOLD (2.0f)
#define JOINT_1_HOMING_ANGLE (-13.021768f)

#define JOINT_2_START_ANGLE (0.0f)

#define JOINT_3_START_ANGLE (0.0f)

#define JOINT_4_HOMING_STEP_ANGLE (0.005f)
#define JOINT_4_HOMING_TORQUE_THRESHOLD (1.5f)
#define JOINT_4_HOMING_ANGLE (-127.5f)
#define JOINT_4_START_ANGLE (0.0f)

#define JOINT_5_HOMING_STEP_ANGLE (0.005f)
#define JOINT_5_HOMING_TORQUE_THRESHOLD (5.0f)
#define JOINT_5_HOMING_ANGLE (-85.0f)
#define JOINT_5_START_ANGLE (0.0f)

#define JOINT_6_START_ANGLE (0.0f)

#define ARM_NO_FORCE_MODE_RC_KEY_VALUE (1)
#define ARM_WORK_MODE_RC_KEY_VALUE (3)

#define ARM_RC_DEADLINE (2)

#define JOINT_1_CONTROL_SEN (0.0002f)
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

/**
 * @brief 根据机械臂控制模式 配置电机控制模式 更新机械臂控制量
 */
extern void arm_mode_control(engineer_scara_arm_s *scara_arm);

#endif /* _ARM_CONTROL_H__ */
