#ifndef _ARM_MOTOR_H__
#define _ARM_MOTOR_H__

#include "arm_task.h"

/**
 * @brief 初始化机械臂RM电机CAN通信功能
 */
extern void arm_rm_motor_can_init(void);

/**
 * @brief 初始化机械臂电机
 */
extern void arm_motor_init(engineer_scara_arm_s *scara_arm);

/**
 * @brief 更新机械臂电机状态量并执行控制
 */
extern void arm_motor_update_and_execute(engineer_scara_arm_s *scara_arm);

/**
 * @brief 设置机械臂电机控制模式
 */
extern void arm_motor_set_mode(engineer_scara_arm_s *scara_arm, rfl_motor_control_mode_e mode);

/**
 * @brief 设置机械臂电机运动速度
 */
extern void arm_motor_set_max_speed(engineer_scara_arm_s *scara_arm);

/**
 * @brief 设置机械臂电机可达角度
 */
extern void arm_motor_set_angle_limit(engineer_scara_arm_s *scara_arm, bool is_to_reset);

#endif /* _ARM_MOTOR_H__ */
