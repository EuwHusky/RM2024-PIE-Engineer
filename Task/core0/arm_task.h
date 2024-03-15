#ifndef _ARM_TASK_H__
#define _ARM_TASK_H__

#include "stdint.h"

#include "board.h"

#include "dev_motor.h"

#include "algo_filter.h"
#include "algo_matrix.h"

#include "referee.h"
#include "remote_control.h"

typedef enum
{
    ARM_MODE_NO_FORCE, // 无力 跟没上电一样
    ARM_MODE_STARTING, // 启动 各个关节全部重新获取初始角度
    ARM_MODE_JOINTS,   // 关节控制 既挖掘机式控制
    ARM_MODE_POSE,     // 位姿控制
    ARM_MODE_CUSTOMER, // 自定义控制器控制
} arm_mode_e;

typedef struct EngineerScaraArm
{
    /*自身属性*/

    arm_mode_e mode;
    arm_mode_e last_mode;
    bool is_arm_ready;
    bool is_joints_ready[6];
    char last_mode_control_key_value;

    /*运动学模型*/

    rfl_matrix_instance j1_to_base_tmat; /*关节1坐标系到基坐标系变换矩阵*/
    float j1_to_base_tmat_data[16];
    rfl_matrix_instance tool_to_j6_tmat; /*工具坐标系到关节6坐标系变换矩阵*/
    float tool_to_j6_tmat_data[16];
    rfl_matrix_instance tool_to_base_tmat; /*工具坐标系到基坐标系变换矩阵*/
    float tool_to_base_tmat_data[16];
    /**
     * @brief   Scara构型机械臂DH参数
     * [0:5]    Joint 1-6
     * [x][0]   alpha
     * [x][1]   a
     * [x][2]   d
     * [x][3]   theta
     * [x][4]   theta_offset
     */
    float dh[6][5];

    /*状态量*/

    /**
     * @brief   工具坐标系在基坐标系六自由度位姿 距离单位 m 角度单位 rad
     * [0:5]    X Y Z YAW PITCH ROLL
     */
    float pose_6d[6];
    /**
     * @brief   关节变量 距离单位 m 角度单位 rad
     * [0:5]    J1-distance J2-angle J3-angle J4-angle J5-angle J6-angle
     */
    float joints_value[6];

    /*控制量*/

    bool use_normal_solution; /*使用默认解 默认解下关节3角度为正*/
    /**
     * @brief   预期的工具坐标系在基坐标系六自由度位姿 距离单位 m 角度单位 rad
     * [0:5]    X Y Z YAW PITCH ROLL
     */
    float set_pose_6d[6];
    /**
     * @brief   逆运动学解算得到的期望关节变量 距离单位 m 角度单位 rad
     * [0:5]    J1-distance J2-angle J3-angle J4-angle J5-angle J6-angle
     */
    float set_joints_value[6];

    /*设备*/

    const RC_ctrl_t *dr16_rc;
    const custom_robot_data_t *custom_cmd;
    const remote_control_t *custom_mk;
    float last_custom_rc_cmd[6];

    uint16_t encoder_value[2];
    float encoder_angle[2]; // 磁编获取到的关节4、6的绝对角度，逆时针为正，单位为角度deg
    sliding_window_filter_s_t encoder_angle_filter[2];

    rfl_motor_s joint_1_motor[2];  // 0-左 1-右
    rfl_motor_s joint_23_motor[2]; // 0-2 1-3
    rfl_motor_s joint_4_motor[1];
    rfl_motor_s joint_56_motor[2]; // 0-左 1-右

} engineer_scara_arm_s;

extern engineer_scara_arm_s scara_arm;

extern void arm_task(void *pvParameters);
extern engineer_scara_arm_s *getArmDataPointer(void);

/* 机械臂结构参数 */

#define ENGINEER_ARM_1_LENGTH (0.306f)  /*第一节小臂臂长*/
#define ENGINEER_ARM_2_LENGTH (0.271f)  /*第二节小臂臂长*/
#define ENGINEER_ARM_3_LENGTH (0.073f)  /*第三节小臂臂长*/
#define ENGINEER_ARM_4_LENGTH (0.0505f) /*第四节小臂臂长*/

#define EFFECTOR_TIMING_BELT_TRANSMISSION_RATIO (2.0f)        /*同步带传动比*/
#define EFFECTOR_CONICAL_TOOTH_PAIR_TRANSMISSION_RATIO (2.0f) /*锥齿组传动比*/

/* 机械臂模型参数 */

// 位姿可达范围
#define ENGINEER_ARM_XY24_MAX_DISTANCE (ENGINEER_ARM_1_LENGTH + ENGINEER_ARM_2_LENGTH) /*关节2到关节4的最大伸展距离*/
#define ENGINEER_ARM_XY24_MIN_DISTANCE (0.1f) /*关节2到关节4的最小伸展距离*/
#define ENGINEER_ARM_PITCH_MAX_ANGLE (85.0f)  /* 末端PITCH最大角度 */
#define ENGINEER_ARM_PITCH_MIN_ANGLE (-85.0f) /* 末端PITCH最小角度 */
#define ENGINEER_ARM_ROLL_MAX_ANGLE (180.0f)  /* 末端ROLL最大角度 */
#define ENGINEER_ARM_ROLL_MIN_ANGLE (-180.0f) /* 末端ROLL最小角度 */

// 关节1电机距离角度转换系数 单位 degree/m
#define ENGINEER_ARM_JOINT_1_DISTANCE_TO_ANGLE_FACTOR (1260.0f)

// 正常运行时的关节可达范围
#define ENGINEER_ARM_JOINT_1_MAX_DISTANCE (0.5f)
#define ENGINEER_ARM_JOINT_1_MIN_DISTANCE (0.0f)
#define ENGINEER_ARM_JOINT_1_MAX_ANGLE                                                                                 \
    (ENGINEER_ARM_JOINT_1_MAX_DISTANCE * ENGINEER_ARM_JOINT_1_DISTANCE_TO_ANGLE_FACTOR)
#define ENGINEER_ARM_JOINT_1_MIN_ANGLE                                                                                 \
    (ENGINEER_ARM_JOINT_1_MIN_DISTANCE * ENGINEER_ARM_JOINT_1_DISTANCE_TO_ANGLE_FACTOR)
#define ENGINEER_ARM_JOINT_2_MAX_ANGLE (98.3f)
#define ENGINEER_ARM_JOINT_2_MIN_ANGLE (-98.3f)
#define ENGINEER_ARM_JOINT_3_MAX_ANGLE (175.0f)
#define ENGINEER_ARM_JOINT_3_MIN_ANGLE (-175.0f)
#define ENGINEER_ARM_JOINT_4_MAX_ANGLE (180.0f)
#define ENGINEER_ARM_JOINT_4_MIN_ANGLE (-180.0f)
#define ENGINEER_ARM_JOINT_5_MAX_ANGLE (ENGINEER_ARM_PITCH_MAX_ANGLE)
#define ENGINEER_ARM_JOINT_5_MIN_ANGLE (ENGINEER_ARM_PITCH_MIN_ANGLE)
#define ENGINEER_ARM_JOINT_6_MAX_ANGLE (ENGINEER_ARM_ROLL_MAX_ANGLE)
#define ENGINEER_ARM_JOINT_6_MIN_ANGLE (ENGINEER_ARM_ROLL_MIN_ANGLE)

// 关节初始化扩展可达范围 用于归中操作 有一定的危险性
#define ENGINEER_ARM_JOINT_1_INITIAL_MAX_DISTANCE (0.6f)
#define ENGINEER_ARM_JOINT_1_INITIAL_MIN_DISTANCE (-0.6f)
#define ENGINEER_ARM_JOINT_1_INITIAL_MAX_ANGLE                                                                         \
    (ENGINEER_ARM_JOINT_1_INITIAL_MAX_DISTANCE * ENGINEER_ARM_JOINT_1_DISTANCE_TO_ANGLE_FACTOR)
#define ENGINEER_ARM_JOINT_1_INITIAL_MIN_ANGLE                                                                         \
    (ENGINEER_ARM_JOINT_1_INITIAL_MIN_DISTANCE * ENGINEER_ARM_JOINT_1_DISTANCE_TO_ANGLE_FACTOR)

/* 机械臂设备参数 */

#define ENGINEER_ARM_JOINTS_123_MOTORS_CAN_ORDINAL (1)
#define ENGINEER_ARM_JOINTS_456_MOTORS_CAN_ORDINAL (2)
#define ENGINEER_ARM_JOINTS_123_RM_MOTORS_CAN_SLAVE_ID (0x200)
#define ENGINEER_ARM_JOINTS_456_RM_MOTORS_CAN_SLAVE_ID (0x200)

// 电机控制范围 由于关节56共用两个电机 故需要单独处理 其他关节的电机直接使用关节可达范围作为控制范围
#define ENGINEER_ARM_JOINT_56_MOTOR_MAX_ANGLE                                                                          \
    (ENGINEER_ARM_JOINT_5_MAX_ANGLE * EFFECTOR_TIMING_BELT_TRANSMISSION_RATIO +                                        \
     ENGINEER_ARM_JOINT_6_MAX_ANGLE * EFFECTOR_CONICAL_TOOTH_PAIR_TRANSMISSION_RATIO *                                 \
         EFFECTOR_TIMING_BELT_TRANSMISSION_RATIO)
#define ENGINEER_ARM_JOINT_56_MOTOR_MIN_ANGLE                                                                          \
    (ENGINEER_ARM_JOINT_5_MIN_ANGLE * EFFECTOR_TIMING_BELT_TRANSMISSION_RATIO +                                        \
     ENGINEER_ARM_JOINT_6_MIN_ANGLE * EFFECTOR_CONICAL_TOOTH_PAIR_TRANSMISSION_RATIO *                                 \
         EFFECTOR_TIMING_BELT_TRANSMISSION_RATIO)

// 电机控制器参数
// #define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_KP (2.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_KP (0.6f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_KI (0.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_KD (0.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_MAX_IOUT (0.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_ANGLE_PID_MAX_OUT (16.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_KP (1200.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_KI (12.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_KD (0.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_MAX_IOUT (4000.0f)
#define ENGINEER_ARM_JOINT_1_RM_M3508_SPEED_PID_MAX_OUT (16000.0f)
// #define ENGINEER_ARM_JOINT_2_UNITREE_GO_M8010_6_K_ANGLE (5.0f)
// #define ENGINEER_ARM_JOINT_2_UNITREE_GO_M8010_6_K_SPEED (0.03f)
// #define ENGINEER_ARM_JOINT_3_UNITREE_GO_M8010_6_K_ANGLE (4.0f)
// #define ENGINEER_ARM_JOINT_3_UNITREE_GO_M8010_6_K_SPEED (0.03f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_KP (0.8f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_KI (0.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_KD (0.2f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_MAX_IOUT (0.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_ANGLE_PID_MAX_OUT (16.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_KP (1200.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_KI (8.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_KD (0.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_MAX_IOUT (4000.0f)
#define ENGINEER_ARM_JOINT_4_RM_M3508_SPEED_PID_MAX_OUT (16000.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_KP (0.8f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_KI (0.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_KD (0.2f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_MAX_IOUT (0.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_ANGLE_PID_MAX_OUT (16.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_KP (2000.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_KI (100.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_KD (0.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_MAX_IOUT (2000.0f)
#define ENGINEER_ARM_JOINT_56_RM_M2006_SPEED_PID_MAX_OUT (10000.0f)

// 磁编码器安装偏差
#define ENGINEER_ARM_JOINT_4_ENCODER_ANGLE_OFFSET (0.0f)
#define ENGINEER_ARM_JOINT_6_ENCODER_ANGLE_OFFSET (0.0f)

#endif /* _ARM_TASK_H__ */
