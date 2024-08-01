#ifndef _BEHAVIOR_TASK_H__
#define _BEHAVIOR_TASK_H__

#include "stdbool.h"

#include "remote_control.h"

#define ENGINEER_DISABLE_DT7_SWITCH_VALUE (1)
#define ENGINEER_MOVE_DT7_SWITCH_VALUE (3)
#define ENGINEER_OPERATE_DT7_SWITCH_VALUE (2)

typedef enum EngineerBehavior
{
    ENGINEER_BEHAVIOR_DISABLE, // 失能
    ENGINEER_BEHAVIOR_RESET,   // 复位

    ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING, // 自动归位 到机动模式默认位置
    ENGINEER_BEHAVIOR_MOVE,             // 机动

    ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING, // 自动归位 到作业模式默认位置
    ENGINEER_BEHAVIOR_AUTO_SILVER_MINING,    // 半自动作业 取银矿
    ENGINEER_BEHAVIOR_AUTO_GOLD_MINING,      // 半自动作业 取金矿
    ENGINEER_BEHAVIOR_MANUAL_OPERATION,      // 手动作业

    ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH, // 自动存矿
    ENGINEER_BEHAVIOR_AUTO_STORAGE_POP,  // 自动取矿
} engineer_behavior_e;

typedef enum EngineerSilverMiningTarget
{
    SILVER_LEFT = 0,
    SILVER_MID,
    SILVER_RIGHT,
} engineer_silver_mining_target_e;

typedef struct EngineerBehaviorManager
{
    /* 基本 */

    engineer_behavior_e behavior;
    engineer_behavior_e last_behavior;

    bool *arm_started;
    bool *chassis_started;
    bool *gimbal_started;

    /* 机器人状态 */

    bool robot_survival_status; // 存活为真
    bool last_robot_survival_status;

    bool need_reboot;
    uint8_t reboot_timer;

    bool buzzer_beep;
    uint8_t buzzer_timer;

    /* 模块状态输入 */

    bool *arm_reset_success;
    bool *arm_move_homing_success;
    bool *arm_operation_homing_success;
    bool *silver_mining_success;
    bool *storage_push_success;
    bool *storage_pop_success;
    bool *gimbal_reset_success;
    bool *gimbal_move_homing_success;
    bool *gimbal_operation_homing_success;

    /* 模块状态输出 */

    bool arm_switch_solution;
    bool arm_grab;
    bool reset_ui;
    bool show_silver_vau;
    engineer_silver_mining_target_e silver_target;
    bool motor_failure_detected;
    uint32_t motor_failure_detect_timer;

    /* 设备 */

    dt7_toggle_switch_position_e dt7_behavior_switch_value;
    dt7_toggle_switch_position_e last_dt7_behavior_switch_value;
    int16_t dt7_reset_trigger_value;   // 使用DT7复位的触发器值
    uint32_t dt7_reset_trigger_timer;  // 使用DT7复位的触发计时器
    uint32_t km_disable_trigger_timer; // 使用键鼠失能的触发计时器
    uint32_t km_reset_trigger_timer;   // 使用键鼠复位的触发计时器
    int16_t dt7_pump_trigger_value;
    uint32_t dt7_pump_trigger_timer;
    int16_t dt7_arm_grab_trigger_value;
    uint32_t dt7_arm_grab_trigger_timer;
    int16_t dt7_arm_switch_trigger_value;  // 使用DT7切换机械臂解算的触发器值
    uint32_t dt7_arm_switch_trigger_timer; // 使用DT7切换机械臂解算的触发计时器
    uint32_t km_reboot_trigger_timer;      // 使用键鼠重启的触发计时器

    /* 给检录时用来快捷开气路的几把玩意 */
    uint8_t pump_flag;

} engineer_behavior_manager_s;

extern void behavior_task(void *pvParameters);
extern const engineer_behavior_manager_s *getEngineerBehaviorManagerPointer(void);

extern engineer_behavior_e getEngineerCurrentBehavior(void);
extern engineer_behavior_e getEngineerLastBehavior(void);

extern void setArmGrabMode(bool enable);

extern bool checkIfArmNeedSwitchSolution(void);
extern bool getArmGrabMode(void);
extern bool checkIfNeedResetUi(void);
extern bool checkIfNeedRebootCore(void);
extern bool checkIfNeedShowSilverVau(void);
extern engineer_silver_mining_target_e getSilverTarget(void);
extern bool checkIfMotorFailureDetected(void);

#endif /* _BEHAVIOR_TASK_H__ */
