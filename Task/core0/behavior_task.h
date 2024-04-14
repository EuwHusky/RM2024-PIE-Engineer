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

    ENGINEER_BEHAVIOR_MOVE, // 机动

    ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING,      // 自动归位 到机动模式默认位置
    ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING, // 自动归位 到作业模式默认位置
    ENGINEER_BEHAVIOR_AUTO_SILVER_MINING,    // 自动作业 取银矿
    // ENGINEER_BEHAVIOR_AUTO_GOLD_MINING,      // 自动作业 取中央金矿
    ENGINEER_BEHAVIOR_MANUAL_OPERATION, // 手动作业
} engineer_behavior_e;

typedef struct EngineerBehaviorManager
{
    /* 基本 */

    engineer_behavior_e behavior;
    engineer_behavior_e last_behavior;
    bool behavior_changed; // 状态改变时置为真 外部读取后置为假

    /* 获取其他模块的状态 */

    bool *arm_reset_success;
    bool *arm_move_homing_success;
    bool *arm_operation_homing_success;
    bool *chassis_move_homing_successfully;
    bool *chassis_operation_homing_successfully;

    /* 设备 */

    const remote_control_s *rc; // 遥控器数据
    char dt7_behavior_switch_value;
    char last_dt7_behavior_switch_value;
    int16_t dt7_reset_trigger_value;   // 使用DT7复位的触发器值
    uint32_t dt7_reset_trigger_timer;  // 使用DT7复位的触发计时器
    uint32_t km_disable_trigger_timer; // 使用键鼠失能的触发计时器
    uint32_t km_reset_trigger_timer;   // 使用键鼠复位的触发计时器
    uint32_t km_switch_trigger_timer;  // 使用键鼠切换机动/作业行为的触发计时器
} engineer_behavior_manager_s;

extern void behavior_task(void *pvParameters);
extern bool checkIfEngineerBehaviorChanged(void);
extern engineer_behavior_e getEngineerCurrentBehavior(void);
extern engineer_behavior_e getEngineerLastBehavior(void);
extern const engineer_behavior_manager_s *getEngineerBehaviorManagerPointer(void);

#endif /* _BEHAVIOR_TASK_H__ */
