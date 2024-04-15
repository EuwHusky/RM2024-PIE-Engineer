#include "behavior_task.h"

#include "board.h"

#include "FreeRTOS.h"
#include "task.h"

#include "referee.h"

#include "INS_task.h"
#include "arm_task.h"

engineer_behavior_manager_s behavior_manager;

static void behavior_manager_init(engineer_behavior_manager_s *behavior_manager);
static void update_robot_status(engineer_behavior_manager_s *behavior_manager);
static void update_behavior(engineer_behavior_manager_s *behavior_manager, engineer_behavior_e new_behavior);
static void operator_manual_operation(engineer_behavior_manager_s *behavior_manager);
static void auto_operation(engineer_behavior_manager_s *behavior_manager);

void behavior_task(void *pvParameters)
{
    vTaskDelay(1200);

    while (!INS_init_finished)
        vTaskDelay(2);
    vTaskDelay(10);

    behavior_manager_init(&behavior_manager);

    while (1)
    {
        update_robot_status(&behavior_manager);

        operator_manual_operation(&behavior_manager);
        auto_operation(&behavior_manager);

        vTaskDelay(10);
    }
}

bool checkIfEngineerBehaviorChanged(void)
{
    if (behavior_manager.behavior_changed)
        return behavior_manager.behavior_changed = false, true;

    return false;
}

engineer_behavior_e getEngineerCurrentBehavior(void)
{
    return behavior_manager.behavior;
}

engineer_behavior_e getEngineerLastBehavior(void)
{
    return behavior_manager.last_behavior;
}

const engineer_behavior_manager_s *getEngineerBehaviorManagerPointer(void)
{
    return &behavior_manager;
}

static void behavior_manager_init(engineer_behavior_manager_s *behavior_manager)
{
    behavior_manager->rc = getRemoteControlPointer();

    behavior_manager->behavior = ENGINEER_BEHAVIOR_DISABLE;
    behavior_manager->last_behavior = ENGINEER_BEHAVIOR_DISABLE;
    behavior_manager->behavior_changed = false;

    behavior_manager->arm_reset_success = getArmResetStatus();
    behavior_manager->arm_move_homing_success = getArmMoveHomingStatue();
    behavior_manager->arm_operation_homing_success = getArmOperationHomingStatus();
}

static void update_robot_status(engineer_behavior_manager_s *behavior_manager)
{
    behavior_manager->last_robot_survival_status = behavior_manager->robot_survival_status;
    behavior_manager->robot_survival_status = (getRobotCurrentHp() == 0) ? false : true;
}

static void update_behavior(engineer_behavior_manager_s *behavior_manager, engineer_behavior_e new_behavior)
{
    behavior_manager->last_behavior = behavior_manager->behavior;
    behavior_manager->behavior = new_behavior;
    behavior_manager->behavior_changed = true;
}

static void operator_manual_operation(engineer_behavior_manager_s *behavior_manager)
{
    // DT7操作

    behavior_manager->last_dt7_behavior_switch_value = behavior_manager->dt7_behavior_switch_value;
    behavior_manager->dt7_behavior_switch_value = behavior_manager->rc->dt7_dr16_data.rc.s[1];

    /**
     * @brief 失能 -> 复位
     * DT7 长拨拨轮触发
     */
    behavior_manager->dt7_reset_trigger_value = behavior_manager->rc->dt7_dr16_data.rc.ch[4];
    if (behavior_manager->behavior == ENGINEER_BEHAVIOR_DISABLE && behavior_manager->dt7_reset_trigger_value > 600)
    {
        behavior_manager->dt7_reset_trigger_timer++;
        if (behavior_manager->dt7_reset_trigger_timer == 20)
        {
            *behavior_manager->arm_reset_success = false;
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_RESET);
        }
    }
    else
        behavior_manager->dt7_reset_trigger_timer = 0;

    /**
     * @brief Any -> 失能
     * DT7 拨拨杆触发
     */
    if (behavior_manager->last_dt7_behavior_switch_value != ENGINEER_DISABLE_DT7_SWITCH_VALUE &&
        behavior_manager->dt7_behavior_switch_value == ENGINEER_DISABLE_DT7_SWITCH_VALUE)
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_DISABLE);

    /**
     * @brief 机动 -> 作业 / 作业 -> 机动
     * DT7 拨拨杆触发
     */
    if (*behavior_manager->arm_reset_success)
    {
        if ((behavior_manager->behavior == ENGINEER_BEHAVIOR_DISABLE ||
             (behavior_manager->behavior == ENGINEER_BEHAVIOR_MOVE)) &&
            behavior_manager->last_dt7_behavior_switch_value != ENGINEER_OPERATE_DT7_SWITCH_VALUE &&
            behavior_manager->dt7_behavior_switch_value == ENGINEER_OPERATE_DT7_SWITCH_VALUE)
        {
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING);
        }
        else if ((behavior_manager->behavior == ENGINEER_BEHAVIOR_DISABLE ||
                  behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION) &&
                 behavior_manager->last_dt7_behavior_switch_value != ENGINEER_MOVE_DT7_SWITCH_VALUE &&
                 behavior_manager->dt7_behavior_switch_value == ENGINEER_MOVE_DT7_SWITCH_VALUE)
        {
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING);
        }
    }

    // 键鼠操作

    /**
     * @brief 失能 -> 复位
     * 键鼠 长拨V键触发
     */
    if (behavior_manager->behavior == ENGINEER_BEHAVIOR_DISABLE && checkIsRcKeyPressed(RC_V))
    {
        behavior_manager->km_reset_trigger_timer++;
        if (behavior_manager->km_reset_trigger_timer == 20)
        {
            *behavior_manager->arm_reset_success = false;
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_RESET);
        }
    }
    else
        behavior_manager->km_reset_trigger_timer = 0;

    /**
     * @brief Any -> 失能
     * 键鼠 长按C键触发
     */
    if (checkIsRcKeyPressed(RC_C))
    {
        behavior_manager->km_disable_trigger_timer++;
        if (behavior_manager->km_disable_trigger_timer == 10)
        {
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_DISABLE);
        }
    }
    else
        behavior_manager->km_disable_trigger_timer = 0;

    /**
     * @brief 机动 -> 作业 / 作业 -> 机动
     * 键鼠 短按G键触发切换
     */
    if (*behavior_manager->arm_reset_success && checkIfRcKeyFallingEdgeDetected(RC_G))
    {
        if (behavior_manager->behavior == ENGINEER_BEHAVIOR_DISABLE ||
            behavior_manager->behavior == ENGINEER_BEHAVIOR_MOVE)
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING);
        else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_DISABLE ||
                 behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING);
    }
}

static void auto_operation(engineer_behavior_manager_s *behavior_manager)
{
    if (behavior_manager->behavior == ENGINEER_BEHAVIOR_RESET && *behavior_manager->arm_reset_success)
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING);
    else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING &&
             *behavior_manager->arm_move_homing_success /*  && *behavior_manager->chassis_move_homing_successfully */)
    {
        *behavior_manager->arm_move_homing_success = false;
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_MOVE);
    }
    else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING &&
             *behavior_manager
                  ->arm_operation_homing_success /*  && *behavior_manager->chassis_operation_homing_successfully */)
    {
        *behavior_manager->arm_operation_homing_success = false;
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_MANUAL_OPERATION);
    }

    if (behavior_manager->robot_survival_status == false && behavior_manager->last_robot_survival_status == true)
    {
        board_write_led_r(LED_ON);
        *behavior_manager->arm_reset_success = false;
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_DISABLE);
    }
    else if (behavior_manager->robot_survival_status == true && behavior_manager->last_robot_survival_status == false)
    {
        board_write_led_r(LED_OFF);
    }
}
