#include "behavior_task.h"

#include "board.h"
#include "hpm_ppor_drv.h"

#include "FreeRTOS.h"
#include "task.h"

#include "referee.h"

#include "INS_task.h"
#include "arm_task.h"
#include "gimbal_task.h"

engineer_behavior_manager_s behavior_manager;

static void behavior_manager_init(engineer_behavior_manager_s *behavior_manager);
static void update_robot_status(engineer_behavior_manager_s *behavior_manager);
static void update_behavior(engineer_behavior_manager_s *behavior_manager, engineer_behavior_e new_behavior);
static void operator_manual_operation(engineer_behavior_manager_s *behavior_manager);
static void auto_operation(engineer_behavior_manager_s *behavior_manager);
static void module_operation(engineer_behavior_manager_s *behavior_manager);

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

        module_operation(&behavior_manager);

        vTaskDelay(10);
    }
}

const engineer_behavior_manager_s *getEngineerBehaviorManagerPointer(void)
{
    return &behavior_manager;
}

engineer_behavior_e getEngineerCurrentBehavior(void)
{
    return behavior_manager.behavior;
}

engineer_behavior_e getEngineerLastBehavior(void)
{
    return behavior_manager.last_behavior;
}

void setArmGrabMode(bool enable)
{
    behavior_manager.arm_grab = enable;
}

bool checkIfArmNeedSwitchSolution(void)
{
    if (behavior_manager.arm_switch_solution)
        return behavior_manager.arm_switch_solution = false, true;

    return false;
}

bool getArmGrabMode(void)
{
    return behavior_manager.arm_grab;
}

bool checkIfNeedResetUi(void)
{
    if (behavior_manager.reset_ui)
        return behavior_manager.reset_ui = false, true;

    return false;
}

uint8_t getUiSlot(void)
{
    return behavior_manager.ui_slot;
}

static void behavior_manager_init(engineer_behavior_manager_s *behavior_manager)
{
    behavior_manager->rc = getRemoteControlPointer();

    behavior_manager->behavior = ENGINEER_BEHAVIOR_DISABLE;
    behavior_manager->last_behavior = ENGINEER_BEHAVIOR_DISABLE;

    behavior_manager->robot_survival_status = false;

    behavior_manager->arm_reset_success = getArmResetStatus();
    behavior_manager->arm_move_homing_success = getArmMoveHomingStatue();
    behavior_manager->arm_operation_homing_success = getArmOperationHomingStatus();
    behavior_manager->silver_mining_success = getSilverMiningStatus();
    behavior_manager->gimbal_reset_success = getGimbalResetStatus();

    behavior_manager->arm_grab = false;
    behavior_manager->arm_switch_solution = false;
    behavior_manager->reset_ui = false;
    behavior_manager->ui_slot = 0;
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
            *behavior_manager->gimbal_reset_success = false;
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_RESET);
            board_write_led_b(LED_ON);
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
    if (*behavior_manager->arm_reset_success && *behavior_manager->gimbal_reset_success)
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
    if (checkIsRcKeyPressed(RC_X) && behavior_manager->behavior == ENGINEER_BEHAVIOR_DISABLE)
    {
        behavior_manager->km_reset_trigger_timer++;
        if (behavior_manager->km_reset_trigger_timer == 50)
        {
            *behavior_manager->arm_reset_success = false;
            *behavior_manager->gimbal_reset_success = false;
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_RESET);
            board_write_led_b(LED_ON);
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
    if (checkIfRcKeyFallingEdgeDetected(RC_G) && *behavior_manager->arm_reset_success &&
        *behavior_manager->gimbal_reset_success)
    {
        if (behavior_manager->behavior == ENGINEER_BEHAVIOR_DISABLE ||
            behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING);
        else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_DISABLE ||
                 behavior_manager->behavior == ENGINEER_BEHAVIOR_MOVE)
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING);
    }

    /**
     * @brief 手动作业 -> 自动取银矿
     * 键鼠 短按Z键触发切换
     */
    if (checkIfRcKeyFallingEdgeDetected(RC_Z) && behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
    {
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_SILVER_MINING);
    }
}

static void auto_operation(engineer_behavior_manager_s *behavior_manager)
{
    if (behavior_manager->behavior == ENGINEER_BEHAVIOR_RESET && *behavior_manager->arm_reset_success &&
        *behavior_manager->gimbal_reset_success)
    {
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING);
        board_write_led_b(LED_OFF);
    }
    else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING &&
             *behavior_manager->arm_move_homing_success)
    {
        *behavior_manager->arm_move_homing_success = false;
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_MOVE);
    }
    else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING &&
             *behavior_manager->arm_operation_homing_success)
    {
        *behavior_manager->arm_operation_homing_success = false;
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_MANUAL_OPERATION);
    }

    if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_SILVER_MINING && *behavior_manager->silver_mining_success)
    {
        *behavior_manager->silver_mining_success = false;
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_MANUAL_OPERATION);
    }

    if (behavior_manager->robot_survival_status == false && behavior_manager->last_robot_survival_status == true)
    {
        *behavior_manager->arm_reset_success = false;
        *behavior_manager->gimbal_reset_success = false;
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_DISABLE);
        ppor_sw_reset(HPM_PPOR, 10);
    }
    else if (behavior_manager->robot_survival_status == false)
    {
        board_write_led_r(LED_ON);
    }
    else if (behavior_manager->robot_survival_status == true)
    {
        board_write_led_r(LED_OFF);
    }
}

static void module_operation(engineer_behavior_manager_s *behavior_manager)
{
    /**
     * @brief 切换机械臂解算
     * DT7 长拨拨轮触发切换
     */
    behavior_manager->dt7_arm_switch_trigger_value = behavior_manager->rc->dt7_dr16_data.rc.ch[4];
    if (behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION &&
        behavior_manager->dt7_arm_switch_trigger_value > 600)
    {
        behavior_manager->dt7_arm_switch_trigger_timer++;
        if (behavior_manager->dt7_arm_switch_trigger_timer == 10)
        {
            behavior_manager->arm_switch_solution = true;
        }
    }
    else
        behavior_manager->dt7_arm_switch_trigger_timer = 0;
    /**
     * @brief 切换机械臂解算
     * 键鼠 短按Q键触发切换
     */
    if (checkIfRcKeyFallingEdgeDetected(RC_Q) && behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
    {
        behavior_manager->arm_switch_solution = true;
    }

    /**
     * @brief 机械臂吸取工作模式切换
     * DT7 长拨拨轮触发切换
     */
    behavior_manager->dt7_arm_grab_trigger_value = behavior_manager->rc->dt7_dr16_data.rc.ch[4];
    if (behavior_manager->dt7_arm_grab_trigger_value < -600)
    {
        behavior_manager->dt7_arm_grab_trigger_timer++;
        if (behavior_manager->dt7_arm_grab_trigger_timer == 20)
        {
            behavior_manager->arm_grab = !behavior_manager->arm_grab;
        }
    }
    else
        behavior_manager->dt7_arm_grab_trigger_timer = 0;
    /**
     * @brief 机械臂吸取工作模式切换
     * 键鼠 短按R键触发切换
     */
    if (checkIfRcKeyFallingEdgeDetected(RC_R))
    {
        behavior_manager->arm_grab = !behavior_manager->arm_grab;
    }

    /**
     * @brief 重置UI
     * 键鼠 按下X键触发
     */
    if (checkIsRcKeyPressed(RC_X))
    {
        behavior_manager->reset_ui = true;
    }

    /**
     * @brief UI切换
     * 键鼠 按下E键触发切换
     */
    if (checkIfRcKeyFallingEdgeDetected(RC_E) && behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
    {
        behavior_manager->ui_slot = behavior_manager->ui_slot >= 1 ? 0 : behavior_manager->ui_slot + 1;
    }
    else if (behavior_manager->behavior != ENGINEER_BEHAVIOR_MANUAL_OPERATION)
    {
        behavior_manager->ui_slot = 0;
    }

    /**
     * @brief 重启
     * 键鼠 长按B键触发
     */
    if (checkIsRcKeyPressed(RC_B))
    {
        behavior_manager->km_reboot_trigger_timer++;
        if (behavior_manager->km_reboot_trigger_timer == 30)
        {
            ppor_sw_reset(HPM_PPOR, 10);
        }
    }
    else
        behavior_manager->km_reboot_trigger_timer = 0;
}
