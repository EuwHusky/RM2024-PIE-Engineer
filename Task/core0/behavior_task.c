#include "behavior_task.h"

#include "board.h"
#include "hpm_can_drv.h"
#include "hpm_ppor_drv.h"

#include "drv_delay.h"

#include "referee.h"

#include "INS_task.h"
#include "arm_task.h"
#include "detect_task.h"
#include "gimbal_task.h"
#include "storage_task.h"

engineer_behavior_manager_s behavior_manager;

static void behavior_manager_init(engineer_behavior_manager_s *behavior_manager);
static void update_robot_status(engineer_behavior_manager_s *behavior_manager);
static void update_behavior(engineer_behavior_manager_s *behavior_manager, engineer_behavior_e new_behavior);
static void operator_manual_operation(engineer_behavior_manager_s *behavior_manager);
static void auto_operation(engineer_behavior_manager_s *behavior_manager);
static void status_display(engineer_behavior_manager_s *behavior_manager);

void behavior_task(void *pvParameters)
{
    rflOsDelayMs(1200);

    while (!INS_init_finished)
        rflOsDelayMs(2);
    rflOsDelayMs(10);

    behavior_manager_init(&behavior_manager);

    while (1)
    {
        update_robot_status(&behavior_manager);

        operator_manual_operation(&behavior_manager);
        auto_operation(&behavior_manager);

        status_display(&behavior_manager);

        rflOsDelayMs(10);
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

bool getArmGrabMode(void)
{
    return behavior_manager.arm_grab;
}

bool checkIfArmNeedSwitchSolution(void)
{
    if (behavior_manager.arm_switch_solution)
        return behavior_manager.arm_switch_solution = false, true;

    return false;
}

bool checkIfNeedResetUi(void)
{
    if (behavior_manager.reset_ui)
        return behavior_manager.reset_ui = false, true;

    return false;
}

bool checkIfNeedRebootCore(void)
{
    return behavior_manager.need_reboot;
}

engineer_visual_aid_ui_type_e getVisualAidUi(void)
{
    return behavior_manager.visual_aid_ui;
}

bool checkIfMotorFailureDetected(void)
{
    return behavior_manager.motor_failure_detected;
}

static void behavior_manager_init(engineer_behavior_manager_s *behavior_manager)
{
    memset(behavior_manager, 0, sizeof(engineer_behavior_manager_s));

    behavior_manager->rc = getRemoteControlPointer();

    behavior_manager->behavior = ENGINEER_BEHAVIOR_DISABLE;
    behavior_manager->last_behavior = ENGINEER_BEHAVIOR_DISABLE;

    behavior_manager->robot_survival_status = false;
    behavior_manager->last_robot_survival_status = false;
    behavior_manager->need_reboot = false;
    behavior_manager->reboot_timer = 0;
    behavior_manager->buzzer_beep = false;
    behavior_manager->buzzer_timer = 0;

    behavior_manager->arm_reset_success = getArmResetStatus();
    behavior_manager->arm_move_homing_success = getArmMoveHomingStatue();
    behavior_manager->arm_operation_homing_success = getArmOperationHomingStatus();
    behavior_manager->silver_mining_success = getSilverMiningStatus();
    behavior_manager->storage_push_success = getStoragePushStatus();
    behavior_manager->storage_pop_success = getStoragePopStatus();
    behavior_manager->gimbal_reset_success = getGimbalResetStatus();
    behavior_manager->gimbal_move_homing_success = getGimbalMoveHomingStatus();
    behavior_manager->gimbal_operation_homing_success = getGimbalOperationHomingStatus();

    behavior_manager->arm_grab = false;
    behavior_manager->arm_switch_solution = false;
    behavior_manager->reset_ui = false;
    behavior_manager->visual_aid_ui = VAU_NONE;
    behavior_manager->motor_failure_detected = false;
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

    if (new_behavior == ENGINEER_BEHAVIOR_DISABLE || new_behavior == ENGINEER_BEHAVIOR_MOVE)
        behavior_manager->buzzer_beep = true;
}

static void operator_manual_operation(engineer_behavior_manager_s *behavior_manager)
{
    /* ================================================== DT7 ================================================== */

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
            behavior_manager->motor_failure_detected = false;
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
    {
        if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING)
            *behavior_manager->arm_move_homing_success = false;
        else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING)
            *behavior_manager->arm_operation_homing_success = false;
        else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_SILVER_MINING)
            *behavior_manager->silver_mining_success = false;
        else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH)
        {
            *behavior_manager->storage_push_success = false;
            StorageCancelOperation(STORAGE_PUSH_IN);
        }
        else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_POP)
            *behavior_manager->storage_pop_success = false;

        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_DISABLE);
    }

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

    /* ================================================== 键鼠 ================================================== */

    /**
     * @brief 失能 -> 复位
     * 键鼠 长拨V键触发
     */
    if (checkIsRcKeyPressed(RC_V) && behavior_manager->behavior == ENGINEER_BEHAVIOR_DISABLE)
    {
        behavior_manager->km_reset_trigger_timer++;
        if (behavior_manager->km_reset_trigger_timer == 50)
        {
            *behavior_manager->arm_reset_success = false;
            *behavior_manager->gimbal_reset_success = false;
            behavior_manager->motor_failure_detected = false;
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_RESET);
            board_write_led_b(LED_ON);
        }
    }
    else
        behavior_manager->km_reset_trigger_timer = 0;

    /**
     * @brief Any -> 失能
     * 键鼠 长按G键触发
     */
    if (checkIsRcKeyPressed(RC_G))
    {
        behavior_manager->km_disable_trigger_timer++;
        if (behavior_manager->km_disable_trigger_timer == 10)
        {
            if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING)
                *behavior_manager->arm_move_homing_success = false;
            else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING)
                *behavior_manager->arm_operation_homing_success = false;
            else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_SILVER_MINING)
                *behavior_manager->silver_mining_success = false;
            else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH)
            {
                *behavior_manager->storage_push_success = false;
                StorageCancelOperation(STORAGE_PUSH_IN);
            }
            else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_POP)
                *behavior_manager->storage_pop_success = false;

            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_DISABLE);
        }
    }
    else
        behavior_manager->km_disable_trigger_timer = 0;

    /**
     * @brief 机动 -> 作业 / 作业 -> 机动
     * 键鼠 短按F键触发切换
     */
    if (checkIfRcKeyFallingEdgeDetected(RC_F) && *behavior_manager->arm_reset_success &&
        *behavior_manager->gimbal_reset_success)
    {
        if (behavior_manager->behavior == ENGINEER_BEHAVIOR_DISABLE ||
            behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING);
        else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_DISABLE ||
                 behavior_manager->behavior == ENGINEER_BEHAVIOR_MOVE)
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING);
    }

    if (checkIsRcKeyPressed(RC_CTRL))
    {
        /**
         * @brief 手动作业 -> 自动取银矿
         * 键鼠 CTRL+Z 触发切换
         */
        if (checkIfRcKeyFallingEdgeDetected(RC_Z) && behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_SILVER_MINING);

        /**
         * @brief 手动作业 -> 自动取金矿
         * 键鼠 CTRL+X 触发切换
         */
        if (checkIfRcKeyFallingEdgeDetected(RC_X) && behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_GOLD_MINING);

        /**
         * @brief 立即停止自动取银矿、自动存/取矿 并切换到作业预归位模式
         * 键鼠 Ctrl+C 触发
         */
        if (checkIfRcKeyFallingEdgeDetected(RC_C) &&
            (behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION ||
             behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_SILVER_MINING ||
             behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH ||
             behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_POP))
        {
            if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_SILVER_MINING)
                *behavior_manager->silver_mining_success = false;
            else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH)
            {
                *behavior_manager->storage_push_success = false;
                StorageCancelOperation(STORAGE_PUSH_IN);
            }
            else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_POP)
                *behavior_manager->storage_pop_success = false;

            resetArmPose();

            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING);
        }

        /**
         * @brief 切换视觉辅助UI类型
         * 键鼠 Ctrl+Q 触发切换
         */
        if (checkIfRcKeyFallingEdgeDetected(RC_Q))
        {
            if (behavior_manager->behavior == ENGINEER_BEHAVIOR_MOVE)
            {
                if (behavior_manager->visual_aid_ui == VAU_NONE)
                    behavior_manager->visual_aid_ui = VAU_GOLD_PRE;
                else if (behavior_manager->visual_aid_ui == VAU_GOLD_PRE)
                    behavior_manager->visual_aid_ui = VAU_SILVER_PRE;
                else
                    behavior_manager->visual_aid_ui = VAU_NONE;
            }
            else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
            {
                if (behavior_manager->visual_aid_ui == VAU_NONE)
                    behavior_manager->visual_aid_ui = VAU_SILVER;
                else
                    behavior_manager->visual_aid_ui = VAU_NONE;
            }
        }
        else if (behavior_manager->behavior != ENGINEER_BEHAVIOR_MOVE &&
                 behavior_manager->behavior != ENGINEER_BEHAVIOR_MANUAL_OPERATION)
            behavior_manager->visual_aid_ui = VAU_NONE;

        /**
         * @brief 切换机械臂解算
         * 键鼠 Ctrl+E 触发切换
         */
        if (checkIfRcKeyFallingEdgeDetected(RC_E) && behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
        {
            behavior_manager->arm_switch_solution = true;
        }
    }
    else if (!checkIsRcKeyPressed(RC_CTRL))
    {
        /**
         * @brief 手动作业 -> 自动存矿
         * 键鼠 短按Z键触发切换
         */
        if (checkIfRcKeyFallingEdgeDetected(RC_Z) && getStorageStatus() != STORAGE_FULL && checkIfArmGrabbed() &&
            behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH);

        /**
         * @brief 手动作业 -> 自动取矿
         * 键鼠 短按X键触发切换
         */
        if (checkIfRcKeyFallingEdgeDetected(RC_X) && getStorageStatus() != STORAGE_EMPTY && !checkIfArmGrabbed() &&
            behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_STORAGE_POP);

        /**
         * @brief 立即停止自动取金/银矿、自动存/取矿 切换回手动作业
         * 键鼠 短按C键触发
         */
        if (checkIfRcKeyFallingEdgeDetected(RC_C) &&
            (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_SILVER_MINING ||
             behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_GOLD_MINING ||
             behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH ||
             behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_POP))
        {
            if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_SILVER_MINING)
                *behavior_manager->silver_mining_success = false;
            else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH)
            {
                *behavior_manager->storage_push_success = false;
                StorageCancelOperation(STORAGE_PUSH_IN);
            }
            else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_POP)
                *behavior_manager->storage_pop_success = false;

            resetArmPose();

            update_behavior(behavior_manager, ENGINEER_BEHAVIOR_MANUAL_OPERATION);
        }

        // 清除标志位 防止历史误操作
        if (checkIfRcKeyFallingEdgeDetected(RC_Q))
            ;
        if (checkIfRcKeyFallingEdgeDetected(RC_E))
            ;
    }

    /* ================================================== 下级模块 ================================================== */

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
     * @brief 手动作业 准备兑矿
     * 自定义控制器 短按CC_LEFT/CC_RIGHT键触发切换
     */
    if (checkIfCcKeyFallingEdgeDetected(CC_LEFT) && behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
    {
        ArmReadyToExchangePose(JOINT_3_ON_THE_LEFT);
    }
    if (checkIfCcKeyFallingEdgeDetected(CC_RIGHT) && behavior_manager->behavior == ENGINEER_BEHAVIOR_MANUAL_OPERATION)
    {
        ArmReadyToExchangePose(JOINT_3_ON_THE_RIGHT);
    }

    /**
     * @brief 重置UI
     * 键鼠 按下V键触发
     */
    if (checkIsRcKeyPressed(RC_V))
    {
        behavior_manager->reset_ui = true;
    }

    /**
     * @brief 重启
     * 键鼠 长按B键触发
     */
    if (checkIsRcKeyPressed(RC_B))
    {
        behavior_manager->km_reboot_trigger_timer++;
        if (behavior_manager->km_reboot_trigger_timer > 20)
            behavior_manager->need_reboot = true;
    }
    if (behavior_manager->need_reboot)
    {
        behavior_manager->reboot_timer++;
        if (behavior_manager->reboot_timer > 50)
            ppor_sw_reset(HPM_PPOR, 10);
    }
}

static void auto_operation(engineer_behavior_manager_s *behavior_manager)
{
    /**
     * @brief 复位成功后自动切换到机动模式
     */
    if (behavior_manager->behavior == ENGINEER_BEHAVIOR_RESET && *behavior_manager->arm_reset_success &&
        *behavior_manager->gimbal_reset_success)
    {
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_MOVE);
        board_write_led_b(LED_OFF);
    }

    /**
     * @brief 机动/作业模式互相切换 自动归位
     */
    else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING &&
             *behavior_manager->arm_move_homing_success && *behavior_manager->gimbal_move_homing_success)
    {
        *behavior_manager->arm_move_homing_success = false;
        *behavior_manager->gimbal_move_homing_success = false;
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_MOVE);
    }
    else if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING &&
             *behavior_manager->arm_operation_homing_success && *behavior_manager->gimbal_operation_homing_success)
    {
        *behavior_manager->arm_operation_homing_success = false;
        *behavior_manager->gimbal_operation_homing_success = false;
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_MANUAL_OPERATION);
    }

    /**
     * @brief 自动取银矿成功后自动切换到作业模式
     */
    if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_SILVER_MINING && *behavior_manager->silver_mining_success)
    {
        *behavior_manager->silver_mining_success = false;
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_MANUAL_OPERATION);
    }

    /**
     * @brief 自动存/取矿成功后自动切换到作业预归位模式
     */
    if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_PUSH && *behavior_manager->storage_push_success)
    {
        *behavior_manager->storage_push_success = false;
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING);
    }
    if (behavior_manager->behavior == ENGINEER_BEHAVIOR_AUTO_STORAGE_POP && *behavior_manager->storage_pop_success)
    {
        *behavior_manager->storage_pop_success = false;
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_AUTO_OPERATION_HOMING);
    }

    /**
     * @brief 阵亡自动切换到失能模式
     */
    if (behavior_manager->robot_survival_status == false && behavior_manager->last_robot_survival_status == true)
    {
        *behavior_manager->arm_reset_success = false;
        *behavior_manager->gimbal_reset_success = false;
        update_behavior(behavior_manager, ENGINEER_BEHAVIOR_DISABLE);
    }
    else if (behavior_manager->robot_survival_status == true && behavior_manager->last_robot_survival_status == false)
    {
        can_reset(BOARD_CAN1, true);
        can_reset(BOARD_CAN2, true);
        can_reset(BOARD_CAN3, true);
        can_reset(BOARD_CAN4, true);
        for (uint16_t i = 0, j = 0; i < 60000; i++) // 哈哈傻逼
            j++;
        can_reset(BOARD_CAN1, false);
        can_reset(BOARD_CAN2, false);
        can_reset(BOARD_CAN3, false);
        can_reset(BOARD_CAN4, false);
    }

    /**
     * @brief 电机堵转检测
     */
    if (behavior_manager->behavior != ENGINEER_BEHAVIOR_DISABLE &&
        (detect_error(ARM_JOINT_56_L_DH) || detect_error(ARM_JOINT_56_R_DH)))
    {
        behavior_manager->motor_failure_detect_timer++;
        if (behavior_manager->motor_failure_detect_timer > 3)
        {
            behavior_manager->motor_failure_detected = true;
        }
    }
    else
    {
        behavior_manager->motor_failure_detect_timer = 0;
    }
}

static void status_display(engineer_behavior_manager_s *behavior_manager)
{
    if (behavior_manager->robot_survival_status == false)
    {
        board_write_led_r(LED_ON);
    }
    else if (behavior_manager->robot_survival_status == true)
    {
        board_write_led_r(LED_OFF);
    }

    if (behavior_manager->buzzer_beep)
    {
        board_beep_open();

        behavior_manager->buzzer_timer++;
        if (behavior_manager->buzzer_timer > 8)
            behavior_manager->buzzer_beep = false;
    }
    else
    {
        board_beep_close();

        behavior_manager->buzzer_timer = 0;
    }
}
