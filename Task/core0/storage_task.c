#include "storage_task.h"

#include "drv_delay.h"

#include "INS_task.h"
#include "behavior_task.h"

#define USED_DETECT_TIMER_THRESHOLD_VALUE (10)
#define EMPTY_DETECT_TIMER_THRESHOLD_VALUE (10)

static void storage_init(engineer_storage_s *storage);
static void storage_update(engineer_storage_s *storage);
static void storage_execute(engineer_storage_s *storage);

static engineer_storage_s storage;

void storage_task(void *pvParameters)
{
    rflOsDelayMs(1000);

    while (!INS_init_finished)
        rflOsDelayMs(10);
    rflOsDelayMs(1000);

    storage_init(&storage);

    while (1)
    {
        storage_update(&storage);

        storage_execute(&storage);

        rflOsDelayMs(5);
    }
}

const engineer_storage_s *getStorageDataPointer(void)
{
    return &storage;
}

engineer_storage_status_e getStorageStatus(void)
{
    if (storage.storage_used_num == 0)
        return STORAGE_EMPTY;
    if (storage.storage_slot_status[STORAGE_MAX_LIMIT - 1] == STORAGE_SLOT_USED)
        return STORAGE_FULL;

    return STORAGE_AVAILABLE;
}

engineer_storage_slot_index_e getStorageCurrentTargetSlot(void)
{
    return storage.current_target_slot;
}

engineer_storage_slot_status_e getStorageSlotStatus(engineer_storage_slot_index_e slot_index)
{
    return storage.storage_slot_status[slot_index];
}

engineer_storage_nugget_type_e getStorageSlotNuggetType(engineer_storage_slot_index_e slot_index)
{
    return storage.slot_nugget_type[slot_index];
}

engineer_storage_nugget_type_e getLatestNuggetTypeToGrab(void)
{
    return storage.latest_nugget_type_to_grab;
}

void setGrabNuggetType(engineer_storage_nugget_type_e nugget_type)
{
    storage.latest_nugget_type_to_grab = nugget_type;
}

/**
 * @brief 获取可存矿石槽位 若无可存槽位则返回空槽位
 * @note 若成功获取到可存槽位 则会自动打开此槽位的吸取功能
 *
 * @return engineer_storage_slot_index_e 存矿位置
 */
engineer_storage_slot_index_e getStoragePushInAvailableSlot(void)
{
    if (storage.storage_slot_status[STORAGE_MAX_LIMIT - 1] == STORAGE_SLOT_USED)
        return STORAGE_NULL;

    uint8_t use_slot_index;

    for (uint8_t i = 0; i < STORAGE_MAX_LIMIT; i++)
    {
        if (storage.storage_slot_status[i] == STORAGE_SLOT_EMPTY)
        {
            // 堆顶
            if (i == (STORAGE_MAX_LIMIT - 1))
            {
                use_slot_index = i;
                break;
            }

            // 堆中
            bool available = true;
            for (uint8_t j = i + 1; j < STORAGE_MAX_LIMIT; j++)
                if (storage.storage_slot_status[j] != STORAGE_SLOT_EMPTY)
                {
                    available = false;
                    break;
                }
            if (available)
            {
                use_slot_index = i;
                break;
            }
        }
    }

    storage.current_target_slot = use_slot_index;

    return storage.current_target_slot;
}

/**
 * @brief 获取可取矿石槽位 若无可取矿石则返回空槽位
 *
 * @return engineer_storage_slot_index_e 取矿位置
 */
engineer_storage_slot_index_e getStoragePopOutAvailableSlot(void)
{
    if (storage.storage_used_num == 0)
        return STORAGE_NULL;

    for (uint8_t i = STORAGE_MAX_LIMIT, j = STORAGE_MAX_LIMIT - 1; i > 0; i--, j--)
        if (storage.storage_slot_status[j] == STORAGE_SLOT_USED)
        {
            storage.current_target_slot = j;
            break;
        }

    return storage.current_target_slot;
}

/**
 * @brief 对当前操作进行确认
 * @note 当前为存入/取出操作，则打开/关闭对应槽位
 */
void StorageConfirmOperation(engineer_storage_operation_e operation)
{
    if (operation == STORAGE_PUSH_IN)
    {
        storage.slot_nugget_type[storage.current_target_slot] = storage.latest_nugget_type_to_grab;
        storage.storage_slot_needed[storage.current_target_slot] = true;
    }
    else if (operation == STORAGE_POP_OUT)
    {
        storage.latest_nugget_type_to_grab = storage.slot_nugget_type[storage.current_target_slot];
        storage.storage_slot_needed[storage.current_target_slot] = false;
    }
}

/**
 * @brief 取消当前操作
 * @note 当前为存入操作，则关闭对应槽位
 */
void StorageCancelOperation(engineer_storage_operation_e operation)
{
    if (operation == STORAGE_PUSH_IN)
        storage.storage_slot_needed[storage.current_target_slot] = false;
}

static void storage_init(engineer_storage_s *storage)
{
    memset(storage, 0, sizeof(engineer_storage_s));

    storage->current_target_slot = STORAGE_BACK;

    for (uint8_t i = 0; i < STORAGE_MAX_LIMIT; i++)
    {
        storage->storage_slot_status[i] = STORAGE_SLOT_EMPTY;
        storage->slot_nugget_type[i] = SILVER_NUGGET;
        storage->storage_slot_needed[i] = false;
        storage->empty_detect_timer[i] = 0;
        storage->used_detect_timer[i] = 0;
    }
    storage->latest_nugget_type_to_grab = SILVER_NUGGET;

    storage->gpio_port[STORAGE_BACK][ENGINEER_STORAGE_POWER] = ENGINEER_STORAGE_BACK_PUMP_GPIO_PORT;
    storage->gpio_port[STORAGE_BACK][ENGINEER_STORAGE_RELIEF] = ENGINEER_STORAGE_BACK_RELIEF_VALVE_GPIO_PORT;
    storage->gpio_port[STORAGE_BACK][ENGINEER_STORAGE_SENSOR] = ENGINEER_STORAGE_BACK_SENSOR_GPIO_PORT;

    storage->gpio_pin[STORAGE_BACK][ENGINEER_STORAGE_POWER] = ENGINEER_STORAGE_BACK_PUMP_GPIO_PIN;
    storage->gpio_pin[STORAGE_BACK][ENGINEER_STORAGE_RELIEF] = ENGINEER_STORAGE_BACK_RELIEF_VALVE_GPIO_PIN;
    storage->gpio_pin[STORAGE_BACK][ENGINEER_STORAGE_SENSOR] = ENGINEER_STORAGE_BACK_SENSOR_GPIO_PIN;

    storage->gpio_port[STORAGE_FRONT][ENGINEER_STORAGE_POWER] = ENGINEER_STORAGE_FRONT_PUMP_GPIO_PORT;
    storage->gpio_port[STORAGE_FRONT][ENGINEER_STORAGE_RELIEF] = ENGINEER_STORAGE_FRONT_RELIEF_VALVE_GPIO_PORT;
    storage->gpio_port[STORAGE_FRONT][ENGINEER_STORAGE_SENSOR] = ENGINEER_STORAGE_FRONT_SENSOR_GPIO_PORT;

    storage->gpio_pin[STORAGE_FRONT][ENGINEER_STORAGE_POWER] = ENGINEER_STORAGE_FRONT_PUMP_GPIO_PIN;
    storage->gpio_pin[STORAGE_FRONT][ENGINEER_STORAGE_RELIEF] = ENGINEER_STORAGE_FRONT_RELIEF_VALVE_GPIO_PIN;
    storage->gpio_pin[STORAGE_FRONT][ENGINEER_STORAGE_SENSOR] = ENGINEER_STORAGE_FRONT_SENSOR_GPIO_PIN;

    // 后储矿气泵
    HPM_IOC->PAD[IOC_PAD_PF00].FUNC_CTL = IOC_PF00_FUNC_CTL_GPIO_F_00;
    gpio_set_pin_output_with_initial(ENGINEER_STORAGE_GPIO, ENGINEER_STORAGE_BACK_PUMP_GPIO_PORT,
                                     ENGINEER_STORAGE_BACK_PUMP_GPIO_PIN, 0);

    // 后储矿卸力阀
    HPM_IOC->PAD[IOC_PAD_PA24].FUNC_CTL = IOC_PA24_FUNC_CTL_GPIO_A_24;
    gpio_set_pin_output_with_initial(ENGINEER_STORAGE_GPIO, ENGINEER_STORAGE_BACK_RELIEF_VALVE_GPIO_PORT,
                                     ENGINEER_STORAGE_BACK_RELIEF_VALVE_GPIO_PIN, 1);

    // 后储矿气压传感器
    HPM_IOC->PAD[IOC_PAD_PA11].FUNC_CTL = IOC_PA11_FUNC_CTL_GPIO_A_11;
    gpio_set_pin_input(ENGINEER_STORAGE_GPIO, ENGINEER_STORAGE_BACK_SENSOR_GPIO_PORT,
                       ENGINEER_STORAGE_BACK_SENSOR_GPIO_PIN);

    // 前储矿气泵
    HPM_IOC->PAD[IOC_PAD_PF01].FUNC_CTL = IOC_PF01_FUNC_CTL_GPIO_F_01;
    gpio_set_pin_output_with_initial(ENGINEER_STORAGE_GPIO, ENGINEER_STORAGE_FRONT_PUMP_GPIO_PORT,
                                     ENGINEER_STORAGE_FRONT_PUMP_GPIO_PIN, 0);

    // 前储矿卸力阀
    HPM_IOC->PAD[IOC_PAD_PB01].FUNC_CTL = IOC_PB01_FUNC_CTL_GPIO_B_01;
    gpio_set_pin_output_with_initial(ENGINEER_STORAGE_GPIO, ENGINEER_STORAGE_FRONT_RELIEF_VALVE_GPIO_PORT,
                                     ENGINEER_STORAGE_FRONT_RELIEF_VALVE_GPIO_PIN, 0);

    // 前储矿气压传感器
    HPM_IOC->PAD[IOC_PAD_PA06].FUNC_CTL = IOC_PA06_FUNC_CTL_GPIO_A_06;
    gpio_set_pin_input(ENGINEER_STORAGE_GPIO, ENGINEER_STORAGE_FRONT_SENSOR_GPIO_PORT,
                       ENGINEER_STORAGE_FRONT_SENSOR_GPIO_PIN);
}

static void storage_update(engineer_storage_s *storage)
{
    // 更新槽位矿石状态
    for (uint8_t i = 0; i < STORAGE_MAX_LIMIT; i++)
    {
        storage->last_storage_slot_status[i] = storage->storage_slot_status[i];

        if (storage->storage_slot_status[i] == STORAGE_SLOT_EMPTY &&
            gpio_read_pin(ENGINEER_STORAGE_GPIO, storage->gpio_port[i][ENGINEER_STORAGE_SENSOR],
                          storage->gpio_pin[i][ENGINEER_STORAGE_SENSOR]) == 0)
        {
            storage->used_detect_timer[i]++;
            if (storage->used_detect_timer[i] >= USED_DETECT_TIMER_THRESHOLD_VALUE)
                storage->storage_slot_status[i] = STORAGE_SLOT_USED;
        }
        else
        {
            storage->used_detect_timer[i] = 0;
        }

        if (storage->storage_slot_status[i] == STORAGE_SLOT_USED &&
            gpio_read_pin(ENGINEER_STORAGE_GPIO, storage->gpio_port[i][ENGINEER_STORAGE_SENSOR],
                          storage->gpio_pin[i][ENGINEER_STORAGE_SENSOR]) == 1)
        {
            storage->empty_detect_timer[i]++;
            if (storage->empty_detect_timer[i] >= EMPTY_DETECT_TIMER_THRESHOLD_VALUE)
                storage->storage_slot_status[i] = STORAGE_SLOT_EMPTY;
        }
        else
        {
            storage->empty_detect_timer[i] = 0;
        }
    }

    // 判断矿石是否意外掉落 若是则关闭该槽位的吸取

    for (uint8_t i = 0; i < STORAGE_MAX_LIMIT; i++)
        if (storage->last_storage_slot_status[i] == STORAGE_SLOT_USED &&
            storage->storage_slot_status[i] == STORAGE_SLOT_EMPTY)
            storage->storage_slot_needed[i] = false;

    // 失能时关闭并重置储矿功能

    if (getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_DISABLE)
        for (uint8_t i = 0; i < STORAGE_MAX_LIMIT; i++)
        {
            storage->storage_slot_status[i] = STORAGE_SLOT_EMPTY;
            storage->storage_slot_needed[i] = false;
            storage->empty_detect_timer[i] = 0;
            storage->used_detect_timer[i] = 0;
        }

    // 计算已使用槽位数量

    storage->storage_used_num = 0;
    for (uint8_t i = 0; i < STORAGE_MAX_LIMIT; i++)
        if (storage->storage_slot_status[i] == STORAGE_SLOT_USED)
            storage->storage_used_num++;
}

static void storage_execute(engineer_storage_s *storage)
{
    for (uint8_t i = 0; i < STORAGE_MAX_LIMIT; i++)
    {
        // // 气泵
        // gpio_write_pin(HPM_GPIO0, storage->gpio_port[i][ENGINEER_STORAGE_POWER],
        //                storage->gpio_pin[i][ENGINEER_STORAGE_POWER],
        //                getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_DISABLE ? 0 : 1);

        // // 卸力阀
        // gpio_write_pin(ENGINEER_STORAGE_GPIO, storage->gpio_port[i][ENGINEER_STORAGE_RELIEF],
        //                storage->gpio_pin[i][ENGINEER_STORAGE_RELIEF],
        //                getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_DISABLE ? 1 : 0);

        // 气泵
        gpio_write_pin(HPM_GPIO0, storage->gpio_port[i][ENGINEER_STORAGE_POWER],
                       storage->gpio_pin[i][ENGINEER_STORAGE_POWER], storage->storage_slot_needed[i] ? 1 : 0);

        // 卸力阀
        gpio_write_pin(ENGINEER_STORAGE_GPIO, storage->gpio_port[i][ENGINEER_STORAGE_RELIEF],
                       storage->gpio_pin[i][ENGINEER_STORAGE_RELIEF], storage->storage_slot_needed[i] ? 0 : 1);
    }
}
