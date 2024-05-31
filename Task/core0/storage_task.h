#ifndef _STORAGE_TASK_H__
#define _STORAGE_TASK_H__

#include "stdbool.h"
#include "stdint.h"

#include "behavior_task.h"

#define STORAGE_MAX_LIMIT 2

typedef enum EngineerStorageStatus
{
    STORAGE_EMPTY = 0,
    STORAGE_AVAILABLE,
    STORAGE_FULL,
} engineer_storage_status_e;

typedef enum EngineerStorageSlotIndex
{
    STORAGE_BACK = 0,
    STORAGE_FRONT,
    STORAGE_NULL,
} engineer_storage_slot_index_e;

typedef enum EngineerStorageSlotStatus
{
    STORAGE_SLOT_EMPTY = 0,
    STORAGE_SLOT_USED,
} engineer_storage_slot_status_e;

typedef enum EngineerStorageNuggetType
{
    GOLD_NUGGET = 0,
    SILVER_NUGGET,
} engineer_storage_nugget_type_e;

typedef enum EngineerStorageNuggetToward
{
    NUGGET_FACING_UPWARD = 0,
    NUGGET_FACING_FORWARD,
} engineer_storage_nugget_toward_e;

typedef enum EngineerStorageOperation
{
    STORAGE_PUSH_IN = 0,
    STORAGE_POP_OUT,
} engineer_storage_operation_e;

typedef struct EngineerStorage
{
    engineer_behavior_e behavior;
    engineer_behavior_e last_behavior;

    engineer_storage_slot_index_e current_target_slot;

    uint8_t storage_used_num;
    engineer_storage_slot_status_e storage_slot_status[STORAGE_MAX_LIMIT];
    engineer_storage_slot_status_e last_storage_slot_status[STORAGE_MAX_LIMIT];
    engineer_storage_nugget_type_e latest_nugget_type_to_grab;
    engineer_storage_nugget_type_e slot_nugget_type[STORAGE_MAX_LIMIT];
    engineer_storage_nugget_toward_e slot_nugget_toward[STORAGE_MAX_LIMIT];
    bool storage_slot_needed[STORAGE_MAX_LIMIT];

    uint8_t empty_detect_timer[STORAGE_MAX_LIMIT];
    uint8_t used_detect_timer[STORAGE_MAX_LIMIT];

    uint32_t gpio_port[STORAGE_MAX_LIMIT][3];
    uint8_t gpio_pin[STORAGE_MAX_LIMIT][3];

} engineer_storage_s;

extern void storage_task(void *pvParameters);
extern const engineer_storage_s *getStorageDataPointer(void);

extern engineer_storage_status_e getStorageStatus(void);

extern engineer_storage_slot_index_e getStorageCurrentTargetSlot(void);
extern engineer_storage_slot_status_e getStorageSlotStatus(engineer_storage_slot_index_e slot_index);
extern engineer_storage_nugget_type_e getStorageSlotNuggetType(engineer_storage_slot_index_e slot_index);
extern engineer_storage_nugget_type_e getLatestNuggetTypeToGrab(void);
extern void setGrabNuggetType(engineer_storage_nugget_type_e nugget_type);

extern void setPushInNuggetToward(engineer_storage_slot_index_e slot_index, engineer_storage_nugget_toward_e toward);
extern engineer_storage_nugget_toward_e getNuggetPopOutToward(engineer_storage_slot_index_e slot_index);

/**
 * @brief 获取可存矿石槽位 若无可存槽位则返回空槽位
 * @note 若成功获取到可存槽位 则会自动打开此槽位的吸取功能
 *
 * @return engineer_storage_slot_index_e 存矿位置
 */
extern engineer_storage_slot_index_e getStoragePushInAvailableSlot(void);

/**
 * @brief 获取可取矿石槽位 若无可取矿石则返回空槽位
 *
 * @return engineer_storage_slot_index_e 取矿位置
 */
extern engineer_storage_slot_index_e getStoragePopOutAvailableSlot(void);

/**
 * @brief 对当前操作进行确认
 * @note 当前为存入/取出操作，则打开/关闭对应槽位
 */
extern void StorageConfirmOperation(engineer_storage_operation_e operation);

/**
 * @brief 取消当前操作
 * @note 当前为存入操作，则关闭对应槽位
 */
extern void StorageCancelOperation(engineer_storage_operation_e operation);

#define ENGINEER_STORAGE_POWER (0)
#define ENGINEER_STORAGE_RELIEF (1)
#define ENGINEER_STORAGE_SENSOR (2)

// 气泵&电磁阀&气压传感器端口定义

#define ENGINEER_STORAGE_GPIO (HPM_GPIO0)

#define ENGINEER_STORAGE_BACK_PUMP_GPIO_PORT (GPIO_DO_GPIOF)
#define ENGINEER_STORAGE_BACK_PUMP_GPIO_PIN (0)
#define ENGINEER_STORAGE_BACK_RELIEF_VALVE_GPIO_PORT (GPIO_DO_GPIOB)
#define ENGINEER_STORAGE_BACK_RELIEF_VALVE_GPIO_PIN (1)
#define ENGINEER_STORAGE_BACK_SENSOR_GPIO_PORT (GPIO_DO_GPIOA)
#define ENGINEER_STORAGE_BACK_SENSOR_GPIO_PIN (11)

#define ENGINEER_STORAGE_FRONT_PUMP_GPIO_PORT (GPIO_DO_GPIOF)
#define ENGINEER_STORAGE_FRONT_PUMP_GPIO_PIN (1)
#define ENGINEER_STORAGE_FRONT_RELIEF_VALVE_GPIO_PORT (GPIO_DO_GPIOA)
#define ENGINEER_STORAGE_FRONT_RELIEF_VALVE_GPIO_PIN (24)
#define ENGINEER_STORAGE_FRONT_SENSOR_GPIO_PORT (GPIO_DO_GPIOA)
#define ENGINEER_STORAGE_FRONT_SENSOR_GPIO_PIN (6)

#endif /* _STORAGE_TASK_H__ */
