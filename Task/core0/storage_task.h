#ifndef _STORAGE_TASK_H__
#define _STORAGE_TASK_H__

#include "stdbool.h"
#include "stdint.h"

#define STORAGE_MAX_LIMIT 2

typedef enum EngineerStorageSlotIndex
{
    STORAGE_BACK = 0,
    STORAGE_FRONT,
    STORAGE_NULL,
} engineer_storage_slot_index_e;

typedef enum EngineerStorageStatus
{
    STORAGE_EMPTY = 0,
    STORAGE_AVAILABLE,
    STORAGE_FULL,
} engineer_storage_status_e;

typedef enum EngineerStorageSlotStatus
{
    STORAGE_SLOT_EMPTY = 0,
    STORAGE_SLOT_USED,
} engineer_storage_slot_status_e;

typedef struct EngineerStorage
{
    engineer_storage_slot_index_e current_target_slot;

    uint8_t storage_used_num;
    engineer_storage_slot_status_e storage_slot_status[STORAGE_MAX_LIMIT];
    engineer_storage_slot_status_e last_storage_slot_status[STORAGE_MAX_LIMIT];
    uint8_t empty_detect_timer[STORAGE_MAX_LIMIT];
    uint8_t used_detect_timer[STORAGE_MAX_LIMIT];

    bool storage_slot_needed[STORAGE_MAX_LIMIT];

} engineer_storage_s;

extern void storage_task(void *pvParameters);
extern const engineer_storage_s *getStorageDataPointer(void);

extern engineer_storage_status_e getStorageStatus(void);
extern engineer_storage_slot_status_e getStorageSlotStatus(engineer_storage_slot_index_e slot_index);

extern engineer_storage_slot_index_e getStorageCurrentTargetSlot(void);

/**
 * @brief 获取可存矿石槽位 若无可存槽位则返回空槽位
 * @note 若成功获取到可存槽位 则会自动打开此槽位的吸取功能
 *
 * @return engineer_storage_slot_index_e 存矿位置
 */
engineer_storage_slot_index_e getStoragePushInAvailableSlot(void);
/**
 * @brief 取消此次矿石存入
 */
void StorageCancelPushIn(void);
/**
 * @brief 获取可取矿石槽位 若无可取矿石则返回空槽位
 *
 * @return engineer_storage_slot_index_e 取矿位置
 */
engineer_storage_slot_index_e getStoragePopOutAvailableSlot(void);
/**
 * @brief 确认取出当前矿石
 */
void StorageConfirmPopOut(void);

// 气泵&电磁阀端口定义
#define ENGINEER_STORAGE_PUMP_GPIO_PORT (GPIO_DO_GPIOF)
#define ENGINEER_STORAGE_PUMP_GPIO_PIN (1)

#define ENGINEER_STORAGE_FRONT_POWER_VALVE_GPIO_PORT (GPIO_DO_GPIOA)
#define ENGINEER_STORAGE_FRONT_POWER_VALVE_GPIO_PIN (25)
#define ENGINEER_STORAGE_FRONT_RELIEF_VALVE_GPIO_PORT (GPIO_DO_GPIOA)
#define ENGINEER_STORAGE_FRONT_RELIEF_VALVE_GPIO_PIN (24)
#define ENGINEER_STORAGE_FRONT_SENSOR_GPIO_PORT (GPIO_DO_GPIOA)
#define ENGINEER_STORAGE_FRONT_SENSOR_GPIO_PIN (6)

#define ENGINEER_STORAGE_BACK_POWER_VALVE_GPIO_PORT (GPIO_DO_GPIOB)
#define ENGINEER_STORAGE_BACK_POWER_VALVE_GPIO_PIN (2)
#define ENGINEER_STORAGE_BACK_RELIEF_VALVE_GPIO_PORT (GPIO_DO_GPIOB)
#define ENGINEER_STORAGE_BACK_RELIEF_VALVE_GPIO_PIN (1)
#define ENGINEER_STORAGE_BACK_SENSOR_GPIO_PORT (GPIO_DO_GPIOA)
#define ENGINEER_STORAGE_BACK_SENSOR_GPIO_PIN (11)
// #define ENGINEER_STORAGE_BACK_SENSOR_GPIO_PORT (GPIO_DO_GPIOC)
// #define ENGINEER_STORAGE_BACK_SENSOR_GPIO_PIN (0)

#endif /* _STORAGE_TASK_H__ */
