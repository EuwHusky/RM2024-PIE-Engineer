#ifndef _STORAGE_TASK_H__
#define _STORAGE_TASK_H__

#include "stdbool.h"
#include "stdint.h"

#define STORAGE_MAX_LIMIT 2

enum EngineerStorageIndex
{
    STORAGE_BACK = 0,
    STORAGE_FRONT,
};

typedef enum EngineerStorageStatus
{
    STORAGE_EMPTY = 0,
    STORAGE_AVAILABLE,
    STORAGE_FULL,
} engineer_storage_status_s;

typedef struct EngineerStorage
{
    uint8_t storage_used_num;
    bool storage_status[STORAGE_MAX_LIMIT]; // 0-无矿 1-有矿

} engineer_storage_s;

extern void storage_task(void *pvParameters);
extern const engineer_storage_s *getStorageDataPointer(void);

extern engineer_storage_status_s getStorageStatus(void);

#endif /* _STORAGE_TASK_H__ */
