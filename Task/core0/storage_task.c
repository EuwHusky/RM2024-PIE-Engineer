#include "storage_task.h"

#include "drv_delay.h"

#include "INS_task.h"

static void storage_init(engineer_storage_s *storage);
static void storage_update(engineer_storage_s *storage);

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

        rflOsDelayMs(5);
    }
}

const engineer_storage_s *getStorageDataPointer(void)
{
    return &storage;
}

engineer_storage_status_s getStorageStatus(void)
{
    if (storage.storage_used_num == 0)
        return STORAGE_EMPTY;
    else if (storage.storage_used_num == STORAGE_MAX_LIMIT)
        return STORAGE_FULL;

    return STORAGE_AVAILABLE;
}

static void storage_init(engineer_storage_s *storage)
{
    memset(storage, 0, sizeof(engineer_storage_s));
}

static void storage_update(engineer_storage_s *storage)
{
    // 读取气压传感器
    for (uint8_t i = 0; i < STORAGE_MAX_LIMIT; i++)
        storage->storage_status[i] = 0;

    storage->storage_used_num = 0;
    for (uint8_t i = 0; i < STORAGE_MAX_LIMIT; i++)
        if (storage->storage_status[i])
            storage->storage_used_num++;
}
