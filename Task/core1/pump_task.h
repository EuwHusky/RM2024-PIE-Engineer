#ifndef _PUMP_TASK_H__
#define _PUMP_TASK_H__

#include "stdint.h"

#define ENGINEER_ARM_PUMP_GPIO_PORT (GPIO_DO_GPIOB)
#define ENGINEER_ARM_PUMP_GPIO_PIN (31)

#define ENGINEER_ARM_VALVE_GPIO_PORT (GPIO_DO_GPIOA)
#define ENGINEER_ARM_VALVE_GPIO_PIN (31)

extern void pump_task(void *pvParameters);

#endif /* _PUMP_TASK_H__ */
