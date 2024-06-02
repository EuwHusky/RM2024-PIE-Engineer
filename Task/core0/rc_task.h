#ifndef _RC_TASK_H__
#define _RC_TASK_H__

#include "referee.h"

#define DBUS_RX_BUF_NUM 36u // 遥控器数据接受数组大小

extern void rc_task(void *pvParameters);

extern const custom_robot_data_t *getCcData(void);
extern const vt_link_remote_control_t *getVtRcData(void);

#endif /* _RC_TASK_H__ */
