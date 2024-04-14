#ifndef _REMOTE_CONTROL_H__
#define _REMOTE_CONTROL_H__

#include "bsp_dt7_dr16.h"
#include "referee_protocol.h"

#define DT7_SW_UP ((uint16_t)1)
#define DT7_SW_MID ((uint16_t)3)
#define DT7_SW_DOWN ((uint16_t)2)
#define checkIfDt7SwitchIsUp(s) (s == DT7_SW_UP)
#define checkIfDt7SwitchIsMid(s) (s == DT7_SW_MID)
#define checkIfDt7SwitchIsDown(s) (s == DT7_SW_DOWN)

typedef enum RemoteControlKey
{
    RC_W = 0,
    RC_S,
    RC_A,
    RC_D,
    RC_SHIFT,
    RC_CTRL,
    RC_Q,
    RC_E,
    RC_R,
    RC_F,
    RC_G,
    RC_Z,
    RC_X,
    RC_C,
    RC_V,
    RC_B,

    RC_LEFT,
    RC_RIGHT,

    RC_KEY_NUM
} remote_control_key_e;

typedef struct RemoteControlKeyStatus
{
    uint8_t is_pressed : 1;
    uint8_t falling_edge_detected : 1; /* 按键被按下时置1 读取后清零 */
    uint8_t rising_edge_detected : 1;  /* 按键被抬起时置1 读取后清零 会有误触发 */
    uint8_t was_pressed : 1;
    uint8_t press_timer;
} remote_control_key_status_s;

typedef struct RemoteControl
{
    rfl_dt7_dr16_data_s dt7_dr16_data;
    const vt_link_remote_control_t *vt_link_data;

    int16_t mouse_x; /* 鼠标左右方向速度 右为正 */
    int16_t mouse_y; /* 鼠标前后方向速度 后为正 */
    int16_t mouse_z; /* 鼠标滚轮速度 向前滚为正 */

    remote_control_key_status_s keys[RC_KEY_NUM];
} remote_control_s;

extern void RemoteControlInit(void);
extern void RemoteControlUpdate(void);

extern int16_t getRcMouseX(void);
extern int16_t getRcMouseY(void);
extern int16_t getRcMouseZ(void);
extern uint8_t checkIsRcKeyPressed(remote_control_key_e key);
extern uint8_t checkIfRcKeyFallingEdgeDetected(remote_control_key_e key);
extern uint8_t checkIfRcKeyRisingEdgeDetected(remote_control_key_e key);

extern remote_control_s *getRemoteControlPointer(void);

#endif /* _REMOTE_CONTROL_H__ */
