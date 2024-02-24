#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "board.h"

#if !BOARD_RUNNING_CORE

#else
/* CANID */
typedef enum {
  /*----------CAN4（底盘can）----------*/
  CAN_CHASSIS_SEND_ID = 0x200, // 底盘的CAN发送id
  CAN_3508_DRIVE_ID1 = 0X201,  // id:1
  CAN_3508_DRIVE_ID2 = 0X202,  // id:2
  CAN_3508_DRIVE_ID3 = 0x203,  // id:3
  CAN_3508_DRIVE_ID4 = 0x204,  // id:4

  CAN_GIMBAL_SEND_ID_2 = 0x1FF, // 云台控制的CAN发送id
  CAN_6020_YAW_ID = 0x205,      // id:1
  CAN_3508_TRIGGER_ID = 0x206,  // id:6

  CAN_SUPER_RX_ID = 0x3ff, // 超电接收ID
  CAN_SUPER_TX_ID = 0x400, // 超电发送ID

  /*----------CAN3（云台can）----------*/
  CAN_GIMBAL_SEND_ID_1 = 0x200, // 云台控制的CAN发送id
  CAN_3508_SHOOTL_ID = 0X201,   // id:1
  CAN_3508_SHOOTR_ID = 0X202,   // id:2
  CAN_2006_TRIGGER_ID = 0x203,  // id:3
  CAN_3508_PITCH_ID = 0x204,    // id:4

  // 新云台更换6020后
  // CAN_GIMBAL_SEND_ID_1 = 0x1FF, // 云台控制的CAN发送id
  // CAN_3508_PITCH_ID = 0x205,    // id:1
  // CAN_3508_SHOOTL_ID = 0X206,   // id:6
  // CAN_3508_SHOOTR_ID = 0X207,   // id:7
  // CAN_2006_TRIGGER_ID = 0x203,  // id:3

  /*----------通信部分----------*/

} can_msg_id_e;

typedef struct
{
  uint16_t power_now;
  uint8_t power_limit;
  uint8_t power_buffer;
  uint8_t super;
} power;

typedef struct
{
  uint16_t ecd;
  int16_t speed_rpm;
  int16_t given_current;
  uint8_t temperate;
  int16_t last_ecd;
} motor_measure_t;

typedef struct
{
  uint16_t cooling_heat;
  uint16_t heat_limit;
  uint16_t shoot_speed_limit;
  uint8_t robot_id;
} refree_data_t;

#define M3508_MAX_CAN_CURRENT (16000.0f) // 3508最大can发送电流值
#define M6020_MAX_CAN_CURRENT (30000.0f) // 6020最大can发送电压值
#define M2006_MAX_CAN_CURRENT (10000.0f) // 2006最大can发送电流值

extern void can_send_gimbal_1(int16_t left, int16_t right, int16_t tri_2006, int16_t pitch);
extern void can_send_gimbal_2(int16_t yaw, int16_t tri_3508);
extern void can_send_chassis(int16_t drive1, int16_t drive2, int16_t drive3, int16_t drive4);
extern void can_send_super(void);

/* 返回电机数据指针 */
extern const motor_measure_t *get_yaw_6020_measure_point(void);
extern const motor_measure_t *get_pitch_3508_measure_point(void);
extern const motor_measure_t *get_shootl_3508_measure_point(void);
extern const motor_measure_t *get_shootr_3508_measure_point(void);
extern const motor_measure_t *get_trigger_2006_measure_point(void);
extern const motor_measure_t *get_trigger_3508_measure_point(void);
extern const motor_measure_t *get_chassis_3508_measure_point(uint8_t i);
#endif

#endif
