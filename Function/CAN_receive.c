#include "CAN_receive.h"
#include "hpm_can_drv.h"

// 对CAN的功能进行分类
#define TRANSMIT_CAN_1 BOARD_CAN1
#define TRANSMIT_CAN_2 BOARD_CAN2
#define GIMBAL_CAN BOARD_CAN3
#define CHASSIS_CAN BOARD_CAN4

static volatile uint8_t error_flags[4]; // 错误中断标志
static volatile bool has_error[4];      // 出现错误

// 分配每路can的中断服务函数（需要和终端服务函数中处理的can一致）
#if !BOARD_RUNNING_CORE
SDK_DECLARE_EXT_ISR_M(BOARD_CAN1_IRQn, transmit_can1_isr);
SDK_DECLARE_EXT_ISR_M(BOARD_CAN2_IRQn, transmit_can2_isr);

#else
#include "Detect_task.h"

SDK_DECLARE_EXT_ISR_M(BOARD_CAN3_IRQn, gimbal_can_isr);
SDK_DECLARE_EXT_ISR_M(BOARD_CAN4_IRQn, chassis_can_isr);

// 电机数据解包
#define get_motor_measure(ptr, data)                               \
  {                                                                \
    (ptr)->last_ecd = (ptr)->ecd;                                  \
    (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);           \
    (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     \
    (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); \
    (ptr)->temperate = (data)[6];                                  \
  }

// 电机数据
static motor_measure_t chassis_motor_data[4];
static motor_measure_t gimbal_motor_data[6];

#endif

#if !BOARD_RUNNING_CORE
// 通信can1的中断服务函数
void transmit_can1_isr(void) {
  uint8_t can = can_get_tx_rx_flags(TRANSMIT_CAN_1); // 获取CAN标志位
  //  如果接收到消息
  if ((can & CAN_EVENT_RECEIVE) != 0) {
    can_receive_buf_t rx_buf; // 接收缓冲
    memset(&rx_buf, 0, sizeof(rx_buf));
    can_read_received_message(TRANSMIT_CAN_1, (can_receive_buf_t *)&rx_buf);
    switch (rx_buf.id) {
      /*-----在此添加传输数据解码程序-----*/
      break;
    }
  }
  can_clear_tx_rx_flags(TRANSMIT_CAN_1, can); // 清除标志位
  // 如果出现错误,更新标志位并重启can
  if ((can & CAN_EVENT_ERROR) != 0) {
    has_error[0] = true;
    can_reset(TRANSMIT_CAN_1, false);
  }
  error_flags[0] = can_get_error_interrupt_flags(TRANSMIT_CAN_1);
  can_clear_error_interrupt_flags(TRANSMIT_CAN_1, error_flags[0]);
}

// 通信can2的中断服务函数
void transmit_can2_isr(void) {
  uint8_t can = can_get_tx_rx_flags(TRANSMIT_CAN_2); // 获取CAN标志位
  //  如果接收到消息
  if ((can & CAN_EVENT_RECEIVE) != 0) {
    can_receive_buf_t rx_buf; // 接收缓冲
    memset(&rx_buf, 0, sizeof(rx_buf));
    can_read_received_message(TRANSMIT_CAN_2, (can_receive_buf_t *)&rx_buf);
    switch (rx_buf.id) {
      /*-----在此添加传输数据解码程序-----*/
      break;
    }
  }
  can_clear_tx_rx_flags(TRANSMIT_CAN_2, can); // 清除标志位
  // 如果出现错误,更新标志位并重启can
  if ((can & CAN_EVENT_ERROR) != 0) {
    has_error[1] = true;
    can_reset(TRANSMIT_CAN_2, false);
  }
  error_flags[1] = can_get_error_interrupt_flags(TRANSMIT_CAN_2);
  can_clear_error_interrupt_flags(TRANSMIT_CAN_2, error_flags[1]);
}
#else
void can_send_gimbal_1(int16_t left, int16_t right, int16_t tri_2006, int16_t pitch) {
  can_transmit_buf_t tx_buf; // 发送缓冲
  memset(&tx_buf, 0, sizeof(tx_buf));
  tx_buf.dlc = can_payload_size_8;
  tx_buf.id = CAN_GIMBAL_SEND_ID_1;
  tx_buf.data[0] = (left >> 8);
  tx_buf.data[1] = left;
  tx_buf.data[2] = (right >> 8);
  tx_buf.data[3] = right;
  tx_buf.data[4] = (tri_2006 >> 8);
  tx_buf.data[5] = tri_2006;
  tx_buf.data[6] = (pitch >> 8);
  tx_buf.data[7] = pitch;
  can_send_message_nonblocking(GIMBAL_CAN, &tx_buf);
}

void can_send_gimbal_2(int16_t yaw, int16_t tri_3508) {
  can_transmit_buf_t tx_buf; // 发送缓冲
  memset(&tx_buf, 0, sizeof(tx_buf));
  tx_buf.dlc = can_payload_size_4;
  tx_buf.id = CAN_GIMBAL_SEND_ID_2;
  tx_buf.data[0] = (yaw >> 8);
  tx_buf.data[1] = yaw;
  tx_buf.data[2] = (tri_3508 >> 8);
  tx_buf.data[3] = tri_3508;
  can_send_message_nonblocking(CHASSIS_CAN, &tx_buf);
}

void can_send_chassis(int16_t drive1, int16_t drive2, int16_t drive3, int16_t drive4) {
  can_transmit_buf_t tx_buf; // 发送缓冲
  memset(&tx_buf, 0, sizeof(tx_buf));
  tx_buf.dlc = can_payload_size_8;
  tx_buf.id = CAN_CHASSIS_SEND_ID;
  tx_buf.data[0] = (drive1 >> 8);
  tx_buf.data[1] = drive1;
  tx_buf.data[2] = (drive2 >> 8);
  tx_buf.data[3] = drive2;
  tx_buf.data[4] = (drive3 >> 8);
  tx_buf.data[5] = drive3;
  tx_buf.data[6] = (drive4 >> 8);
  tx_buf.data[7] = drive4;
  can_send_message_nonblocking(CHASSIS_CAN, &tx_buf);
}

void can_send_super(void) {
  can_transmit_buf_t tx_buf; // 发送缓冲
  memset(&tx_buf, 0, sizeof(tx_buf));
  tx_buf.dlc = can_payload_size_8;
  tx_buf.id = CAN_SUPER_TX_ID;
  // tx_buf.data[0] = (drive1 >> 8);
  // tx_buf.data[1] = drive1;
  // tx_buf.data[2] = (drive2 >> 8);
  // tx_buf.data[3] = drive2;
  // tx_buf.data[4] = (drive3 >> 8);
  // tx_buf.data[5] = drive3;
  // tx_buf.data[6] = (drive4 >> 8);
  // tx_buf.data[7] = drive4;
  can_send_message_nonblocking(CHASSIS_CAN, &tx_buf);
}

// 云台控制的can中断服务函数
void gimbal_can_isr(void) {
  uint8_t can = can_get_tx_rx_flags(GIMBAL_CAN); // 获取CAN标志位
  //  如果接收到消息
  if ((can & CAN_EVENT_RECEIVE) != 0) {
    can_receive_buf_t rx_buf; // 接收缓冲
    memset(&rx_buf, 0, sizeof(rx_buf));
    can_read_received_message(GIMBAL_CAN, (can_receive_buf_t *)&rx_buf);
    switch (rx_buf.id) {
    case CAN_3508_SHOOTL_ID:
    case CAN_3508_SHOOTR_ID:
    case CAN_2006_TRIGGER_ID:
    case CAN_3508_PITCH_ID: {
      uint8_t i = rx_buf.id - CAN_3508_SHOOTL_ID;
      get_motor_measure(&gimbal_motor_data[i], rx_buf.data);
      detect_hook(My_shootl + i); // 记录时间
      break;
    }
    }
  }
  can_clear_tx_rx_flags(GIMBAL_CAN, can); // 清除标志位
  // 如果出现错误,更新标志位并重启can
  if ((can & CAN_EVENT_ERROR) != 0) {
    has_error[2] = true;
    can_reset(GIMBAL_CAN, false);
  }
  error_flags[2] = can_get_error_interrupt_flags(GIMBAL_CAN);
  can_clear_error_interrupt_flags(GIMBAL_CAN, error_flags[2]);
}

// 底盘控制的can中断服务函数
void chassis_can_isr(void) {
  uint8_t can = can_get_tx_rx_flags(CHASSIS_CAN); // 获取CAN标志位
  //  如果接收到消息
  if ((can & CAN_EVENT_RECEIVE) != 0) {
    can_receive_buf_t rx_buf; // 接收缓冲
    memset(&rx_buf, 0, sizeof(rx_buf));
    can_read_received_message(CHASSIS_CAN, (can_receive_buf_t *)&rx_buf);
    switch (rx_buf.id) {
    case CAN_3508_DRIVE_ID1:
    case CAN_3508_DRIVE_ID2:
    case CAN_3508_DRIVE_ID3:
    case CAN_3508_DRIVE_ID4: {
      uint8_t i = rx_buf.id - CAN_3508_DRIVE_ID1;
      get_motor_measure(&chassis_motor_data[i], rx_buf.data);
      detect_hook(My_drive1 + i); // 记录时间
      break;
    }
    case CAN_6020_YAW_ID:
    case CAN_3508_TRIGGER_ID: {
      uint8_t i = rx_buf.id - CAN_3508_SHOOTL_ID;
      get_motor_measure(&gimbal_motor_data[i], rx_buf.data);
      detect_hook(My_yaw + rx_buf.id - CAN_6020_YAW_ID); // 记录时间
      break;
    }
    case CAN_SUPER_RX_ID: {
      // 接收超电信息
    }
    }
  }
  can_clear_tx_rx_flags(CHASSIS_CAN, can); // 清除标志位
  // 如果出现错误,更新标志位并重启can
  if ((can & CAN_EVENT_ERROR) != 0) {
    has_error[3] = true;
    can_reset(CHASSIS_CAN, false);
  }
  error_flags[3] = can_get_error_interrupt_flags(CHASSIS_CAN);
  can_clear_error_interrupt_flags(CHASSIS_CAN, error_flags[3]);
}

const motor_measure_t *get_shootl_3508_measure_point(void) { return &gimbal_motor_data[0]; }
const motor_measure_t *get_shootr_3508_measure_point(void) { return &gimbal_motor_data[1]; }
const motor_measure_t *get_trigger_2006_measure_point(void) { return &gimbal_motor_data[2]; }
const motor_measure_t *get_trigger_3508_measure_point(void) { return &gimbal_motor_data[5]; }
const motor_measure_t *get_yaw_6020_measure_point(void) { return &gimbal_motor_data[4]; }
const motor_measure_t *get_pitch_3508_measure_point(void) { return &gimbal_motor_data[3]; }

const motor_measure_t *get_chassis_3508_measure_point(uint8_t i) { return &chassis_motor_data[i]; };
#endif
