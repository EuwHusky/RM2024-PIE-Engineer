#include "Dualcore_task.h"
#include "FreeRTOS.h"
#include "hpm_mbx_drv.h"
#include "task.h"

#include "Detect_task.h"

/* 因为通信数据量较大，所以采用消息字+共享内存方式通信，并未启用fifo发送/接收 */
/* 通用变量 */
uint32_t message_tx;                                          // 发送消息
uint32_t message_rx;                                          // 接收消息
volatile uint8_t *shared_address_021 = (uint8_t *)0x0117C000; // 申请共用内存区域（core0-core1）
volatile uint8_t *shared_address_120 = (uint8_t *)0x0117E000; // 申请共用内存区域（core1-core0）

// 传输数据用的结构体
transmit_data_021 data_021;
transmit_data_120 data_120;

/* 通用函数 */
static void message_update(void); // 发送&接收标志信号

// 根据不同核心引用不同的头文件，以及不同变量声明
#if !BOARD_RUNNING_CORE
/* 此处引用core0的头文件 */
#include "INS_task.h"

/* 此处声明core0使用的变量 */
INS_t *ins;            // 陀螺仪数据指针
Refree_data_t *refree; // 裁判系统数据指针
/*----------------------------------------*/
#else
/* 此处引用core1的头文件 */

/* 此处声明core1使用的变量 */

#endif

// 双核通信任务
void Dualcore_task(void *pvParameters) {
  mbx_init(MBX);                               // mbx初始化
  intc_m_enable_irq_with_priority(MBX_IRQ, 1); // 初始化mbx中断
  mbx_enable_intr(MBX, MBX_CR_TWMEIE_MASK);    // 使能消息发送中断
  mbx_enable_intr(MBX, MBX_CR_RWMVIE_MASK);    // 使能消息接收中断

#if !BOARD_RUNNING_CORE
  // 等待陀螺仪线程初始化完成
  while (!INS_init_finished)
    vTaskDelay(1);

  ins = get_INS_data_point();
  refree = get_refree_data_point();

  // 标志位初始值设定
  SET_BIT_0(message_tx, 0); // 0位置0表示core0写入；置1表示core0写入完成

  while (true) {
    message_update(); // 发送&接收标志位
    if (GET_BIT_VALUE(message_rx, 0) == 0) {
      //// 装填陀螺仪数据（角度转弧度）
      //data_021.Yaw = ins->Yaw / 180.0f * PI;
      //data_021.Yaw_Speed = ins->Gyro[2];
      //data_021.Pitch = ins->Pitch / 180.0f * PI;
      //data_021.Pitch_Speed = ins->Gyro[1];

      //memcpy(&data_021.computer_data, computer, sizeof(recv_data_t));           // 装填小电脑数据
      //memcpy(&data_021.refree_data, refree, sizeof(Refree_data_t));             // 装填裁判系统数据
      //memcpy((void *)shared_address_021, &data_021, sizeof(transmit_data_021)); // 向内存中写入
      SET_BIT_1(message_tx, 0);                                                 // 写入完成，标志位置1
    }
    if (GET_BIT_VALUE(message_rx, 1) == 1) {
      memcpy(&data_120, (void *)shared_address_120, sizeof(transmit_data_120)); // 从内存中读取
      SET_BIT_0(message_tx, 1);                                                 // 读取完成，标志位置0
      detect_hook(My_dualcore);                                                 // 记录收到数据时间
    }
    vTaskDelay(1);
  }
#else
  SET_BIT_0(message_tx, 1); // 1位置0表示core1写入；置1表示core1写入完成

  while (true) {
    message_update();
    if (GET_BIT_VALUE(message_rx, 1) == 0) {
      // 在此向cpu0发送数据
      // memcpy(&data_120, &data_021, sizeof(transmit_data_120));
      memcpy((void *)shared_address_120, &data_120, sizeof(transmit_data_120));
      SET_BIT_1(message_tx, 1); // 写入完成，标志位置1
    }
    if (GET_BIT_VALUE(message_rx, 0) == 1) {
      memcpy(&data_021, (void *)shared_address_021, sizeof(transmit_data_021)); // 读取cpu0发送来的数据
      SET_BIT_0(message_tx, 0);                                                 // 读取完成，标志位置0
      detect_hook(My_dualcore);                                                 // 记录收到数据时间
    }
    vTaskDelay(1);
  }
#endif
}

// 返回核心0向核心1传输的数据地址
transmit_data_021 *get_data_021_point() {
  return &data_021;
}

// 返回核心1向核心0传输的数据地址
transmit_data_120 *get_data_120_point() {
  return &data_120;
}

// 更新消息
static void message_update(void) {
  // 若可写
  if (allow_write) {
    mbx_send_message(MBX, message_tx);        // 发送消息
    allow_write = false;                      // 状态更新：不可写
    mbx_enable_intr(MBX, MBX_CR_TWMEIE_MASK); // 使能发送中断
  }
  // 若可读
  if (allow_read) {
    mbx_retrieve_message(MBX, &message_rx);   // 接收消息
    allow_read = false;                       // 状态更新：不可读
    mbx_enable_intr(MBX, MBX_CR_RWMVIE_MASK); // 使能接收中断
  }
}

// mbx消息中断
void isr_mbx(void) {
  volatile uint32_t sr = MBX->SR; // 状态寄存器
  volatile uint32_t cr = MBX->CR; // 控制寄存器
  // 接收消息字有效，并且接收消息字有效中断使能
  if ((sr & MBX_SR_RWMV_MASK) && (cr & MBX_CR_RWMVIE_MASK)) {
    mbx_disable_intr(MBX, MBX_CR_RWMVIE_MASK); // 禁用接收消息字有效中断
    allow_read = true;                         // 更新标志位：可以读取
  }
  // 发送消息字为空（已被读取），并且发送消息字已被读取中断使能
  if ((sr & MBX_SR_TWME_MASK) && (cr & MBX_CR_TWMEIE_MASK)) {
    mbx_disable_intr(MBX, MBX_CR_TWMEIE_MASK); // 禁用发送消息字已被读取中断
    allow_write = true;                        // 更新标志位：可以发送
  }
}
SDK_DECLARE_EXT_ISR_M(MBX_IRQ, isr_mbx)
