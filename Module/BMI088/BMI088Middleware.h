#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "board.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"
#include "hpm_spi_drv.h"

// 定义使用的SPI以及片选IO以及中断触发IO
#define BMI088_SPI HPM_SPI0                // SPI序号
#define CSB1_ACCEL_GPIO_Port GPIO_DO_GPIOC // 加速度计片选
#define CSB1_ACCEL_Pin 17
#define CSB2_GYRO_GPIO_Port GPIO_DO_GPIOC // 陀螺仪片选
#define CSB2_GYRO_Pin 22
#define INT1_GPIO_Port GPIO_DI_GPIOC // 发送加速度数据时产生中断信号
#define INT1_Pin 16
#define INT3_GPIO_Port GPIO_DI_GPIOD // 发送角速度数据时产生中断信号
#define INT3_Pin 22

extern void BMI088_GPIO_init(void);
extern void BMI088_com_init(void);
extern void BMI088_delay_ms(uint16_t ms);
extern void BMI088_delay_us(uint16_t us);

extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

#endif
