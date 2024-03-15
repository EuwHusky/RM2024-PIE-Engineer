#ifndef _HPM_BOARD_H
#define _HPM_BOARD_H

#include "hpm_clock_drv.h"
#include "hpm_common.h"
#include "hpm_gpiom_soc_drv.h"
#include "hpm_lcdc_drv.h"
#include "hpm_soc.h"
#include "hpm_soc_feature.h"
#include "pinmux.h"
#include <math.h>
#include <stdio.h>

// 额外添加
#define PI acos(-1)
#define __packed __attribute__((packed))
// 提供位操作方法（bit从0开始）
#define GET_BIT_VALUE(x, bit) ((x & (1 << bit)) >> bit) // 获取第bit位的数值
#define SET_BIT_0(x, bit) (x &= ~(1 << bit))            // 第bit位置0
#define SET_BIT_1(x, bit) (x |= (1 << bit))             // 第bit位置1
/*--------------------*/

#if !defined(CONFIG_NDEBUG_CONSOLE) || !CONFIG_NDEBUG_CONSOLE
#include "hpm_debug_console.h"
#endif

#define BOARD_NAME "hpm6750evkmini"
#define BOARD_UF2_SIGNATURE (0x0A4D5048UL)
#define SEC_CORE_IMG_START ILM_LOCAL_BASE

// 核心0为主核心
#ifndef BOARD_RUNNING_CORE
#define BOARD_RUNNING_CORE HPM_CORE0
#endif

/* 该部分宏定义用于选择控制台输出串口 */
// 默认uart0为core0的调试串口，uart1为core1的调试串口（不建议更改）
#ifndef BOARD_CONSOLE_TYPE
#define BOARD_CONSOLE_TYPE CONSOLE_TYPE_UART
#endif
#if BOARD_CONSOLE_TYPE == CONSOLE_TYPE_UART
#ifndef BOARD_CONSOLE_BASE
#if BOARD_RUNNING_CORE == HPM_CORE0
#define BOARD_CONSOLE_BASE HPM_UART0
#define BOARD_CONSOLE_CLK_NAME clock_uart0
#else
#define BOARD_CONSOLE_BASE HPM_UART1
#define BOARD_CONSOLE_CLK_NAME clock_uart1
#endif
#endif
#define BOARD_CONSOLE_BAUDRATE (115200UL) // 115200UL
#endif
/*------------------------------*/
/* RGB相关 */
#define LED_OFF 1
#define LED_ON 0

#define BOARD_RGB_GPIO_CTRL HPM_GPIO0
#define BOARD_RGB_GPIO_INDEX GPIO_DO_GPIOA
#define BOARD_R_GPIO_PIN 03
#define BOARD_G_GPIO_PIN 04
#define BOARD_B_GPIO_PIN 02
/*------------------------------*/
/* 按键相关 */
#define BOARD_KEY_GPIO_CTRL HPM_GPIO0
#define BOARD_KEY_GPIO_IRQ IRQn_GPIO0_A
#define BOARD_KEY_GPIO_INDEX GPIO_DO_GPIOA
#define BOARD_KEY0_GPIO_PIN 0
/*------------------------------*/
/* CAN相关 */
#define BOARD_CAN_BAUDRATE (1000000UL)
#define BOARD_CAN1 HPM_CAN0
#define BOARD_CAN1_IRQn IRQn_CAN0
#define BOARD_CAN2 HPM_CAN1
#define BOARD_CAN2_IRQn IRQn_CAN1
#define BOARD_CAN3 HPM_CAN2
#define BOARD_CAN3_IRQn IRQn_CAN2
#define BOARD_CAN4 HPM_CAN3
#define BOARD_CAN4_IRQn IRQn_CAN3

/*------------------------------*/
/* DMA相关 */
#define BOARD_XDMA HPM_XDMA
#define BOARD_HDMA HPM_HDMA
#define BOARD_XDMA_IRQ IRQn_XDMA
#define BOARD_HDMA_IRQ IRQn_HDMA
#define BOARD_DMAMUX HPM_DMAMUX
/*------------------------------*/
/* DBUS相关（使用uart4） */
#define DBUS_BAUDRATE (100000UL)
#define DBUS_UART HPM_UART4
#define DBUS_UART_CLK clock_uart4
#define DBUS_UART_RX_DMA_CHN (0U)
// #define DBUS_UART_RX_DMAMUX_CHN DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_HDMA, DBUS_UART_RX_DMA_CHN)
#define DBUS_UART_RX_DMAMUX_CHN DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_XDMA, DBUS_UART_RX_DMA_CHN)
#define DBUS_UART_RX_DMA_REQ HPM_DMA_SRC_UART4_RX
/* 电管裁判系统UART相关 */
#define PM_BAUDRATE (115200UL)
#define PM_UART HPM_UART3
#define PM_UART_CLK clock_uart3
#define PM_UART_RX_DMA_CHN (1U)
#define PM_UART_RX_DMAMUX_CHN DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_HDMA, PM_UART_RX_DMA_CHN)
#define PM_UART_RX_DMA_REQ HPM_DMA_SRC_UART3_RX
/* 图传裁判系统UART相关 */
#define VT_BAUDRATE (115200UL)
#define VT_UART HPM_UART2
#define VT_UART_CLK clock_uart2
#define VT_UART_RX_DMA_CHN (2U)
#define VT_UART_RX_DMAMUX_CHN DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_HDMA, VT_UART_RX_DMA_CHN)
#define VT_UART_RX_DMA_REQ HPM_DMA_SRC_UART2_RX

/* UART相关 */
#define BOARD_UART_BAUDRATE (115200UL)
#define BOARD_UART1 HPM_UART0
#define BOARD_UART1_CLK clock_uart0
#define BOARD_UART2 HPM_UART1
#define BOARD_UART2_CLK clock_uart1
#define BOARD_UART3 HPM_UART2
#define BOARD_UART3_CLK clock_uart2
#define BOARD_UART4 HPM_UART3
#define BOARD_UART4_CLK clock_uart3
#define BOARD_UART5 HPM_UART4
#define BOARD_UART5_CLK clock_uart4
#define BOARD_UART6 HPM_UART5
#define BOARD_UART6_CLK clock_uart5
#define BOARD_UART6_TX_DMA_CHN (3U)
#define BOARD_UART6_TX_DMAMUX_CHN DMA_SOC_CHN_TO_DMAMUX_CHN(BOARD_HDMA, BOARD_UART6_TX_DMA_CHN)
#define BOARD_UART6_TX_DMA_REQ HPM_DMA_SRC_UART5_TX

/*------------------------------*/
/* i2c 相关 */
#define BOARD_I2C1 HPM_I2C0
#define BOARD_I2C1_CLK clock_i2c0
#define BOARD_I2C2 HPM_I2C1
#define BOARD_I2C2_CLK clock_i2c1

/*------------------------------*/
/* 蜂鸣器 相关 */
#define BOARD_BEEP_POART (HPM_GPIO0)
#define BOARD_BEEP_INDEX (GPIO_DO_GPIOD)
#define BOARD_BEEP_PIN (17)
/*------------------------------*/
/* USB 相关 */
#define BOARD_USB0_ID_PORT (HPM_GPIO0)
#define BOARD_USB0_ID_GPIO_INDEX (GPIO_DO_GPIOF)
#define BOARD_USB0_ID_GPIO_PIN (10)
#define BOARD_USB0_OC_PORT (HPM_GPIO0)
#define BOARD_USB0_OC_GPIO_INDEX (GPIO_DI_GPIOF)
#define BOARD_USB0_OC_GPIO_PIN (8)

#define BOARD_USB1_ID_PORT (HPM_GPIO0)
#define BOARD_USB1_ID_GPIO_INDEX (GPIO_DO_GPIOF)
#define BOARD_USB1_ID_GPIO_PIN (7)
#define BOARD_USB1_OC_PORT (HPM_GPIO0)
#define BOARD_USB1_OC_GPIO_INDEX (GPIO_DI_GPIOF)
#define BOARD_USB1_OC_GPIO_PIN (5)
/*------------------------------*/

#define BOARD_CPU_FREQ (648000000UL)

#ifndef BOARD_SHOW_CLOCK
#define BOARD_SHOW_CLOCK 1
#endif
#ifndef BOARD_SHOW_BANNER
#define BOARD_SHOW_BANNER 1
#endif

#ifndef BOARD_RUNNING_CORE
#define BOARD_RUNNING_CORE 0
#endif

#if defined(__cplusplus)
extern "C"
{
#endif /* __cplusplus */

    typedef void (*board_timer_cb)(void);

    void board_delay_ms(uint32_t ms); // 毫秒延时
    void board_delay_us(uint32_t us); // 微秒延时

    void board_init(void);         // 初始化板子
    void board_init_clock(void);   // 初始化soc时钟
    void board_init_console(void); // 初始化控制台
    void board_init_pmp(void);     // 初始化pmp和pma

    void board_init_led_pins(void);        // 初始化LED
    void board_write_led_r(uint8_t state); // 对红色灯写状态
    void board_write_led_g(uint8_t state); // 对绿色灯写状态
    void board_write_led_b(uint8_t state); // 对蓝色灯写状态

    void board_init_beep_pins(void); // 初始化蜂鸣器
    void board_beep_open(void);      // 打开蜂鸣器
    void board_beep_close(void);     // 关闭蜂鸣器

    void board_init_uart(UART_Type *ptr);           // 初始化UART
    uint32_t board_init_uart_clock(UART_Type *ptr); // 初始化串口时钟

    void board_init_can(CAN_Type *ptr);           // 初始化CAN
    uint32_t board_init_can_clock(CAN_Type *ptr); // 初始化CAN时钟

    void board_init_i2c(I2C_Type *ptr); // 初始化i2c

    void board_init_spi_pins(SPI_Type *ptr);      // 初始化spi
    uint32_t board_init_spi_clock(SPI_Type *ptr); // 初始化spi时钟

    void board_init_usb_pins(void); // 初始化usb

    extern hpm_stat_t uart_rx_trigger_dma(DMA_Type *dma_ptr, uint8_t ch_num, UART_Type *uart_ptr, uint32_t dst,
                                          uint32_t size);
    extern hpm_stat_t uart_tx_trigger_dma(DMA_Type *dma_ptr, uint8_t ch_num, UART_Type *uart_ptr, uint32_t src,
                                          uint32_t size);

#endif /* _HPM_BOARD_H */
