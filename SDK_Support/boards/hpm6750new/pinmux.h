#ifndef HPM_PINMUX_H
#define HPM_PINMUX_H

#include "board.h"
#include "hpm_gpio_drv.h"
#include "hpm_gpiom_drv.h"

#define CAN0_TXD IOC_PAD_PB15
#define CAN0_RXD IOC_PAD_PB17
#define CAN1_TXD IOC_PAD_PB19
#define CAN1_RXD IOC_PAD_PB18
#define CAN2_TXD IOC_PAD_PB09
#define CAN2_RXD IOC_PAD_PB08
#define CAN3_RXD IOC_PAD_PA29
#define CAN3_TXD IOC_PAD_PA30

#define I2C0_SDA IOC_PAD_PA05
#define I2C0_SCL IOC_PAD_PA06
#define I2C1_SCL IOC_PAD_PA11
#define I2C1_SDA IOC_PAD_PA10

#define SPI0_MISO IOC_PAD_PD26
#define SPI0_MOSI IOC_PAD_PD21
#define SPI0_SCLK IOC_PAD_PD27

#define SPI1_MISO IOC_PAD_PD30
#define SPI1_MOSI IOC_PAD_PE04
#define SPI1_SCLK IOC_PAD_PD31

#define USB0_ID IOC_PAD_PF10
#define USB0_OC IOC_PAD_PE29
#define USB0_PWR IOC_PAD_PF09
#define USB1_ID IOC_PAD_PF07
#define USB1_OC IOC_PAD_PF05
#define USB1_PWR IOC_PAD_PF06

#define UART0_RX IOC_PAD_PB28
#define UART0_TX IOC_PAD_PB23
#define UART1_RX IOC_PAD_PB26
#define UART1_TX IOC_PAD_PB27
#define UART2_RX IOC_PAD_PB21
#define UART2_TX IOC_PAD_PB22
#define UART3_RX IOC_PAD_PB24
#define UART3_TX IOC_PAD_PB25
#define UART4_RX IOC_PAD_PB29
#define UART4_TX IOC_PAD_PB30
#define UART5_RX IOC_PAD_PA07
#define UART5_TX IOC_PAD_PA08

#define PWM0_CH1 IOC_PAD_PC01
#define PWM0_CH2 IOC_PAD_PC00
#define PWM0_CH3 IOC_PAD_PB31
#define PWM0_CH4 IOC_PAD_PA31

#define KEY_0 IOC_PAD_PA00
#define KEY_1 IOC_PAD_PA01

#define RGB_R IOC_PAD_PA03
#define RGB_G IOC_PAD_PA04
#define RGB_B IOC_PAD_PA02

#define BEEP IOC_PAD_PD17

#define IO01 IOC_PAD_PA24
#define IO02 IOC_PAD_PA25
#define IO04 IOC_PAD_PB01
#define IO05 IOC_PAD_PB02
#define IO12 IOC_PAD_PC30
#define IO13 IOC_PAD_PC31
#define IO24 IOC_PAD_PF00
#define IO25 IOC_PAD_PF01

#define IO03 IOC_PAD_PB04
#define IO06 IOC_PAD_PC16
#define IO07 IOC_PAD_PC17
#define IO08 IOC_PAD_PC21
#define IO09 IOC_PAD_PC22
#define IO10 IOC_PAD_PC25
#define IO11 IOC_PAD_PC26
#define IO14 IOC_PAD_PD11
#define IO15 IOC_PAD_PD14
#define IO16 IOC_PAD_PD15
#define IO17 IOC_PAD_PD22
#define IO18 IOC_PAD_PE03
#define IO19 IOC_PAD_PE23
#define IO20 IOC_PAD_PE27
#define IO21 IOC_PAD_PE28
#define IO22 IOC_PAD_PE30
#define IO23 IOC_PAD_PE31
#define IO26 IOC_PAD_PF03

#ifdef __cplusplus
extern "C" {

#endif
void init_led_pins_as_gpio(void);
void init_uart_pins(UART_Type *ptr);
void init_i2c_pins(I2C_Type *ptr);
void init_i2c_pins_as_gpio(I2C_Type *ptr);
void init_usb_pins(USB_Type *ptr);
void init_can_pins(CAN_Type *ptr);
void init_pwm_pins(PWM_Type *ptr);
void init_spi_pins(SPI_Type *ptr);
void init_sdram_pins(void);

void init_pins(void);

#ifdef __cplusplus
}
#endif
#endif /* HPM_PINMUX_H */