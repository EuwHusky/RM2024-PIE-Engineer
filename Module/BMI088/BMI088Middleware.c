#include "BMI088Middleware.h"

// 初始化相关IO
void BMI088_GPIO_init(void)
{
  printf("BMI088_GPIO_init...\n");
  // 初始化片选：加速度计
  HPM_IOC->PAD[IOC_PAD_PC17].FUNC_CTL = IOC_PC17_FUNC_CTL_GPIO_C_17;
  gpio_set_pin_output_with_initial(HPM_GPIO0, CSB1_ACCEL_GPIO_Port, CSB1_ACCEL_Pin, 1);
  // 初始化片选：陀螺仪
  HPM_IOC->PAD[IOC_PAD_PC22].FUNC_CTL = IOC_PC22_FUNC_CTL_GPIO_C_22;
  gpio_set_pin_output_with_initial(HPM_GPIO0, CSB2_GYRO_GPIO_Port, CSB2_GYRO_Pin, 1);
  // 初始化中断触发引脚
  // HPM_IOC->PAD[IOC_PAD_PF01].FUNC_CTL = IOC_PF01_FUNC_CTL_GPIO_F_01;
  // gpio_set_pin_input(HPM_GPIO0, INT1_GPIO_Port, INT1_Pin);
  // gpio_enable_pin_interrupt(HPM_GPIO0, INT1_GPIO_Port, INT1_Pin);
  // gpio_config_pin_interrupt(HPM_GPIO0, INT1_GPIO_Port, INT1_Pin, gpio_interrupt_trigger_edge_rising);
  // gpiom_set_pin_controller(HPM_GPIOM, INT1_GPIO_Port, INT1_Pin, gpiom_soc_gpio0);
  // intc_m_enable_irq_with_priority(IRQn_GPIO0_F, 1);
  // HPM_IOC->PAD[IOC_PAD_PB27].FUNC_CTL = IOC_PB27_FUNC_CTL_GPIO_B_27;
  // gpio_set_pin_input(HPM_GPIO0, INT3_GPIO_Port, INT3_Pin);
  // gpio_enable_pin_interrupt(HPM_GPIO0, INT3_GPIO_Port, INT3_Pin);
  // gpio_config_pin_interrupt(HPM_GPIO0, INT3_GPIO_Port, INT3_Pin, gpio_interrupt_trigger_edge_rising);
  // gpiom_set_pin_controller(HPM_GPIOM, INT3_GPIO_Port, INT3_Pin, gpiom_soc_gpio0);
  // intc_m_enable_irq_with_priority(IRQn_GPIO0_B, 1);
}

// 初始化SPI
void BMI088_com_init(void)
{
  printf("BMI088_com_init...\n");
  board_init_spi_clock(BMI088_SPI);        // 初始化时钟
  board_init_spi_pins(BMI088_SPI);         // 初始化spi引脚
  spi_timing_config_t timing_config = {0}; // spi时钟配置
  spi_format_config_t format_config = {0}; // spi格式配置

  /* 设置主设备的SPI频率 */
  spi_master_get_default_timing_config(&timing_config); // 获取spi主机默认时序配置
  timing_config.master_config.clk_src_freq_in_hz = clock_get_frequency(clock_spi2);
  timing_config.master_config.sclk_freq_in_hz = 4000000; // 10000000,400000
  if (status_success != spi_master_timing_init(BMI088_SPI, &timing_config))
  {
    printf("SPI master timing init failed\n");
  }

  /* 为主设备设置 SPI 格式配置 */
  spi_master_get_default_format_config(&format_config);
  format_config.master_config.addr_len_in_bytes = 1;  // 地址长度
  format_config.common_config.data_len_in_bits = 8;   // 数据长度
  format_config.common_config.mode = spi_master_mode; // SPI工作模式
  spi_format_init(BMI088_SPI, &format_config);
}

void BMI088_delay_ms(uint16_t ms)
{
  board_delay_ms(ms);
}

void BMI088_delay_us(uint16_t us)
{
  board_delay_us(us);
}

void BMI088_ACCEL_NS_L(void)
{
  gpio_write_pin(HPM_GPIO0, CSB1_ACCEL_GPIO_Port, CSB1_ACCEL_Pin, 0);
}
void BMI088_ACCEL_NS_H(void)
{
  gpio_write_pin(HPM_GPIO0, CSB1_ACCEL_GPIO_Port, CSB1_ACCEL_Pin, 1);
}

void BMI088_GYRO_NS_L(void)
{
  gpio_write_pin(HPM_GPIO0, CSB2_GYRO_GPIO_Port, CSB2_GYRO_Pin, 0);
}
void BMI088_GYRO_NS_H(void)
{
  gpio_write_pin(HPM_GPIO0, CSB2_GYRO_GPIO_Port, CSB2_GYRO_Pin, 1);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
  uint8_t rx_data;
  hpm_stat_t stat;
  spi_control_config_t control_config = {0}; // spi控制配置
  spi_master_get_default_control_config(&control_config);
  control_config.common_config.trans_mode = spi_trans_write_read_together; // 传输模式
  stat = spi_transfer(BMI088_SPI, &control_config, NULL, NULL, &txdata, 1, &rx_data, 1);
  return rx_data;
}
