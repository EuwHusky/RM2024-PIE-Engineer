#include "rc_task.h"

#include "hpm_dma_drv.h"
#include "hpm_dmamux_drv.h"
#include "hpm_uart_drv.h"

#include "FreeRTOS.h"
#include "task.h"

#include "drv_can.h"
#include "drv_dma.h"

#include "bsp_dt7_dr16.h"

#include "remote_control.h"

#include "detect_task.h"
#include "dualcore_task.h"

static void dbus_uart_init(void);

static void vt_link_relay_pack_0_can_rx_callback(uint8_t *rx_data);
static void vt_link_relay_pack_1_can_rx_callback(uint8_t *rx_data);
static void vt_link_relay_pack_2_can_rx_callback(uint8_t *rx_data);
static void vt_link_relay_pack_3_can_rx_callback(uint8_t *rx_data);
static void vt_link_relay_pack_4_can_rx_callback(uint8_t *rx_data);

ATTR_PLACE_AT_NONCACHEABLE uint8_t dbus_rx_buf[DBUS_RX_BUF_NUM]; // 遥控器数据接收缓冲区
volatile bool dbus_uart_rx_dma_done = true;                      // dma传输完成标志位
rfl_dt7_dr16_data_s dt7_dr16_prior_data = {0};
rfl_dt7_dr16_data_s dt7_dr16_data = {0};

const uint8_t *vt_link_relay_data[5] = {0, 0, 0, 0, 0};

ATTR_PLACE_AT_NONCACHEABLE custom_robot_data_t cc_data;
ATTR_PLACE_AT_NONCACHEABLE vt_link_remote_control_t vt_rc_data;

void dbus_dma_isr(void)
{
    dbus_uart_rx_dma_done = true;

    // 遍历缓冲区寻找正确数据 当数据有误时偏移一位再次尝试
    // 首先将数据保存至先验量 确认数据无误后再复制到对外可用的位置 以确保外部用到的数据的正确性

    for (uint8_t i = 0; i < (DBUS_RX_BUF_NUM / 2); i++)
    {
        rflDt7Dr16Decode(&dbus_rx_buf[i], &dt7_dr16_prior_data);
        if (rflDt7Dr16CheckIsDataCorrect(&dt7_dr16_prior_data))
        {
            memcpy(&dt7_dr16_data, &dt7_dr16_prior_data, sizeof(rfl_dt7_dr16_data_s));
            detect_hook_in_isr(DBUS_DH); // 记录更新时间
            break;
        }
    }
}

void rc_task(void *pvParameters)
{
    while (detect_error(DUAL_COMM_DH))
        vTaskDelay(14);

    dbus_uart_init(); // 初始化遥控器串口

    rflDmaAddCallbackFunction(BOARD_XDMA, DBUS_UART_RX_DMA_CHN, dbus_dma_isr);

    RemoteControlInit(&dt7_dr16_data, &vt_rc_data, &cc_data);

    for (uint8_t i = 0; i < 5; i++)
    {
        rflCanRxMessageBoxAddId(4, 0x300 + i);
        vt_link_relay_data[i] = rflCanGetRxMessageBoxData(4, 0x300 + i);
    }

    rflCanRxMessageBoxAddRxCallbackFunc(4, 0x300, vt_link_relay_pack_0_can_rx_callback);
    rflCanRxMessageBoxAddRxCallbackFunc(4, 0x301, vt_link_relay_pack_1_can_rx_callback);
    rflCanRxMessageBoxAddRxCallbackFunc(4, 0x302, vt_link_relay_pack_2_can_rx_callback);
    rflCanRxMessageBoxAddRxCallbackFunc(4, 0x303, vt_link_relay_pack_3_can_rx_callback);
    rflCanRxMessageBoxAddRxCallbackFunc(4, 0x304, vt_link_relay_pack_4_can_rx_callback);

    while (true)
    {
        // dma传输完成
        if (dbus_uart_rx_dma_done)
        {
            uart_rx_trigger_dma(BOARD_XDMA, DBUS_UART_RX_DMA_CHN, DBUS_UART,
                                core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)dbus_rx_buf),
                                DBUS_RX_BUF_NUM);
            dbus_uart_rx_dma_done = false;
        }

        RemoteControlUpdate(!detect_error(DBUS_DH), !detect_error(VT_REFEREE_DH));

        vTaskDelay(5);
    }
}

const custom_robot_data_t *getCcData(void)
{
    return &cc_data;
}

const vt_link_remote_control_t *getVtRcData(void)
{
    return &vt_rc_data;
}

static void dbus_uart_init(void)
{
    uart_config_t config = {0}; // 串口配置
    board_init_uart(DBUS_UART);
    uart_default_config(DBUS_UART, &config);                    // 填充默认配置
    config.fifo_enable = true;                                  // 使能FIFO
    config.dma_enable = true;                                   // 使能DMA
    config.baudrate = DBUS_BAUDRATE;                            // 设置波特率
    config.src_freq_in_hz = clock_get_frequency(DBUS_UART_CLK); // 获得时钟频率
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
    if (uart_init(DBUS_UART, &config) != status_success)
    {
        printf("failed to initialize uart\n");
        while (1)
        {
        }
    }
    intc_m_enable_irq_with_priority(BOARD_XDMA_IRQ, 1);
    dmamux_config(BOARD_DMAMUX, DBUS_UART_RX_DMAMUX_CHN, DBUS_UART_RX_DMA_REQ, true);
}

static void vt_link_relay_pack_0_can_rx_callback(uint8_t *rx_data)
{
    uint32_t temp;
    float temp_out;
    for (uint8_t i = 0; i < 2; i++)
    {
        temp = ((vt_link_relay_data[0][4u * i + 3u] << 24) | (vt_link_relay_data[0][4u * i + 2u] << 16) |
                (vt_link_relay_data[0][4u * i + 1u] << 8) | vt_link_relay_data[0][4u * i + 0u]);
        temp_out = *(float *)&temp;
        cc_data.pose[0 + i] = temp_out;
    }

    detect_hook_in_isr(VT_REFEREE_DH);
}
static void vt_link_relay_pack_1_can_rx_callback(uint8_t *rx_data)
{
    uint32_t temp;
    float temp_out;
    for (uint8_t i = 0; i < 2; i++)
    {
        temp = ((vt_link_relay_data[1][4u * i + 3u] << 24) | (vt_link_relay_data[1][4u * i + 2u] << 16) |
                (vt_link_relay_data[1][4u * i + 1u] << 8) | vt_link_relay_data[1][4u * i + 0u]);
        temp_out = *(float *)&temp;
        cc_data.pose[2 + i] = temp_out;
    }

    detect_hook_in_isr(VT_REFEREE_DH);
}
static void vt_link_relay_pack_2_can_rx_callback(uint8_t *rx_data)
{
    uint32_t temp;
    float temp_out;
    for (uint8_t i = 0; i < 2; i++)
    {
        temp = ((vt_link_relay_data[2][4u * i + 3u] << 24) | (vt_link_relay_data[2][4u * i + 2u] << 16) |
                (vt_link_relay_data[2][4u * i + 1u] << 8) | vt_link_relay_data[2][4u * i + 0u]);
        temp_out = *(float *)&temp;
        cc_data.pose[4 + i] = temp_out;
    }

    detect_hook_in_isr(VT_REFEREE_DH);
}
static void vt_link_relay_pack_3_can_rx_callback(uint8_t *rx_data)
{
    memcpy(&vt_rc_data, vt_link_relay_data[3], 8u);

    detect_hook_in_isr(VT_REFEREE_DH);
}
static void vt_link_relay_pack_4_can_rx_callback(uint8_t *rx_data)
{
    vt_rc_data.keyboard_value = ((vt_link_relay_data[4][1] << 8) | vt_link_relay_data[4][0]);
    cc_data.key = vt_link_relay_data[4][7];

    detect_hook_in_isr(VT_REFEREE_DH);
}
