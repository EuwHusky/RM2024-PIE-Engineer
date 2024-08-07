#include "referee_task.h"

#include "hpm_dma_drv.h"
#include "hpm_dmamux_drv.h"
#include "hpm_uart_drv.h"

#include "FreeRTOS.h"
#include "task.h"

#include "drv_dma.h"

#include "referee.h"
#include "referee_frame_process.h"
#include "referee_robot_interaction_manager.h"

#include "ui_element_builder.h"

#include "INS_task.h"
#include "arm_task.h"
#include "behavior_task.h"
#include "detect_task.h"

// 裁判系统串口初始化
static void pm_uart_init(void);

static void client_ui_operation(void);

// 电管链路裁判系统数据相关变量和结构
ATTR_PLACE_AT_NONCACHEABLE uint8_t pm_rx_buf[PM_UART_RX_BUF_LENGHT]; // 接收原始数据
ATTR_PLACE_AT_NONCACHEABLE uint8_t pm_tx_buf[REF_PROTOCOL_FRAME_MAX_SIZE];
volatile bool pm_uart_rx_dma_done = true; // dma传输完成标志位
volatile bool pm_uart_tx_dma_done = true; // dma传输完成标志位
fifo_s_t *pm_uart_fifo = NULL;
uint8_t *pm_tx_frame_pointer;

static uint8_t ui_gcl_index = 0;

void pm_rx_referee_dma_isr(void)
{
    fifo_s_puts(pm_uart_fifo, (char *)pm_rx_buf, PM_UART_RX_BUF_LENGHT);
    pm_uart_rx_dma_done = true; // 更新标志位
    detect_hook_in_isr(PM_REFEREE_DH);
}

void pm_tx_referee_dma_isr(void)
{
    pm_uart_tx_dma_done = true;
}

void referee_task(void *pvParameters)
{
    while (!INS_init_finished)
        vTaskDelay(10);
    vTaskDelay(200);

    pm_uart_init(); // 初始化裁判系统串口

    refereeInitData();
    refereeInitFrameProcesser();
    pm_uart_fifo = get_pm_fifo();

    refereeRobotInteractionManagerInit(32, 1);
    refereeRobotInteractionFigureConfig(UI_REFRESH_REAL_TIME, uiMagicStick0);
    refereeRobotInteractionFigureConfig(UI_REFRESH_REAL_TIME, uiMagicStick1);
    refereeRobotInteractionFigureConfig(UI_REFRESH_REAL_TIME, uiMagicStick2);
    refereeRobotInteractionFigureConfig(UI_REFRESH_IN_QUEUE, uiBodyIndicator);
    refereeRobotInteractionFigureConfig(UI_REFRESH_REAL_TIME, uiArmGrabberIndicator);
    refereeRobotInteractionFigureConfig(UI_REFRESH_REAL_TIME, uiFrontStorageIndicator);
    refereeRobotInteractionFigureConfig(UI_REFRESH_REAL_TIME, uiBackStorageIndicator);
    refereeRobotInteractionFigureConfig(UI_REFRESH_IN_QUEUE, uiDt7Dr16linkIndicator);
    refereeRobotInteractionFigureConfig(UI_REFRESH_IN_QUEUE, uiVtlinkIndicator);
    refereeRobotInteractionFigureConfig(UI_REFRESH_ONCE, uiRemoteIndicatorLeftSplitLine);
    refereeRobotInteractionFigureConfig(UI_REFRESH_ONCE, uiRemoteIndicatorRightSplitLine);
    ui_gcl_index = refereeRobotInteractionFigureConfig(UI_REFRESH_ONCE, uiGimbalCaliLine);
    refereeRobotInteractionFigureConfig(UI_REFRESH_IN_QUEUE, uiLifterLeftMotorOverheatWarningBuilder);
    refereeRobotInteractionFigureConfig(UI_REFRESH_IN_QUEUE, uiLifterRightMotorOverheatWarningBuilder);
    refereeRobotInteractionFigureConfig(UI_REFRESH_IN_QUEUE, uiErrorIndicator);

    while (1)
    {
        if (pm_uart_rx_dma_done)
        {
            pm_uart_rx_dma_done = false;
            refereeUnpackFifoData(PM_REFEREE_LINK);
            uart_rx_trigger_dma(BOARD_HDMA, PM_UART_RX_DMA_CHN, PM_UART,
                                core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)pm_rx_buf),
                                PM_UART_RX_BUF_LENGHT);
        }

        client_ui_operation();

        if (pm_uart_tx_dma_done)
        {
            detect_hook(UI_REFEREE_DH);

            pm_tx_frame_pointer = refereeEncodeRobotInteractionData(xTaskGetTickCount(), CLIENT_UI_PLOT);

            if (pm_tx_frame_pointer != NULL)
            {
                memcpy(pm_tx_buf, pm_tx_frame_pointer, getRefSentDataLen());
                uart_tx_trigger_dma(BOARD_HDMA, PM_UART_TX_DMA_CHN, PM_UART,
                                    core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)pm_tx_buf),
                                    getRefSentDataLen());
                refereeRobotInteractionManagerSuccessfullySentHook(xTaskGetTickCount());
                pm_uart_tx_dma_done = false;
            }
        }

        // 发送挂了重新拉起
        if (detect_error(UI_REFEREE_DH))
        {
            pm_uart_tx_dma_done = true;
            uart_reset_tx_fifo(PM_UART);
            uart_flush(PM_UART);
        }

        vTaskDelay(11);
    }
}

static void client_ui_operation(void)
{
    // 重新刷新UI
    if (checkIfNeedResetUi())
    {
        refereeClientUiOperate(UI_RESET_ALL, 0);
    }

    // 运动警示线&图传云台校准线
    if (getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_MOVE ||
        getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING)
    {
        refereeClientUiOperate(UI_DISPLAY_FIGURE, ui_gcl_index);
    }
    else
    {
        refereeClientUiOperate(UI_HIDE_FIGURE, ui_gcl_index);
    }
}

// 裁判系统串口初始化
static void pm_uart_init(void)
{
    uart_config_t config = {0}; // 串口配置
    board_init_uart(PM_UART);
    uart_default_config(PM_UART, &config);                    // 填充默认配置
    config.fifo_enable = true;                                // 使能FIFO
    config.dma_enable = true;                                 // 使能DMA
    config.baudrate = PM_BAUDRATE;                            // 设置波特率
    config.src_freq_in_hz = clock_get_frequency(PM_UART_CLK); // 获得时钟频率
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
    config.tx_fifo_level = uart_tx_fifo_trg_not_full;
    if (uart_init(PM_UART, &config) != status_success)
    {
        printf("failed to initialize uart\n");
        while (1)
        {
        }
    }
    intc_m_enable_irq_with_priority(BOARD_HDMA_IRQ, 1);
    dmamux_config(BOARD_DMAMUX, PM_UART_RX_DMAMUX_CHN, PM_UART_RX_DMA_REQ, true);
    dmamux_config(BOARD_DMAMUX, PM_UART_TX_DMAMUX_CHN, PM_UART_TX_DMA_REQ, true);

    rflDmaAddCallbackFunction(BOARD_HDMA, PM_UART_RX_DMA_CHN, pm_rx_referee_dma_isr);
    rflDmaAddCallbackFunction(BOARD_HDMA, PM_UART_TX_DMA_CHN, pm_tx_referee_dma_isr);
}
