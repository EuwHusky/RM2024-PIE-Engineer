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
#include "behavior_task.h"
#include "detect_task.h"

// 裁判系统串口初始化
static void pm_uart_init(void);
// 图传链路串口初始化
static void vt_uart_init(void);

static void client_ui(void);

// 电管链路裁判系统数据相关变量和结构
ATTR_PLACE_AT_NONCACHEABLE uint8_t pm_rx_buf[PM_UART_RX_BUF_LENGHT]; // 接收原始数据
ATTR_PLACE_AT_NONCACHEABLE uint8_t pm_tx_buf[REF_PROTOCOL_FRAME_MAX_SIZE];
volatile bool pm_uart_rx_dma_done = true; // dma传输完成标志位
volatile bool pm_uart_tx_dma_done = true; // dma传输完成标志位
fifo_s_t *pm_uart_fifo = NULL;
uint8_t *pm_tx_frame_pointer;

// 图传链路裁判系统数据相关变量和结构
ATTR_PLACE_AT_NONCACHEABLE uint8_t vt_rx_buf[VT_UART_RX_BUF_LENGHT]; // 接收原始数据
volatile bool vt_uart_rx_dma_done = true;                            // dma传输完成标志位
fifo_s_t *vt_uart_fifo = NULL;

static uint32_t step_clock = 0;
uint32_t test_all = 0;
uint32_t test_pm = 0;
uint32_t test_vt = 0;
uint32_t test_ui = 0;
uint32_t test_reset = 0;

void pm_rx_referee_dma_isr(void)
{
    fifo_s_puts(pm_uart_fifo, (char *)pm_rx_buf, PM_UART_RX_BUF_LENGHT);
    pm_uart_rx_dma_done = true; // 更新标志位
    detect_hook(PM_REFEREE_DH);
}

void pm_tx_referee_dma_isr(void)
{
    pm_uart_tx_dma_done = true;
}

void vt_referee_dma_isr(void)
{
    fifo_s_puts(vt_uart_fifo, (char *)vt_rx_buf, VT_UART_RX_BUF_LENGHT);
    vt_uart_rx_dma_done = true; // 更新标志位
    detect_hook(VT_REFEREE_DH);
}

void referee_task(void *pvParameters)
{
    while (!INS_init_finished)
        vTaskDelay(10);
    vTaskDelay(200);

    pm_uart_init(); // 初始化裁判系统串口

    vt_uart_init(); // 初始化图传链路串口

    refereeInitData();
    refereeInitFrameProcesser();
    pm_uart_fifo = get_pm_fifo();
    vt_uart_fifo = get_vt_fifo();

    refereeInitRobotInteractionManager(&step_clock, 17, 16, 0);
    refereeSetRobotInteractionFigureBuilder(0, uiModeIndicatorBuilder);
    refereeSetRobotInteractionFigureBuilder(1, uiSplitLine0Builder);
    refereeSetRobotInteractionFigureBuilder(2, uiGrabberPoweredBuilder);
    refereeSetRobotInteractionFigureBuilder(3, uiGrabbedBuilder);
    refereeSetRobotInteractionFigureBuilder(4, uiSplitLine1Builder);
    refereeSetRobotInteractionFigureBuilder(5, uiStorageFrontUsedBuilder);
    refereeSetRobotInteractionFigureBuilder(6, uiStorageBackUsedBuilder);
    refereeSetRobotInteractionFigureBuilder(VAU_AID_0_UI_INDEX, uiVauAid0Builder);
    refereeSetRobotInteractionFigureBuilder(VAU_AID_1_UI_INDEX, uiVauAid1Builder);
    refereeSetRobotInteractionFigureBuilder(VAU_AID_2_UI_INDEX, uiVauAid2Builder);
    refereeSetRobotInteractionFigureBuilder(AIM_SIGHT_0_UI_INDEX, uiAimSight0Builder);
    refereeSetRobotInteractionFigureBuilder(AIM_SIGHT_1_UI_INDEX, uiAimSight1Builder);
    refereeSetRobotInteractionFigureBuilder(SAFE_RIGHT_BARRIER_WARNING_LINE_UI_INDEX,
                                            uiSafeRightBarrierWarningLineBuilder);
    refereeSetRobotInteractionFigureBuilder(DANGER_RIGHT_BARRIER_WARNING_LINE_UI_INDEX,
                                            uiDangerRightBarrierWarningLineBuilder);
    refereeSetRobotInteractionFigureBuilder(LIFTER_LEFT_MOTOR_OVER_TEMP_WARNING_UI_INDEX,
                                            uiLifterLeftMotorOverheatWarningBuilder);
    refereeSetRobotInteractionFigureBuilder(LIFTER_RIGHT_MOTOR_OVER_TEMP_WARNING_UI_INDEX,
                                            uiLifterRightMotorOverheatWarningBuilder);

    while (1)
    {
        if (pm_uart_rx_dma_done)
        {
            test_pm++;
            pm_uart_rx_dma_done = false;
            refereeUnpackFifoData(PM_REFEREE_LINK);
            uart_rx_trigger_dma(BOARD_HDMA, PM_UART_RX_DMA_CHN, PM_UART,
                                core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)pm_rx_buf),
                                PM_UART_RX_BUF_LENGHT);
        }

        if (vt_uart_rx_dma_done)
        {
            test_vt++;
            vt_uart_rx_dma_done = false;
            refereeUnpackFifoData(VT_REFEREE_LINK);
            uart_rx_trigger_dma(BOARD_HDMA, VT_UART_RX_DMA_CHN, VT_UART,
                                core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)vt_rx_buf),
                                VT_UART_RX_BUF_LENGHT);
        }

        client_ui();

        step_clock++;
        if (pm_uart_tx_dma_done)
        {
            test_ui++;
            detect_hook(UI_REFEREE_DH);

            pm_tx_frame_pointer = refereeEncodeRobotInteractionData(CLIENT_UI_PLOT);

            if (pm_tx_frame_pointer != NULL)
            {
                memcpy(pm_tx_buf, pm_tx_frame_pointer, getRefSentDataLen());
                uart_tx_trigger_dma(BOARD_HDMA, PM_UART_TX_DMA_CHN, PM_UART,
                                    core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)pm_tx_buf),
                                    getRefSentDataLen());
                refereeRobotInteractionManagerSuccessfullySentHook();
                pm_uart_tx_dma_done = false;
            }
        }

        // 发送挂了重新拉起
        if (detect_error(UI_REFEREE_DH))
        {
            test_reset++;
            pm_uart_tx_dma_done = true;
            uart_reset_tx_fifo(PM_UART);
            uart_flush(PM_UART);
        }

        test_all++;

        vTaskDelay(17);
    }
}

static void client_ui(void)
{
    if (checkIfNeedResetUi())
    {
        refereeClientUiOperate(UI_RESET_ALL, 0);
    }

    if (getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_MOVE ||
        getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_AUTO_MOVE_HOMING)
    {
        refereeClientUiOperate(UI_DISPLAY_FIGURE, SAFE_RIGHT_BARRIER_WARNING_LINE_UI_INDEX);
        refereeClientUiOperate(UI_DISPLAY_FIGURE, DANGER_RIGHT_BARRIER_WARNING_LINE_UI_INDEX);
        refereeClientUiOperate(UI_HIDE_FIGURE, LIFTER_LEFT_MOTOR_OVER_TEMP_WARNING_UI_INDEX);
        refereeClientUiOperate(UI_HIDE_FIGURE, LIFTER_RIGHT_MOTOR_OVER_TEMP_WARNING_UI_INDEX);
    }
    else
    {
        refereeClientUiOperate(UI_HIDE_FIGURE, SAFE_RIGHT_BARRIER_WARNING_LINE_UI_INDEX);
        refereeClientUiOperate(UI_HIDE_FIGURE, DANGER_RIGHT_BARRIER_WARNING_LINE_UI_INDEX);
        refereeClientUiOperate(UI_DISPLAY_FIGURE, LIFTER_LEFT_MOTOR_OVER_TEMP_WARNING_UI_INDEX);
        refereeClientUiOperate(UI_DISPLAY_FIGURE, LIFTER_RIGHT_MOTOR_OVER_TEMP_WARNING_UI_INDEX);
    }

    if (getVisualAidUi() == VAU_NONE)
    {
        refereeClientUiOperate(UI_HIDE_FIGURE, VAU_AID_0_UI_INDEX);
        refereeClientUiOperate(UI_HIDE_FIGURE, VAU_AID_1_UI_INDEX);
        refereeClientUiOperate(UI_HIDE_FIGURE, VAU_AID_2_UI_INDEX);
    }
    else
    {
        refereeClientUiOperate(UI_DISPLAY_FIGURE, VAU_AID_0_UI_INDEX);
        refereeClientUiOperate(UI_DISPLAY_FIGURE, VAU_AID_1_UI_INDEX);
        refereeClientUiOperate(UI_DISPLAY_FIGURE, VAU_AID_2_UI_INDEX);
    }

    if (getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_AUTO_SILVER_MINING ||
        getEngineerCurrentBehavior() == ENGINEER_BEHAVIOR_AUTO_GOLD_MINING)
    {
        refereeClientUiOperate(UI_DISPLAY_FIGURE, AIM_SIGHT_0_UI_INDEX);
        refereeClientUiOperate(UI_DISPLAY_FIGURE, AIM_SIGHT_1_UI_INDEX);
    }
    else
    {
        refereeClientUiOperate(UI_HIDE_FIGURE, AIM_SIGHT_0_UI_INDEX);
        refereeClientUiOperate(UI_HIDE_FIGURE, AIM_SIGHT_1_UI_INDEX);
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

/*图传链路串口初始化**/
static void vt_uart_init(void)
{
    uart_config_t config = {0}; // 串口配置
    board_init_uart(VT_UART);
    uart_default_config(VT_UART, &config);                    // 填充默认配置
    config.fifo_enable = true;                                // 使能FIFO
    config.dma_enable = true;                                 // 使能DMA
    config.baudrate = VT_BAUDRATE;                            // 设置波特率
    config.src_freq_in_hz = clock_get_frequency(VT_UART_CLK); // 获得时钟频率
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
    if (uart_init(VT_UART, &config) != status_success)
    {
        printf("failed to initialize uart\n");
        while (1)
        {
        }
    }
    intc_m_enable_irq_with_priority(BOARD_HDMA_IRQ, 1);
    dmamux_config(BOARD_DMAMUX, VT_UART_RX_DMAMUX_CHN, VT_UART_RX_DMA_REQ, true);

    rflDmaAddCallbackFunction(BOARD_HDMA, VT_UART_RX_DMA_CHN, vt_referee_dma_isr);
}
