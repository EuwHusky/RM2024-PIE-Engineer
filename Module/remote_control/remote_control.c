#include "remote_control.h"
#include "FreeRTOS.h"
#include "hpm_dma_drv.h"
#include "hpm_dmamux_drv.h"
#include "hpm_uart_drv.h"
#include "task.h"

#include "drv_dma.h"

#include "detect_task.h"
#include "dualcore_task.h"

#define DBUS_TASK_LOOP_TIME 5 // 任务间隔 ms

RC_ctrl_t rc_ctrl; // 遥控器数据结构体
IfKey rckey;       // 键盘数据结构体

// 接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
ATTR_PLACE_AT_NONCACHEABLE uint8_t sbus_rx_buf[SBUS_RX_BUF_NUM]; // 遥控器数据接收缓冲区
volatile bool dbus_uart_rx_dma_done = true;                      // dma传输完成标志位
static uint8_t dbus_err_counts = 0;                              // 遥控器数据错误次数

static int16_t RC_abs(int16_t value);                                         // 取正函数
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl); // 遥控器协议解析
static bool RC_data_is_error(void);                                           // 判断遥控器数据是否出错

static void dbus_uart_init(void); // 遥控器串口初始化

void dbus_dma_isr(void)
{
    dbus_uart_rx_dma_done = true;
    sbus_to_rc(&sbus_rx_buf[dbus_err_counts], &rc_ctrl); // 数据解包
    detect_hook(DBUS_DH);                                // 记录更新时间
    if (RC_data_is_error())                              // 记录数据错误次数
        dbus_err_counts++;
    dbus_err_counts %= 18; // 遥控器数据解包偏移值
}

// // DMA中断回调函数
// void dbus_dma_isr(void)
// {
//     volatile hpm_stat_t stat_rx_chn;

//     if (stat_rx_chn = dma_check_transfer_status(BOARD_XDMA, DBUS_UART_RX_DMA_CHN), stat_rx_chn &
//     DMA_CHANNEL_STATUS_TC)
//     {
//         dbus_uart_rx_dma_done = true;
//         sbus_to_rc(&sbus_rx_buf[dbus_err_counts], &rc_ctrl); // 数据解包
//         detect_hook(DBUS_DH);                                // 记录更新时间
//         if (RC_data_is_error())                              // 记录数据错误次数
//             dbus_err_counts++;
//         dbus_err_counts %= 18; // 遥控器数据解包偏移值
//     }
// }
// SDK_DECLARE_EXT_ISR_M(BOARD_XDMA_IRQ, dbus_dma_isr)

// 遥控器链路接收任务
void dbus_task(void *pvParameters)
{
    while (detect_error(DUAL_COMM_DH))
        vTaskDelay(DBUS_TASK_LOOP_TIME);

    dbus_uart_init(); // 初始化遥控器串口

    rflDmaAddCallbackFunction(BOARD_XDMA, DBUS_UART_RX_DMA_CHN, dbus_dma_isr);

    while (true)
    {
        // dma传输完成
        if (dbus_uart_rx_dma_done)
        {
            uart_rx_trigger_dma(BOARD_XDMA, DBUS_UART_RX_DMA_CHN, DBUS_UART,
                                core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)sbus_rx_buf),
                                SBUS_RX_BUF_NUM);
            dbus_uart_rx_dma_done = false;
        }
        vTaskDelay(DBUS_TASK_LOOP_TIME);
    }
}

/*遥控器串口初始化**/
void dbus_uart_init(void)
{
    hpm_stat_t stat;
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

/*获取遥控器数据指针*/
const RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}

// 判断遥控器数据是否出错，
static bool RC_data_is_error(void)
{
    // 使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_abs(rc_ctrl.rc.ch[0]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[1]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[2]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[3]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_abs(rc_ctrl.rc.ch[4]) > RC_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[0] == 0)
    {
        goto error;
    }
    if (rc_ctrl.rc.s[1] == 0)
    {
        goto error;
    }
    return false;

error:
    rc_ctrl.rc.ch[0] = 0;
    rc_ctrl.rc.ch[1] = 0;
    rc_ctrl.rc.ch[2] = 0;
    rc_ctrl.rc.ch[3] = 0;
    rc_ctrl.rc.ch[4] = 0;
    rc_ctrl.rc.s[0] = RC_SW_DOWN;
    rc_ctrl.rc.s[1] = RC_SW_DOWN;
    rc_ctrl.mouse.x = 0;
    rc_ctrl.mouse.y = 0;
    rc_ctrl.mouse.z = 0;
    rc_ctrl.mouse.press_l = 0;
    rc_ctrl.mouse.press_r = 0;
    rc_ctrl.key.v = 0;
    return true;
}

// 取正函数
static int16_t RC_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}

/*遥控器协议解析*/
static void sbus_to_rc(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) &
                        0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                       //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = (sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07ff;      //

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

/*rc按键状态更新函数*/
void rc_keyboard_update(void)
{

    if (rckey.Time_Now + RC_KEY_ELI >=
        hpm_csr_get_core_mcycle()) // 单位间隔时间内返回false值,即消抖，即按键有效状态保留时间
    {
        return;
    }
    uint16_t Vkey = rc_ctrl.key.v;
    rckey.Last_State = rckey.Now_State; // 保存上一次按键状态
    rckey.Now_State = Vkey & 0xffff;    // 如果按下是0，就是KEY_ENT = Vkey ^ 0xFFFF;
    rckey.Up_Down = rckey.Now_State & (rckey.Now_State ^ rckey.Last_State); // 检测按键是否首次按下
    rckey.Down_Up = rckey.Last_State & (rckey.Now_State ^ rckey.Last_State);
    rckey.Time_Now = hpm_csr_get_core_mcycle(); // 更新时间线 单位：ms
    rckey.Continue = 0;                         // 清除原有持续按下值
    for (int i = 0; i < RC_KEY_NUM; i++)
    {
        if (rckey.Now_State & (1 << i)) // 检测持续按下
        {
            if (rckey.Time_Flag[i] == 0)
                rckey.Time_Flag[i] = rckey.Time_Now + RC_KEY_DELAY;
        }
        else // 其他情况flag清零
        {
            rckey.Time_Flag[i] = 0;
        }
        if ((rckey.Time_Now >= rckey.Time_Flag[i]) && rckey.Time_Flag[i] != 0) // 时间超过或到达目标时间
        {
            rckey.Continue |= 1 << i;
            rckey.Time_Flag[i] = rckey.Time_Now + RC_KEY_WAIT; // 重置目标时间
        }
    }
}

/*
 *读取按键抬起按下过程
 *return：1 按下
 *读取后会清除状态
 */
int get_keyboard_up_down(uint16_t Rc_Key)
{
    uint16_t up_down;
    up_down = rckey.Up_Down & Rc_Key;
    // 读取后清除按键状态
    rckey.Up_Down ^= up_down;
    if (up_down)
        return 1;
    else
        return 0;
}

/*
 *检测按键按下抬起过程
 *读取后会清除该状态
 */
int get_keyboard_down_up(uint16_t Rc_Key)
{
    uint16_t down_up;
    down_up = rckey.Down_Up & Rc_Key;
    // 读取后清除按键状态
    rckey.Down_Up ^= down_up;
    if (down_up)
        return 1;
    else
        return 0;
}

/*
 *读取按键当前是否按下
 *return 1则正在按下
 */
int get_keyboard_now_state(uint16_t Rc_Key)
{
    uint16_t now_state;
    now_state = rckey.Now_State & Rc_Key;
    if (now_state)
        return 1;
    else
        return 0;
}

/*
 *读取按键长按间隔确认值，与上个函数的差别在于长按时每隔 RC_KEY_WAIT 才返回1。
 *读取后会清除状态
 */
int get_keyboard_cont_enter(uint16_t Rc_Key)
{
    uint16_t cont;
    cont = rckey.Continue & Rc_Key;
    // 读取后清除按键状态
    rckey.Continue ^= cont;
    if (cont)
        return 1;
    else
        return 0;
}

/*
 *按下抬起过程奇数次返回1，偶数次返回0
 *对于同一个按键get_keyboard_down_up 和 get_switch_keyboard_down_up 不能同时使用
 */
int get_switch_keyboard_down_up(uint16_t Rc_Key)
{
    static uint16_t switch_down_up = 0;
    switch_down_up ^= get_keyboard_down_up(Rc_Key);

    return ((switch_down_up & Rc_Key) == 0) ? 0 : 1;
}

/*
 *抬起按下过程奇数次返回1，偶数次返回0
 *对于同一个按键get_keyboard_up_down 和 get_switch_keyboard_up_down 不能同时使用
 */
int get_switch_keyboard_up_down(uint16_t Rc_Key)
{
    static uint16_t switch_up_down = 0;
    switch_up_down ^= get_keyboard_up_down(Rc_Key);

    return ((switch_up_down & Rc_Key) == 0) ? 0 : 1;
}
