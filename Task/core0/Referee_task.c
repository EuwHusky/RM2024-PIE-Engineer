#include "hpm_dma_drv.h"
#include "hpm_dmamux_drv.h"
#include "hpm_uart_drv.h"

#include "FreeRTOS.h"
#include "task.h"

#include "algo_robomaster_referee_protocol.h"

#include "CRC8_CRC16.h"
#include "INS_task.h"
#include "detect_task.h"
#include "fifo.h"
#include "referee_task.h"

// 裁判系统串口初始化
static void pm_uart_init(void);
// 图传链路串口初始化
static void vt_uart_init(void);
// DMA接收&发送触发函数
static hpm_stat_t uart_rx_trigger_dma(DMA_Type *dma_ptr, uint8_t ch_num, UART_Type *uart_ptr, uint32_t dst,
                                      uint32_t size);
static hpm_stat_t uart_tx_trigger_dma(DMA_Type *dma_ptr, uint8_t ch_num, UART_Type *uart_ptr, uint32_t src,
                                      uint32_t size);
// 裁判系统数据解包
static void referee_unpack_fifo_data(referee_link_type_e referee_link_type);
// 自定义UI相关操作
// static void UI_Delate(uint8_t Del_Operate, uint8_t Del_Layer);
// static void Line_Draw(graphic_data_struct_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Color,
//                       uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
// static void Rectangle_Draw(graphic_data_struct_t *image, char imagename[3], uint32_t Graph_Operate,
//                            uint32_t Graph_Color, uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y);
// static void Circle_Draw(graphic_data_struct_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t
// Graph_Color,
//                         uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius);
// static void Arc_Draw(graphic_data_struct_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Color,
//                      uint32_t start_angle, uint32_t end_angle, uint32_t Start_x, uint32_t Start_y,
//                      uint32_t Graph_Radius, uint32_t width);
// static void Float_Draw(graphic_data_struct_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Color,
//                        uint32_t Start_x, uint32_t Start_y, int32_t Graph_Float);
// static void Char_Draw(ext_client_custom_character_t *image, char imagename[3], uint32_t Graph_Operate,
//                       uint32_t Graph_Color, uint32_t Start_x, uint32_t Start_y, char *Char_Data, uint32_t
//                       Graph_Size);
// static int UI_ReFresh_picture(Refreedata_UI *refreedata_UI);
// static int UI_ReFresh_char(ext_client_custom_character_t *char_data);

/*----------------------------------------*/
// 电管链路裁判系统数据相关变量和结构
ATTR_PLACE_AT_NONCACHEABLE uint8_t pm_rx_buf[PM_UART_RX_BUF_LENGHT]; // 接收原始数据
volatile bool pm_uart_rx_dma_done = true;                            // dma传输完成标志位
fifo_s_t pm_uart_fifo;                                               // FIFO结构体
uint8_t pm_uart_fifo_buf[PM_UART_FIFO_BUF_LENGTH];                   // FIFO缓存数组

// 图传链路裁判系统数据相关变量和结构
ATTR_PLACE_AT_NONCACHEABLE uint8_t vt_rx_buf[VT_UART_RX_BUF_LENGHT]; // 接收原始数据
volatile bool vt_uart_rx_dma_done = true;                            // dma传输完成标志位
fifo_s_t vt_uart_fifo;                                               // FIFO结构体
uint8_t vt_uart_fifo_buf[VT_UART_FIFO_BUF_LENGTH];                   // FIFO缓存数组

unpack_data_t referee_unpack_obj; // 数据解包结构体

static Referee_data_t referee_data; // 需要用到的裁判系统数据结构体
/*----------------------------------------*/
uint16_t Robot_ID;
uint16_t Cilent_ID;
uint8_t UI_Seq = 0;   // 包序号
uint8_t UI_data[128]; // 发送数据长度

// 裁判系统串口任务
void referee_task(void *pvParameters)
{
    while (!INS_init_finished)
        vTaskDelay(10);
    vTaskDelay(200);

    pm_uart_init();                                                        // 初始化裁判系统串口
    fifo_s_init(&pm_uart_fifo, pm_uart_fifo_buf, PM_UART_FIFO_BUF_LENGTH); // FIFO初始化

    vt_uart_init();                                                        // 初始化图传链路串口
    fifo_s_init(&vt_uart_fifo, vt_uart_fifo_buf, VT_UART_FIFO_BUF_LENGTH); // FIFO初始化

    init_referee_struct_data(); // 初始化裁判系统数据结构体

    while (1)
    {
        if (pm_uart_rx_dma_done)
        {
            pm_uart_rx_dma_done = false;
            referee_unpack_fifo_data(PM_REFEREE_LINK);
            uart_rx_trigger_dma(BOARD_HDMA, PM_UART_RX_DMA_CHN, PM_UART,
                                core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)pm_rx_buf),
                                PM_UART_RX_BUF_LENGHT);
        }

        if (vt_uart_rx_dma_done)
        {
            vt_uart_rx_dma_done = false;
            referee_unpack_fifo_data(VT_REFEREE_LINK);
            uart_rx_trigger_dma(BOARD_HDMA, VT_UART_RX_DMA_CHN, VT_UART,
                                core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)vt_rx_buf),
                                VT_UART_RX_BUF_LENGHT);
        }

        // refree_data.robot_id = get_robot_id();
        // Cilent_ID = refree_data.robot_id + 0x100;
        // refree_data.now_power = get_chassis_power();
        // refree_data.power_buffer = get_chassis_power_buffer();
        // refree_data.power_limit = get_chassis_power_limit();
        // refree_data.shoot_speed = get_shoot_speed();
        // refree_data.shoot_speed_limit = get_shoot_heat0_speed_limit();
        // refree_data.heat = get_shooter_cooling_heat();
        // refree_data.heat_limit = get_shoot_heat0_limit();

        // printf("%d,%f,%d\n", Robot_ID, refree_data.now_power, refree_data.power_limit);

        vTaskDelay(20);
    }
}

// // DMA中断回调函数
// void referee_dma_isr(void)
// {
//     volatile hpm_stat_t stat_rx_chn;

//     if (stat_rx_chn = dma_check_transfer_status(BOARD_HDMA, PM_UART_RX_DMA_CHN), stat_rx_chn & DMA_CHANNEL_STATUS_TC)
//     {
//         fifo_s_puts(&pm_uart_fifo, (char *)pm_rx_buf, PM_UART_RX_BUF_LENGHT);
//         pm_uart_rx_dma_done = true; // 更新标志位
//         detect_hook(PM_REFEREE_DH);
//     }
//     else if (stat_rx_chn = dma_check_transfer_status(BOARD_HDMA, VT_UART_RX_DMA_CHN),
//              stat_rx_chn & DMA_CHANNEL_STATUS_TC)
//     {
//         fifo_s_puts(&vt_uart_fifo, (char *)vt_rx_buf, VT_UART_RX_BUF_LENGHT);
//         vt_uart_rx_dma_done = true; // 更新标志位
//         detect_hook(VT_REFEREE_DH);
//     }
// }
// SDK_DECLARE_EXT_ISR_M(BOARD_HDMA_IRQ, referee_dma_isr)

// 裁判系统串口初始化
void pm_uart_init(void)
{
    uart_config_t config = {0}; // 串口配置
    board_init_uart(PM_UART);
    uart_default_config(PM_UART, &config);                    // 填充默认配置
    config.fifo_enable = true;                                // 使能FIFO
    config.dma_enable = true;                                 // 使能DMA
    config.baudrate = PM_BAUDRATE;                            // 设置波特率
    config.src_freq_in_hz = clock_get_frequency(PM_UART_CLK); // 获得时钟频率
    config.rx_fifo_level = uart_rx_fifo_trg_not_empty;
    if (uart_init(PM_UART, &config) != status_success)
    {
        printf("failed to initialize uart\n");
        while (1)
        {
        }
    }
    intc_m_enable_irq_with_priority(BOARD_HDMA_IRQ, 1);
    dmamux_config(BOARD_DMAMUX, PM_UART_RX_DMAMUX_CHN, PM_UART_RX_DMA_REQ, true);
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
}

Referee_data_t *get_referee_data_pointer(void)
{
    return &referee_data;
}

/* 裁判系统数据解包 */
void referee_unpack_fifo_data(referee_link_type_e referee_link_type)
{
    uint8_t byte = 0;
    uint8_t sof = HEADER_SOF; // 帧头
    unpack_data_t *p_obj = &referee_unpack_obj;
    fifo_s_t *p_fifo = NULL;

    if (referee_link_type == PM_REFEREE_LINK)
        p_fifo = &pm_uart_fifo;
    else if (referee_link_type == VT_REFEREE_LINK)
        p_fifo = &vt_uart_fifo;

    while (fifo_s_used(p_fifo))
    {
        byte = fifo_s_get(p_fifo);
        switch (p_obj->unpack_step)
        {
        case STEP_HEADER_SOF: {
            if (byte == sof)
            {
                p_obj->unpack_step = STEP_LENGTH_LOW;
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            else
            {
                p_obj->index = 0;
            }
        }
        break;

        case STEP_LENGTH_LOW: {
            p_obj->data_len = byte;
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_LENGTH_HIGH;
        }
        break;

        case STEP_LENGTH_HIGH: {
            p_obj->data_len |= (byte << 8);
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->data_len < (REF_PROTOCOL_FRAME_MAX_SIZE - REF_HEADER_CRC_CMDID_LEN))
            {
                p_obj->unpack_step = STEP_FRAME_SEQ;
            }
            else
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;
            }
        }
        break;
        case STEP_FRAME_SEQ: {
            p_obj->protocol_packet[p_obj->index++] = byte;
            p_obj->unpack_step = STEP_HEADER_CRC8;
        }
        break;

        case STEP_HEADER_CRC8: {
            p_obj->protocol_packet[p_obj->index++] = byte;

            if (p_obj->index == REF_PROTOCOL_HEADER_SIZE)
            {
                if (verify_CRC8_check_sum(p_obj->protocol_packet, REF_PROTOCOL_HEADER_SIZE))
                {
                    p_obj->unpack_step = STEP_DATA_CRC16;
                }
                else
                {
                    p_obj->unpack_step = STEP_HEADER_SOF;
                    p_obj->index = 0;
                }
            }
        }
        break;

        case STEP_DATA_CRC16: {
            if (p_obj->index < (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
            {
                p_obj->protocol_packet[p_obj->index++] = byte;
            }
            if (p_obj->index >= (REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
            {
                p_obj->unpack_step = STEP_HEADER_SOF;
                p_obj->index = 0;

                if (verify_CRC16_check_sum(p_obj->protocol_packet, REF_HEADER_CRC_CMDID_LEN + p_obj->data_len))
                {
                    referee_data_solve(p_obj->protocol_packet);
                }
            }
        }
        break;

        default: {
            p_obj->unpack_step = STEP_HEADER_SOF;
            p_obj->index = 0;
        }
        break;
        }
    }
}

// int UI_ReFresh_char(ext_client_custom_character_t *char_data) // UI字符型刷新推送
// {
//     uint8_t *framepoint;                         // memcpy连续拷贝指针1
//     uint8_t *framepoint2;                        // memcpy连续拷贝指针2
//     UI_Packhead ui_packhead;                     // UI数据帧头
//     ext_student_interactive_header_data_t ui_id; // ui发送数据id配置
//     UI_Refresh_char character;

//     framepoint = (uint8_t *)&character;
//     ui_packhead.SOF = UI_SOF;         // 帧头固定0XA5
//     ui_packhead.Data_Length = 6 + 45; // 数据段长度
//     ui_packhead.Seq = UI_Seq;         // 包序号

//     memcpy(&character.ch, &ui_packhead, 4);

//     ui_packhead.CRC8 = get_CRC8_check_sum(character.ch, 4, 0XFF); // 校验帧头前四位
//     memset(&character.ch, 0, sizeof(character.ch));
//     ui_packhead.CMD_ID = UI_CMD_Robo_Exchange; // 命令ID
//     memcpy(framepoint, &ui_packhead, 7);       // 填充CRC8校验值与命令ID

//     ui_id.data_cmd_id = UI_Data_ID_DrawChar;
//     ui_id.sender_ID = Robot_ID;
//     ui_id.receiver_ID = Cilent_ID; // 填充操作信息，发送ID，接收ID
//     memcpy(framepoint + sizeof(ui_packhead), &ui_id, sizeof(ui_id));

//     framepoint2 = (unsigned char *)&char_data->grapic_data_struct;
//     memcpy(framepoint + sizeof(ui_packhead) + sizeof(ui_id), framepoint2,
//            sizeof(char_data->grapic_data_struct)); // 填充图形操作信息

//     framepoint2 = (unsigned char *)&char_data->data;
//     memcpy(framepoint + sizeof(ui_packhead) + sizeof(ui_id) + sizeof(char_data->grapic_data_struct), framepoint2,
//            sizeof(char_data->data));          // 填充图形操作信息
//     append_CRC16_check_sum(character.ch, 60); // CRC16整包校验

//     memcpy(UI_data, character.ch, 60);
//     // usart6_tx_dma_enable(UI_data, 60);
//     UI_Seq++; // 包序号+1
//     return UI_Seq;
// }

// int UI_ReFresh_picture(Refreedata_UI *refreedata_UI)
// {
//     UI_Packhead ui_packhead;                     // UI数据帧头
//     ext_student_interactive_header_data_t ui_id; // ui发送数据id配置

//     ui_packhead.SOF = UI_SOF;                              // 帧头固定0XA5
//     ui_packhead.Data_Length = 6 + refreedata_UI->num * 15; // 数据段长度
//     ui_packhead.Seq = UI_Seq;                              // 包序号

//     memcpy(refreedata_UI->Picture_Data, &ui_packhead, 4);

//     ui_packhead.CRC8 = get_CRC8_check_sum(refreedata_UI->Picture_Data, 4, 0XFF); // 校验帧头前四位
//     memset(refreedata_UI->Picture_Data, 0, sizeof(refreedata_UI->Picture_Data));
//     ui_packhead.CMD_ID = UI_CMD_Robo_Exchange;            // 命令ID
//     memcpy(refreedata_UI->Picture_Data, &ui_packhead, 7); // 填充CRC8校验值与命令ID

//     if (refreedata_UI->num == 1)
//         ui_id.data_cmd_id = UI_Data_ID_Draw1;
//     else if (refreedata_UI->num == 2)
//         ui_id.data_cmd_id = UI_Data_ID_Draw2;
//     else if (refreedata_UI->num == 5)
//         ui_id.data_cmd_id = UI_Data_ID_Draw5;
//     else if (refreedata_UI->num == 7)
//         ui_id.data_cmd_id = UI_Data_ID_Draw7;

//     ui_id.sender_ID = Robot_ID;
//     ui_id.receiver_ID = Cilent_ID; // 填充操作信息，发送ID，接收ID
//     memcpy(refreedata_UI->Picture_Data + sizeof(ui_packhead), &ui_id, sizeof(ui_id));

//     for (uint8_t i = 0; i < refreedata_UI->num; i++)
//         memcpy(refreedata_UI->Picture_Data + sizeof(ui_packhead) + sizeof(ui_id) + i * sizeof(graphic_data_struct_t),
//                refreedata_UI->graphic_data[i], sizeof(graphic_data_struct_t)); // 填充图形操作信息

//     append_CRC16_check_sum(refreedata_UI->Picture_Data, 15 * (refreedata_UI->num + 1)); // CRC16整包校验
//     memcpy(UI_data, refreedata_UI->Picture_Data, 15 * (refreedata_UI->num + 1));
//     // usart6_tx_dma_enable(UI_data, 15 * (refreedata_UI->num + 1));
//     UI_Seq++; // 包序号+1
//     return UI_Seq;
// }
// /************************************************绘制直线*************************************************
// **参数:*image Graph_Data 类型变量指针，用于存放数据
//         imagename[3]     图片名称,用于表示更改
//         Graph_Operate    图片操作
//         Graph_Color      图形颜色
//         Start_x,Start_y  起始坐标
//         End_x,End_y      中止坐标
// **********************************************************************************************************/
// void Line_Draw(graphic_data_struct_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Color,
//                uint32_t Start_x, uint32_t Start_y, uint32_t End_x, uint32_t End_y)
// {
//     int i;
//     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
//         image->graphic_name[2 - i] = imagename[i];
//     image->operate_tpye = Graph_Operate;
//     image->layer = 9; // 操作图层默认为9
//     image->color = Graph_Color;
//     image->width = 2; // 线宽默认为2
//     image->start_x = Start_x;
//     image->start_y = Start_y;
//     image->end_x = End_x;
//     image->end_y = End_y;
// }
// /************************************************绘制矩形*************************************************
// **参数：*image Graph_Data类型变量指针，用于存放图形数据
//         imagename[3]   图片名称，用于标识更改
//         Graph_Operate   图片操作，见头文件
//         Graph_Color    图形颜色
//         Start_x、Start_y    开始坐标
//         End_x、End_y   结束坐标（对顶角坐标）
// **********************************************************************************************************/
// void Rectangle_Draw(graphic_data_struct_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Color,
//                     uint32_t Start_x, uint32_t End_x, uint32_t Start_y, uint32_t End_y)
// {
//     int i;
//     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
//         image->graphic_name[2 - i] = imagename[i];
//     image->graphic_tpye = UI_Graph_Rectangle;
//     image->operate_tpye = Graph_Operate;
//     image->layer = 9; // 默认图层
//     image->color = Graph_Color;
//     image->width = 5; // 默认线宽
//     image->start_x = Start_x;
//     image->start_y = Start_y;
//     image->end_x = End_x;
//     image->end_y = End_y;
// }
// /************************************************绘制整圆*************************************************
// **参数：*image Graph_Data类型变量指针，用于存放图形数据
//         imagename[3]   图片名称，用于标识更改
//         Graph_Operate   图片操作，见头文件
//         Graph_Color    图形颜色
//         Start_x,Start_y    圆心坐标
//         Graph_Radius  图形半径
// **********************************************************************************************************/
// void Circle_Draw(graphic_data_struct_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Color,
//                  uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius)
// {
//     int i;
//     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
//         image->graphic_name[2 - i] = imagename[i];
//     image->graphic_tpye = UI_Graph_Circle;
//     image->operate_tpye = Graph_Operate;
//     image->layer = 9; // 默认图层
//     image->color = Graph_Color;
//     image->width = 5; // 默认线宽
//     image->start_x = Start_x;
//     image->start_y = Start_y;
//     image->radius = Graph_Radius;
// }

// /************************************************绘制正圆弧*************************************************
// **参数：*image Graph_Data类型变量指针，用于存放图形数据
//         imagename[3]   图片名称，用于标识更改
//         Graph_Operate   图片操作，见头文件
//         Graph_Color    图形颜色
//         Start_x,Start_y    圆心坐标
//         start_angle,end_angle   圆弧开始结束度数
//         Graph_Radius  圆弧半径
// **********************************************************************************************************/
// void Arc_Draw(graphic_data_struct_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Color,
//               uint32_t start_angle, uint32_t end_angle, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius,
//               uint32_t width)
// {
//     int i;
//     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
//         image->graphic_name[2 - i] = imagename[i];
//     image->graphic_tpye = UI_Graph_Arc;
//     image->operate_tpye = Graph_Operate;
//     image->layer = 7; // 默认图层
//     image->color = Graph_Color;
//     image->width = width; // 默认线宽
//     image->start_angle = start_angle;
//     image->end_angle = end_angle;
//     image->start_x = Start_x;
//     image->start_y = Start_y;
//     image->end_x = Graph_Radius;
//     image->end_y = Graph_Radius;
// }

// /************************************************绘制字符*************************************************
// 参数:*image Graph_Data类型变量指针，用于存放图形数据
//         imagename[3]   图片名称，用于标识更改
//         Graph_Operate  图片操作，见头文件
//         Graph_Color    图形颜色
//         Start_x,Start_y    开始坐标
//         *Char_Data         待发送字符串开始地址
// **********************************************************************************************************/
// void Char_Draw(ext_client_custom_character_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Color,
//                uint32_t Start_x, uint32_t Start_y, char *Char_Data, uint32_t Graph_Size)
// {
//     int i;

//     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
//         image->grapic_data_struct.graphic_name[2 - i] = imagename[i];
//     image->grapic_data_struct.graphic_tpye = UI_Graph_Char;
//     image->grapic_data_struct.operate_tpye = Graph_Operate;
//     image->grapic_data_struct.layer = 9; // 默认图层
//     image->grapic_data_struct.color = Graph_Color;
//     image->grapic_data_struct.width = 4; // 默认线宽
//     image->grapic_data_struct.start_x = Start_x;
//     image->grapic_data_struct.start_y = Start_y;
//     image->grapic_data_struct.start_angle = 30; // 默认字号
//     image->grapic_data_struct.end_angle = Graph_Size;

//     for (i = 0; i < image->grapic_data_struct.end_angle; i++)
//     {
//         image->data[i] = *Char_Data;
//         Char_Data++;
//     }
// }
// /******************************************绘制浮点型数据*************************************************
// **参数：*image Graph_Data类型变量指针，用于存放图形数据
//         imagename[3]   图片名称，用于标识更改
//         Graph_Operate   图片操作，见头文件
//         Graph_Color    图形颜色
//         Start_x、Start_x    开始坐标
//         Graph_Float   要显示的变量
// **********************************************************************************************************/
// void Float_Draw(graphic_data_struct_t *image, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Color,
//                 uint32_t Start_x, uint32_t Start_y, int32_t Graph_Float)
// {
//     int i;

//     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
//         image->graphic_name[2 - i] = imagename[i];
//     image->graphic_tpye = UI_Graph_Float;
//     image->operate_tpye = Graph_Operate;
//     image->layer = 9; // 默认图层
//     image->color = Graph_Color;
//     image->width = 3; // 默认线宽
//     image->start_x = Start_x;
//     image->start_y = Start_y;
//     image->start_angle = 30; // 默认字号
//     image->end_angle = 3;    // 默认小数位数
//     //   memcpy((void *)image->radius, (void *)&Graph_Float, 4);0
//     if (Graph_Float > 2145387000)
//         Graph_Float = 2145387000;
//     else if (Graph_Float < -2145387000)
//         Graph_Float = -2145387000;
//     if (Graph_Float >= 0)
//     {
//         image->end_y = Graph_Float / 2097152;
//         image->end_x = (Graph_Float - image->end_y * 2097152) / 1024;
//         image->radius = Graph_Float - image->end_y * 2097152 - image->end_x * 1024;
//     }
//     else
//     {
//         Graph_Float = -Graph_Float;
//         image->end_y = 2047 - Graph_Float / 2097152;
//         image->end_x = 2047 - (Graph_Float - image->end_y * 2097152) / 1024;
//         image->radius = 1024 - (Graph_Float - image->end_y * 2097152 - image->end_x * 1024);
//     }
// }

// dma接收触发函数
hpm_stat_t uart_rx_trigger_dma(DMA_Type *dma_ptr, uint8_t ch_num, UART_Type *uart_ptr, uint32_t dst, uint32_t size)
{
    dma_handshake_config_t config;
    dma_default_handshake_config(dma_ptr, &config);
    config.ch_index = ch_num;
    config.dst = dst;
    config.dst_fixed = false;
    config.src = (uint32_t)&uart_ptr->RBR;
    config.src_fixed = true;
    config.data_width = DMA_TRANSFER_WIDTH_BYTE;
    config.size_in_byte = size;
    return dma_setup_handshake(dma_ptr, &config, true);
}
// dma发送触发函数
hpm_stat_t uart_tx_trigger_dma(DMA_Type *dma_ptr, uint8_t ch_num, UART_Type *uart_ptr, uint32_t src, uint32_t size)
{
    dma_handshake_config_t config;
    dma_default_handshake_config(dma_ptr, &config);
    config.ch_index = ch_num;
    config.dst = (uint32_t)&uart_ptr->THR;
    config.dst_fixed = true;
    config.src = src;
    config.src_fixed = false;
    config.data_width = DMA_TRANSFER_WIDTH_BYTE;
    config.size_in_byte = size;
    return dma_setup_handshake(dma_ptr, &config, true);
}
