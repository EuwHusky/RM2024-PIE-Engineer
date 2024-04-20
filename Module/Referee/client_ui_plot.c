#include "string.h"

#include "client_ui_plot.h"

/**
 * @brief 绘制直线
 *
 * @param figure 图形数据结构体
 * @param figure_name 图形名称，仅作为索引
 * @param operate_tpye 图形操作类型
 * @param layer 图层 0-9 9为最上层
 * @param color 图形颜色
 * @param width 图形线宽 1-9
 * @param start_x 起始点x坐标
 * @param end_x 终点x坐标
 * @param start_y 起始点y坐标
 * @param end_y 终点y坐标
 */
void uiPlotLine(interaction_figure_t *figure, char figure_name[3], figure_operation_type_e operate_tpye, uint32_t layer,
                figure_color_type_e color, uint32_t width, uint32_t start_x, uint32_t end_x, uint32_t start_y,
                uint32_t end_y)
{
    memset(figure, 0, sizeof(interaction_figure_t));
    for (uint8_t i = 0; i < 3 && figure_name[i] != '\0'; i++)
        figure->figure_name[2 - i] = figure_name[i];
    figure->operate_tpye = operate_tpye;
    figure->layer = layer;
    figure->layer = figure->layer > 9 ? 9 : figure->layer;
    figure->color = color;
    figure->width = width;
    figure->width = figure->width > 9 ? 9 : figure->width;
    figure->start_x = start_x;
    figure->start_y = start_y;
    figure->details_d = end_x;
    figure->details_e = end_y;
}

/**
 * @brief 绘制矩形
 *
 * @param figure 图形数据结构体
 * @param figure_name 图形名称，仅作为索引
 * @param operate_tpye 图形操作类型
 * @param layer 图层 0-9 9为最上层
 * @param color 图形颜色
 * @param width 图形线宽 1-9
 * @param start_x 起始点x坐标
 * @param end_x 对顶点x坐标
 * @param start_y 起始点y坐标
 * @param end_y 对顶点y坐标
 */
void uiPlotRectangle(interaction_figure_t *figure, char figure_name[3], figure_operation_type_e operate_tpye,
                     uint32_t layer, figure_color_type_e color, uint32_t width, uint32_t start_x, uint32_t end_x,
                     uint32_t start_y, uint32_t end_y)
{
    memset(figure, 0, sizeof(interaction_figure_t));
    for (uint8_t i = 0; i < 3 && figure_name[i] != '\0'; i++)
        figure->figure_name[2 - i] = figure_name[i];
    figure->operate_tpye = operate_tpye;
    figure->figure_tpye = FIGURE_RECTANGLE;
    figure->layer = layer;
    figure->layer = figure->layer > 9 ? 9 : figure->layer;
    figure->color = color;
    figure->width = width;
    figure->width = figure->width > 9 ? 9 : figure->width;
    figure->start_x = start_x;
    figure->start_y = start_y;
    figure->details_d = end_x;
    figure->details_e = end_y;
}

// /************************************************绘制整圆*************************************************
// **参数：*figure Graph_Data类型变量指针，用于存放图形数据
//         imagename[3]   图片名称，用于标识更改
//         Graph_Operate   图片操作，见头文件
//         Graph_Color    图形颜色
//         Start_x,Start_y    圆心坐标
//         Graph_Radius  图形半径
// **********************************************************************************************************/
// void Circle_Draw(figure_data_struct_t *figure, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Color,
//                  uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius)
// {
//     int i;
//     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
//         figure->figure_name[2 - i] = imagename[i];
//     figure->figure_tpye = UI_Graph_Circle;
//     figure->operate_tpye = Graph_Operate;
//     figure->layer = 9; // 默认图层
//     figure->color = Graph_Color;
//     figure->width = 5; // 默认线宽
//     figure->start_x = Start_x;
//     figure->start_y = Start_y;
//     figure->radius = Graph_Radius;
// }

// /************************************************绘制正圆弧*************************************************
// **参数：*figure Graph_Data类型变量指针，用于存放图形数据
//         imagename[3]   图片名称，用于标识更改
//         Graph_Operate   图片操作，见头文件
//         Graph_Color    图形颜色
//         Start_x,Start_y    圆心坐标
//         start_angle,end_angle   圆弧开始结束度数
//         Graph_Radius  圆弧半径
// **********************************************************************************************************/
// void Arc_Draw(figure_data_struct_t *figure, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Color,
//               uint32_t start_angle, uint32_t end_angle, uint32_t Start_x, uint32_t Start_y, uint32_t Graph_Radius,
//               uint32_t width)
// {
//     int i;
//     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
//         figure->figure_name[2 - i] = imagename[i];
//     figure->figure_tpye = UI_Graph_Arc;
//     figure->operate_tpye = Graph_Operate;
//     figure->layer = 7; // 默认图层
//     figure->color = Graph_Color;
//     figure->width = width; // 默认线宽
//     figure->start_angle = start_angle;
//     figure->end_angle = end_angle;
//     figure->start_x = Start_x;
//     figure->start_y = Start_y;
//     figure->end_x = Graph_Radius;
//     figure->end_y = Graph_Radius;
// }

// /************************************************绘制字符*************************************************
//  参数:*figure Graph_Data类型变量指针，用于存放图形数据
//          imagename[3]   图片名称，用于标识更改
//          Graph_Operate  图片操作，见头文件
//          Graph_Color    图形颜色
//          Start_x,Start_y    开始坐标
//          *Char_Data         待发送字符串开始地址
// **********************************************************************************************************/
// void Char_Draw(ext_client_custom_character_t *figure, char imagename[3], uint32_t Graph_Operate, uint32_t
// Graph_Color,
//                uint32_t Start_x, uint32_t Start_y, char *Char_Data, uint32_t Graph_Size)
// {
//     int i;

//     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
//         figure->grapic_data_struct.figure_name[2 - i] = imagename[i];
//     figure->grapic_data_struct.figure_tpye = UI_Graph_Char;
//     figure->grapic_data_struct.operate_tpye = Graph_Operate;
//     figure->grapic_data_struct.layer = 9; // 默认图层
//     figure->grapic_data_struct.color = Graph_Color;
//     figure->grapic_data_struct.width = 4; // 默认线宽
//     figure->grapic_data_struct.start_x = Start_x;
//     figure->grapic_data_struct.start_y = Start_y;
//     figure->grapic_data_struct.start_angle = 30; // 默认字号
//     figure->grapic_data_struct.end_angle = Graph_Size;

//     for (i = 0; i < figure->grapic_data_struct.end_angle; i++)
//     {
//         figure->data[i] = *Char_Data;
//         Char_Data++;
//     }
// }
// /******************************************绘制浮点型数据*************************************************
// **参数：*figure Graph_Data类型变量指针，用于存放图形数据
//         imagename[3]   图片名称，用于标识更改
//         Graph_Operate   图片操作，见头文件
//         Graph_Color    图形颜色
//         Start_x、Start_x    开始坐标
//         Graph_Float   要显示的变量
// **********************************************************************************************************/
// void Float_Draw(figure_data_struct_t *figure, char imagename[3], uint32_t Graph_Operate, uint32_t Graph_Color,
//                 uint32_t Start_x, uint32_t Start_y, int32_t Graph_Float)
// {
//     int i;

//     for (i = 0; i < 3 && imagename[i] != '\0'; i++)
//         figure->figure_name[2 - i] = imagename[i];
//     figure->figure_tpye = UI_Graph_Float;
//     figure->operate_tpye = Graph_Operate;
//     figure->layer = 9; // 默认图层
//     figure->color = Graph_Color;
//     figure->width = 3; // 默认线宽
//     figure->start_x = Start_x;
//     figure->start_y = Start_y;
//     figure->start_angle = 30; // 默认字号
//     figure->end_angle = 3;    // 默认小数位数
//     //   memcpy((void *)figure->radius, (void *)&Graph_Float, 4);0
//     if (Graph_Float > 2145387000)
//         Graph_Float = 2145387000;
//     else if (Graph_Float < -2145387000)
//         Graph_Float = -2145387000;
//     if (Graph_Float >= 0)
//     {
//         figure->end_y = Graph_Float / 2097152;
//         figure->end_x = (Graph_Float - figure->end_y * 2097152) / 1024;
//         figure->radius = Graph_Float - figure->end_y * 2097152 - figure->end_x * 1024;
//     }
//     else
//     {
//         Graph_Float = -Graph_Float;
//         figure->end_y = 2047 - Graph_Float / 2097152;
//         figure->end_x = 2047 - (Graph_Float - figure->end_y * 2097152) / 1024;
//         figure->radius = 1024 - (Graph_Float - figure->end_y * 2097152 - figure->end_x * 1024);
//     }
// }
