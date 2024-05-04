#ifndef _CLIENT_UI_PLOT_H__
#define _CLIENT_UI_PLOT_H__

#include "referee_protocol.h"

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
extern void uiPlotLine(interaction_figure_t *figure, char figure_name[3], figure_operation_type_e operate_tpye,
                       uint32_t layer, figure_color_type_e color, uint32_t width, uint32_t start_x, uint32_t end_x,
                       uint32_t start_y, uint32_t end_y);

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
extern void uiPlotRectangle(interaction_figure_t *figure, char figure_name[3], figure_operation_type_e operate_tpye,
                            uint32_t layer, figure_color_type_e color, uint32_t width, uint32_t start_x, uint32_t end_x,
                            uint32_t start_y, uint32_t end_y);

extern void uiPlotArc(interaction_figure_t *figure, char figure_name[3], figure_operation_type_e operate_tpye,
                      uint32_t layer, figure_color_type_e color, uint32_t width, uint32_t start_x, uint32_t start_y,
                      uint32_t start_degree, uint32_t end_degree, uint32_t x_length, uint32_t y_length);

/**
 * @brief 绘制整形数
 *
 * @param figure 图形数据结构体
 * @param figure_name 图形名称，仅作为索引
 * @param operate_tpye 图形操作类型
 * @param layer 图层 0-9 9为最上层
 * @param color 图形颜色
 * @param width 图形线宽 1-9
 * @param start_x 起始点x坐标
 * @param start_y 起始点y坐标
 * @param value 显示的值
 */
extern void uiPlotIntNum(interaction_figure_t *figure, char figure_name[3], figure_operation_type_e operate_tpye,
                         uint32_t layer, figure_color_type_e color, uint32_t width, uint32_t start_x, uint32_t start_y,
                         int32_t value);

#endif /* _CLIENT_UI_PLOT_H__ */
