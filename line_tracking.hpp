#pragma once

#include "zf_common_headfile.hpp"
#include "zf_device_ips200_fb.hpp"
#include "lane_types.h"
#include "preprocess.h"
#include "lane_track_eight_neighborhood.h"

// SPI 屏幕分辨率：240x320 (framebuffer)
static constexpr uint16 LCD_W = 240;
static constexpr uint16 LCD_H = 320;

// 说明：当前工程的巡线算法直接使用旋转 180° 后的灰度图作为输入（显示与算法一致）。

// 视觉线程输出底部中心线误差，控制线程据此生成左右轮目标转速
// 图像在 line_tracking.cpp 中已旋转 180°；近端车道若移到画面上方，可把本值改小（例如 0～40）
static constexpr int32 LINE_FOLLOW_ROW_BEGIN = IMAGE_HEIGHT - 55;      // 参与底部平均的起始行（取底部60行）
static constexpr int32 LCD_LINE_WIDTH_SCALE = 5;                        // 显示宽度相对当前实现放大倍数
static constexpr int32 LCD_BASE_LINE_WIDTH = 2;                         // 当前实现为2像素（原点+右侧1点）
static constexpr int32 LCD_DRAW_LINE_WIDTH = LCD_BASE_LINE_WIDTH * LCD_LINE_WIDTH_SCALE;

// err_x 符号约定：
// 控制层采用 target_left = base + steer, target_right = base - steer；
// 在该约定下，steer>0 会让车“向右转”。若你观察到“线在左侧却向右修正”，把本值设为 -1 翻转误差符号。
static constexpr int32 ERR_X_SIGN = -1;

// TFT 显示内容选择：
// 0：显示旋转180°后的原始灰度图（当前默认行为）
// 1：显示开闭运算 + 连通域去噪 + 清边界后的二值图（preprocess_run 输出）
static constexpr int32 TFT_SHOW_STAGE = 1;

// 对外：传入摄像头原始灰度图指针，在 LCD 上叠加赛道信息，并返回底部中心线误差（err_x）
void line_tracking_process_frame(zf_device_ips200 &lcd,
                                 uint8_t *gray_ptr,
                                 int32 &err_x_out);
