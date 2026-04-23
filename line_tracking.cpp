#include "line_tracking.hpp"
#include <algorithm>
#include <cmath>

static GrayImage g_gray_frame = {};
static BinaryImage g_binary_frame = {};
static LaneResult g_lane_result = {};
static_assert((IMAGE_WIDTH * IMAGE_HEIGHT) == (UVC_WIDTH * UVC_HEIGHT),
              "Line tracking expects same pixel count after rotation.");

namespace {

static inline int32 clamp_int32(int32 x, int32 lo, int32 hi)
{
    if(x < lo) return lo;
    if(x > hi) return hi;
    return x;
}

static void swap_lane_left_right(LaneResult &res)
{
    for(int row = 0; row < IMAGE_HEIGHT; ++row)
    {
        std::swap(res.left_border[row], res.right_border[row]);
    }
    for(int i = 0; i < TRACK_MAX_POINTS; ++i)
    {
        std::swap(res.left_points[i], res.right_points[i]);
    }
    std::swap(res.left_points_count, res.right_points_count);
    std::swap(res.left_start, res.right_start);
}

} // namespace

void line_tracking_process_frame(zf_device_ips200 &lcd,
                                 uint8_t *gray_ptr,
                                 int32 &err_x_out)
{
    err_x_out = 0;
    if(nullptr == gray_ptr)
    {
        return;
    }

    // 摄像头图像先旋转 180°（修正倒装），显示与后续算法映射都基于该图
    static uint8_t rot_gray[UVC_WIDTH * UVC_HEIGHT];
    for(int y = 0; y < UVC_HEIGHT; y++)
    {
        for(int x = 0; x < UVC_WIDTH; x++)
        {
            const int sx = UVC_WIDTH - 1 - x;
            const int sy = UVC_HEIGHT - 1 - y;
            rot_gray[y * UVC_WIDTH + x] = gray_ptr[sy * UVC_WIDTH + sx];
        }
    }
    // 显示源图：旋转后的灰度图
    uint8_t *const src_gray_disp = rot_gray;

    // ================= Line_Tracking 循迹计算 =================
    // 算法内部坐标系为 120x160（宽x高），等价于把 160x120 的图像转置（旋转 90°）。
    // 这样算法的“底部”(y=159)对应原图的 x=159 这一侧，实现“以160作为底部向上搜索”。
    for(int y = 0; y < IMAGE_HEIGHT; y++)      // y: 0..159  <- 原图 x: 0..159
    {
        for(int x = 0; x < IMAGE_WIDTH; x++)   // x: 0..119  <- 原图 y: 0..119
        {
            g_gray_frame.data[y][x] = src_gray_disp[x * UVC_WIDTH + y];
        }
    }

    preprocess_run(&g_gray_frame, &g_binary_frame);
    lane_track_eight_neighborhood_run(&g_binary_frame, &g_lane_result);
    swap_lane_left_right(g_lane_result);

    // ============== 摄像头采集 + 屏幕显示（缩放） ==============
    static uint8_t disp_gray[LCD_W * LCD_H];
    for(uint16 y = 0; y < LCD_H; y++)
    {
        const uint16 y_s = (uint16)((uint32)y * (uint32)UVC_HEIGHT / (uint32)LCD_H);
        for(uint16 x = 0; x < LCD_W; x++)
        {
            const uint16 x_s = (uint16)((uint32)x * (uint32)UVC_WIDTH / (uint32)LCD_W);
            // 逆映射回显示坐标：src(y_s, x_s) <-> binary(row=x_s, col=y_s)
            disp_gray[y * LCD_W + x] = g_binary_frame.data[x_s][y_s];
        }
    }

    // 关键：show_gray_image 里真正用的是 width/height，不是 dis_width/dis_height
    // 所以这里 width/height 必须传 LCD 的大小，才能把画面“铺满”
    lcd.show_gray_image(0, 0,
                        disp_gray,
                        LCD_W,
                        LCD_H,
                        LCD_W,
                        LCD_H,
                        0);   // threshold = 0 表示按灰度显示

    // ============== 在屏幕上叠加：赛道边界 + 中心拟合线 ==============
    // 算法坐标：120x160（宽x高）；屏幕显示的是旋转180°后的 160x120 图像。
    // 映射关系（算法点 -> 原图点）：(x_alg, y_alg) -> (x_src = y_alg, y_src = x_alg)
    // err_x 统计窗口固定为 100±3 行
    const int row_begin = (int)clamp_int32(103 - 3, 0, IMAGE_HEIGHT - 1);
    const int row_end = (int)clamp_int32(103 + 3, row_begin, IMAGE_HEIGHT - 1);
    // row_begin 是算法的 y（对应原图 x），在屏幕上表现为一条竖线
    const uint16 x_lcd_row_begin = (uint16)((uint32)row_begin * (uint32)LCD_W / (uint32)UVC_WIDTH);
    for(uint16 y = 0; y < LCD_H; ++y)
    {
        lcd.draw_point(x_lcd_row_begin, y, RGB565_YELLOW);
    }
    for(int row = 0; row < IMAGE_HEIGHT; ++row) // row: y_alg
    {
        // 原图坐标：x_src=row, y_src=border(row)
        const uint16 x_lcd = (uint16)((uint32)row * (uint32)LCD_W / (uint32)UVC_WIDTH);
        const uint16 y_lcd_l = (uint16)((uint32)g_lane_result.left_border[row] * (uint32)LCD_H / (uint32)UVC_HEIGHT);
        const uint16 y_lcd_r = (uint16)((uint32)g_lane_result.right_border[row] * (uint32)LCD_H / (uint32)UVC_HEIGHT);
        const uint8 center_to_draw = g_lane_result.center_line[row];
        const uint16 y_lcd_c = (uint16)((uint32)center_to_draw * (uint32)LCD_H / (uint32)UVC_HEIGHT);

        // 将显示宽度调整为当前实现的5倍：2px -> 10px
        const int32 half = LCD_DRAW_LINE_WIDTH / 2;
        for(int32 dx = -half; dx < (LCD_DRAW_LINE_WIDTH - half); ++dx)
        {
            const int32 yl = (int32)y_lcd_l + dx;
            const int32 yr = (int32)y_lcd_r + dx;
            const int32 yc = (int32)y_lcd_c + dx;

            if(yl >= 0 && yl < LCD_H) lcd.draw_point(x_lcd, (uint16)yl, RGB565_GREEN);
            if(yr >= 0 && yr < LCD_H) lcd.draw_point(x_lcd, (uint16)yr, RGB565_RED);
            if(yc >= 0 && yc < LCD_H) lcd.draw_point(x_lcd, (uint16)yc, RGB565_CYAN);
        }
    }

    // 起始点标记（可选）
    if(g_lane_result.start_found)
    {
        const int lsx = (int)g_lane_result.left_start.x;
        const int lsy = (int)g_lane_result.left_start.y;
        const int rsx = (int)g_lane_result.right_start.x;
        const int rsy = (int)g_lane_result.right_start.y;

        // start 点坐标同样是算法坐标，映射到屏幕显示坐标
        if(lsx >= 0 && lsy >= 0)
        {
            const uint16 x_lcd = (uint16)((uint32)lsy * (uint32)LCD_W / (uint32)UVC_WIDTH);
            const uint16 y_lcd = (uint16)((uint32)lsx * (uint32)LCD_H / (uint32)UVC_HEIGHT);
            lcd.draw_point(x_lcd, y_lcd, RGB565_YELLOW);
        }
        if(rsx >= 0 && rsy >= 0)
        {
            const uint16 x_lcd = (uint16)((uint32)rsy * (uint32)LCD_W / (uint32)UVC_WIDTH);
            const uint16 y_lcd = (uint16)((uint32)rsx * (uint32)LCD_H / (uint32)UVC_HEIGHT);
            lcd.draw_point(x_lcd, y_lcd, RGB565_YELLOW);
        }
    }

    // 用“底部区域”(算法 y 方向末端，对应原图 x=160 这一侧)的 center_line 做平均误差
    uint32 x_sum = 0;
    uint32 row_cnt = 0;
    for(int row = row_begin; row <= row_end; row++)
    {
        x_sum += g_lane_result.center_line[row];
        ++row_cnt;
    }

    const uint32 x_avg = (row_cnt == 0) ? (uint32)IMAGE_CENTER_X : (x_sum / row_cnt);
    err_x_out = ((int32)x_avg - (int32)IMAGE_CENTER_X) * (int32)ERR_X_SIGN;
}

