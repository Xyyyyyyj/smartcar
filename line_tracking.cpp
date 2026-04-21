#include "line_tracking.hpp"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdarg>

static GrayImage g_gray_frame = {};
static BinaryImage g_binary_frame = {};
static LaneResult g_lane_result = {};
static uint8 g_shortflag = 0;
static uint8 g_left_shortflag = 0;
static uint8 g_right_shortflag = 0;
static_assert((IMAGE_WIDTH * IMAGE_HEIGHT) == (UVC_WIDTH * UVC_HEIGHT),
              "Line tracking expects same pixel count after rotation.");
static constexpr int32 CORNER_MAX_ROW = 80;
static constexpr int32 CORNER_JUMP_THRESH = 6;
static constexpr int32 MIDLINE_CROSS_CONSEC_ROWS = 3;

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

struct line_track_features
{
    uint8 trueshortflag = 0;
    uint8 left_trueshortflag = 0;
    uint8 right_trueshortflag = 0;
    uint8 left_duan = 0;
    uint8 right_duan = 0;

    uint8 rightdown_flag = 0;
    int16 rightdown_x = 0;
    int16 rightdown_y = 0;
    uint8 leftdown_flag = 0;
    int16 leftdown_x = 0;
    int16 leftdown_y = 0;
    uint8 leftup_flag = 0;
    int16 leftup_x = 0;
    int16 leftup_y = 0;
    uint8 rightup_flag = 0;
    int16 rightup_x = 0;
    int16 rightup_y = 0;

    uint8 lcenter_flag = 0;
    int16 lcenter_x = 0;
    int16 lcenter_y = 0;
    uint8 rcenter_flag = 0;
    int16 rcenter_x = 0;
    int16 rcenter_y = 0;

    uint8 corner_frame_valid = 0;
    uint8 left_corner_flag = 0;
    int16 left_corner_x = 0;
    int16 left_corner_y = 0;
    uint8 right_corner_flag = 0;
    int16 right_corner_x = 0;
    int16 right_corner_y = 0;
};

static void draw_circle_filled(zf_device_ips200 &lcd, int cx, int cy, int r, uint16 color)
{
    if(r <= 0) return;
    for(int dy = -r; dy <= r; ++dy)
    {
        const int y = cy + dy;
        if(y < 0 || y >= (int)LCD_H) continue;
        const int dx_lim = (int)std::sqrt((double)(r * r - dy * dy));
        int x0 = cx - dx_lim;
        int x1 = cx + dx_lim;
        if(x0 < 0) x0 = 0;
        if(x1 >= (int)LCD_W) x1 = (int)LCD_W - 1;
        for(int x = x0; x <= x1; ++x)
        {
            lcd.draw_point((uint16)x, (uint16)y, color);
        }
    }
}

static bool fit_line_least_squares(const int *y_arr, const int *x_arr, int n, float &k_out, float &b_out)
{
    if(n < 2) return false;
    double sy = 0.0, sx = 0.0, syy = 0.0, syx = 0.0;
    for(int i = 0; i < n; ++i)
    {
        const double y = (double)y_arr[i];
        const double x = (double)x_arr[i];
        sy += y;
        sx += x;
        syy += y * y;
        syx += y * x;
    }
    const double denom = (double)n * syy - sy * sy;
    if(denom == 0.0) return false;
    const double k = ((double)n * syx - sy * sx) / denom;
    const double b = (sx - k * sy) / (double)n;
    k_out = (float)k;
    b_out = (float)b;
    return true;
}

static constexpr int LEFT_RB_WIDTH_JUMP_THR =5;   // 末段宽度突变阈值（stage4->entered）
static constexpr int LEFT_RB_EDGE_JUMP_THR = 5;   // stage0->stage1 左边界突变阈值
static constexpr int LEFT_RB_STABLE_THR = 5;      // “右边界稳定”阈值
static constexpr int LEFT_RB_LOST_DEBOUNCE_FRAMES = 3;
static constexpr int LEFT_RB_PRINT_EVERY_N_FRAMES_IN_MODE = 10;
static constexpr int LEFT_RB_LOOKAHEAD_ROWS = 80; // 形态检测前瞻窗口
// ===== 形态检测阈值（stage2/stage3 在这里调）=====
static constexpr int LEFT_RB_MIN_DOWN_RUN = 5;    // 连续“变窄”最短长度
static constexpr int LEFT_RB_MIN_UP_RUN = 5;      // 连续“变宽”最短长度
static constexpr int LEFT_RB_MIN_TOTAL_SWING = 6;// 宽度总摆幅阈值
static constexpr int LEFT_EDGE_STUCK_THR = BORDER_MIN + 2;  // <= 该阈值视为贴边无效
static constexpr float RIGHT_STRAIGHT_SLOPE_DIFF_THR = 0.50f; // 右边界“直线”判定放宽：两段斜率差阈值
static constexpr float RIGHT_STRAIGHT_SLOPE_ABS_THR = 0.60f;  // 右边界“直线”判定放宽：斜率绝对值阈值

static void left_rb_printf(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    std::vprintf(fmt, args);
    va_end(args);
    std::fflush(stdout);
}

static inline int clamp_i32(const int v, const int lo, const int hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static bool find_left_edge_on_row_fast(const BinaryImage &bin, const int row, const int reference_center, int &left_edge_out)
{
    const int center = clamp_i32(reference_center, BORDER_MIN + 1, BORDER_MAX - 1);
    for(int col = center; col > BORDER_MIN; --col)
    {
        if(bin.data[row][col] == PIXEL_WHITE && bin.data[row][col - 1] == PIXEL_BLACK)
        {
            left_edge_out = col;
            return true;
        }
    }
    return false;
}

struct LeftRoundaboutSM
{
    enum class State : uint8_t
    {
        Idle = 0,
        Stage1_LoseLeft = 2,
        Stage2_WidenThenNarrow = 3, // 新增：先宽后窄
        Stage3_NarrowThenWiden = 4, // 新增：先窄后宽
        Stage4_LoseLeftAgain = 5,   // 原 stage3
        Entered = 6,
    };

    bool f1_widen_left = false;
    bool f2_left_lost = false;
    bool f3_widen_then_narrow = false;
    bool f4_narrow_then_widen = false;
    bool f5_left_lost_again = false;
    bool f5_width_jump_end = false;

    State state = State::Idle;
    int widen_start_row = -1;
    bool in_left_roundabout_mode = false;

    int stage0_print_div = 0;
    int stage1_print_div = 0;
    int stage2_print_div = 0;
    int stage3_print_div = 0;
    int stage4_print_div = 0;

    int left_lost_frames = 0;
    int left_lost_again_frames = 0;

    bool have_prev = false;
    int prev_left = IMAGE_CENTER_X - 10;
    int prev_right = IMAGE_CENTER_X + 10;
    int prev_width = 20;

    bool have_prev_rows = false;
    int prev_left_row[IMAGE_HEIGHT] = {};
    int prev_right_row[IMAGE_HEIGHT] = {};

    // 进入左环岛后需要标记的 1/2/3 点（算法坐标：x=边界列，y=row）
    bool p1_found = false;
    int p1_x = -1;
    int p1_y = -1;
    bool p2_found = false;
    int p2_x = -1; // 2点：右边界
    int p2_y = -1;
    bool p3_found = false;
    int p3_x = -1; // 3点：2点行的左边界
    int p3_y = -1;

    static int abs_i(int v) { return v < 0 ? -v : v; }

    void reset()
    {
        f1_widen_left = false;
        f2_left_lost = false;
        f3_widen_then_narrow = false;
        f4_narrow_then_widen = false;
        f5_left_lost_again = false;
        f5_width_jump_end = false;
        state = State::Idle;
        widen_start_row = -1;
        in_left_roundabout_mode = false;
        stage0_print_div = 0;
        stage1_print_div = 0;
        stage2_print_div = 0;
        stage3_print_div = 0;
        stage4_print_div = 0;
        left_lost_frames = 0;
        left_lost_again_frames = 0;
        have_prev = false;
        prev_left = IMAGE_CENTER_X - 10;
        prev_right = IMAGE_CENTER_X + 10;
        prev_width = 20;
        have_prev_rows = false;
        for(int i = 0; i < IMAGE_HEIGHT; ++i)
        {
            prev_left_row[i] = IMAGE_CENTER_X - 10;
            prev_right_row[i] = IMAGE_CENTER_X + 10;
        }

        p1_found = false;
        p1_x = -1;
        p1_y = -1;
        p2_found = false;
        p2_x = -1;
        p2_y = -1;
        p3_found = false;
        p3_x = -1;
        p3_y = -1;
    }

    void detect_points_after_entered(const LaneResult &res, const bool right_border_is_straight)
    {
        if(!in_left_roundabout_mode) return;
        // 需要“随图像刷新”的点：每帧重新计算 1/2/3 点
        p1_found = false;
        p1_x = -1;
        p1_y = -1;
        p2_found = false;
        p2_x = -1;
        p2_y = -1;
        p3_found = false;
        p3_x = -1;
        p3_y = -1;

        // 判断区间：从下往上第5行到第100行
        // bottom_row = IMAGE_HEIGHT - 1
        // row_hi: bottom-5, row_lo: bottom-100
        const int row_hi = clamp_i32((IMAGE_HEIGHT - 1) - 5, 0, IMAGE_HEIGHT - 2);   // -2 保证 row+1 不越界
        const int row_lo = clamp_i32((IMAGE_HEIGHT - 1) - 100, 0, row_hi - 1);
        int max_d_left = -100000;
        int max_d_width = -100000;
        int max_abs_d_right = 0;
        bool all_points_found = false;
        int first_p1_hit_row = -1;
        int first_p1_hit_d_left = 0;
        int first_p1_hit_d_width = 0;
        int first_p1_hit_abs_d_right = 0;

        // 先做“区间级”右边界稳定性判定：
        // 只有整个扫描区间内相邻行右边界都不过阈值，才允许进入 1/2/3 点识别。
        bool right_border_stable_window = true;
        for(int row = row_hi - 1; row >= row_lo; --row)
        {
            const int prev_row_below = row + 1;
            const int r_prev = (int)res.right_border[prev_row_below];
            const int r_cur = (int)res.right_border[row];
            const int abs_d_right = abs_i(r_prev - r_cur);
            if(abs_d_right > max_abs_d_right) max_abs_d_right = abs_d_right;
            if(abs_d_right > LEFT_RB_STABLE_THR)
            {
                right_border_stable_window = false;
                break;
            }
        }

        const bool right_border_overall_stable = right_border_stable_window;
        if(!right_border_overall_stable)
        {
            left_rb_printf("[LEFT_RB][P1_SCAN] row=[%d..%d] right_straight=%d window_stable=%d all_points=%d max(l_prev-l_cur)=%d max(w_cur-w_prev)=%d max(|r_prev-r_cur|)=%d stable_thr=%d\r\n",
                           row_lo, row_hi, right_border_is_straight ? 1 : 0, right_border_stable_window ? 1 : 0,
                           all_points_found ? 1 : 0, max_d_left, max_d_width, max_abs_d_right, LEFT_RB_STABLE_THR);
            left_rb_printf("[LEFT_RB][P1_SCAN] p1_found=%d p1=(x=%d,y=%d) first_hit_row=%d dL=%d dW=%d dR=%d\r\n",
                           p1_found ? 1 : 0, p1_x, p1_y,
                           first_p1_hit_row, first_p1_hit_d_left, first_p1_hit_d_width, first_p1_hit_abs_d_right);
            return;
        }

        // 从下往上扫描（row 越大越靠近车体）。
        // 这里明确约定：“上一行”在当前行下方，即 prev_row_below = row + 1。
        // 为保证“当前行/上一行”都在判断区间内，row 从 row_hi-1 开始。
        for(int row = row_hi - 1; row >= row_lo; --row)
        {
            const int prev_row_below = row + 1;
            const int l_prev = (int)res.left_border[prev_row_below];
            const int r_prev = (int)res.right_border[prev_row_below];
            const int l_cur = (int)res.left_border[row];
            const int r_cur = (int)res.right_border[row];

            const int w_prev = r_prev - l_prev;
            const int w_cur = r_cur - l_cur;
            const int d_left = l_prev - l_cur;
            const int d_width = w_cur - w_prev;
            const int abs_d_right = abs_i(r_prev - r_cur);

            if(d_left > max_d_left) max_d_left = d_left;
            if(d_width > max_d_width) max_d_width = d_width;
            if(abs_d_right > max_abs_d_right) max_abs_d_right = abs_d_right;

            if(all_points_found) continue;

            // 1点：右边界不突变，上一次(上一行)左边界 - 这次(该行)左边界 > 10
            // 且该行赛道宽度 - 上一行赛道宽度 > 10
            if(!p1_found)
            {
                if(d_left > 10 && d_width > 10)
                {
                    if(first_p1_hit_row < 0)
                    {
                        first_p1_hit_row = row;
                        first_p1_hit_d_left = d_left;
                        first_p1_hit_d_width = d_width;
                        first_p1_hit_abs_d_right = abs_d_right;
                    }
                    p1_found = true;
                    p1_x = l_cur;
                    p1_y = row;
                }
            }

            // 2点：右边界为直线，这一行左边界 - 上一行左边界 > 2
            // 记录此时左右边界位置：右边界为 2 点；其左边界为 3 点
            if(!p2_found)
            {
                if((l_cur - l_prev) > 2)
                {
                    p2_found = true;
                    p2_x = r_cur;
                    p2_y = row;
                    p3_found = true;
                    p3_x = l_cur;
                    p3_y = row;
                }
            }

            if(p1_found && p2_found && p3_found)
            {
                all_points_found = true;
            }
        }

        left_rb_printf("[LEFT_RB][P1_SCAN] row=[%d..%d] right_straight=%d window_stable=%d all_points=%d max(l_prev-l_cur)=%d max(w_cur-w_prev)=%d max(|r_prev-r_cur|)=%d stable_thr=%d\r\n",
                       row_lo, row_hi, right_border_is_straight ? 1 : 0, right_border_stable_window ? 1 : 0, all_points_found ? 1 : 0,
                       max_d_left, max_d_width, max_abs_d_right, LEFT_RB_STABLE_THR);
        left_rb_printf("[LEFT_RB][P1_SCAN] p1_found=%d p1=(x=%d,y=%d) first_hit_row=%d dL=%d dW=%d dR=%d\r\n",
                       p1_found ? 1 : 0, p1_x, p1_y,
                       first_p1_hit_row, first_p1_hit_d_left, first_p1_hit_d_width, first_p1_hit_abs_d_right);
    }

    static void print_left_edge_total(const BinaryImage &bin, const LaneResult &res)
    {
        const int start_row = clamp_i32(START_SEARCH_ROW, 8, IMAGE_HEIGHT - 1);
        const int end_row = clamp_i32(start_row - LEFT_RB_LOOKAHEAD_ROWS, 0, start_row - 1);

        int found_cnt = 0;
        int lost_cnt = 0;
        int stuck_cnt = 0;
        int edge_min = 9999;
        int edge_max = -1;
        int64_t edge_sum = 0;

        for(int row = start_row; row >= end_row; --row)
        {
            int edge = -1;
            const bool ok = find_left_edge_on_row_fast(bin, row, (int)res.center_line[row], edge);
            if(ok)
            {
                ++found_cnt;
                edge_sum += edge;
                if(edge < edge_min) edge_min = edge;
                if(edge > edge_max) edge_max = edge;
                if(edge <= LEFT_EDGE_STUCK_THR)
                {
                    ++stuck_cnt;
                    ++lost_cnt;
                }
            }
            else
            {
                ++lost_cnt;
            }
        }

        const int avg = (found_cnt > 0) ? (int)(edge_sum / found_cnt) : -1;
        left_rb_printf("[LEFT_RB] left_edge_total rows=%d found=%d lost=%d stuck=%d edge[min=%d max=%d avg=%d] stuck_thr<=%d\r\n",
                       (start_row - end_row + 1), found_cnt, lost_cnt, stuck_cnt,
                       (edge_max >= 0 ? edge_min : -1), edge_max, avg, LEFT_EDGE_STUCK_THR);
    }

    void update(const BinaryImage &bin, const LaneResult &res)
    {
        const int start_row = clamp_i32(START_SEARCH_ROW, 8, IMAGE_HEIGHT - 1);
        const int win_row_hi = clamp_i32(start_row - 5, 0, start_row);
        const int win_row_lo = clamp_i32(start_row - 85, 0, win_row_hi);

        int64_t sum_l = 0;
        int64_t sum_r = 0;
        int64_t sum_c = 0;
        int32_t cnt = 0;
        int32_t left_lost_cnt = 0;

        for(int row = win_row_hi; row >= win_row_lo; --row)
        {
            const int l = (int)res.left_border[row];
            const int r = (int)res.right_border[row];
            const int c = (int)res.center_line[row];
            sum_l += l;
            sum_r += r;
            sum_c += c;
            ++cnt;

            int edge_l = -1;
            const bool ok_l = find_left_edge_on_row_fast(bin, row, c, edge_l);
            if((!ok_l) || (edge_l <= LEFT_EDGE_STUCK_THR)) ++left_lost_cnt;
        }

        const int left_now = (cnt > 0) ? (int)(sum_l / cnt) : (int)res.left_border[start_row];
        const int right_now = (cnt > 0) ? (int)(sum_r / cnt) : (int)res.right_border[start_row];
        const int width_now = right_now - left_now;
        const int ref_center = (cnt > 0) ? (int)(sum_c / cnt) : (int)res.center_line[start_row];
        (void)ref_center;
        const bool left_lost = (cnt > 0) ? (left_lost_cnt * 10 >= cnt * 2) : false;

        bool cross_like = false;
        bool right_stable = false;
        int d_left_dbg = 0;
        int d_right_dbg = 0;
        int d_width_dbg = 0;

        if(have_prev)
        {
            d_left_dbg = left_now - prev_left;
            d_right_dbg = right_now - prev_right;
            d_width_dbg = width_now - prev_width;

            right_stable = (abs_i(d_right_dbg) <= LEFT_RB_STABLE_THR);
            const bool left_jump = (abs_i(d_left_dbg) >= LEFT_RB_EDGE_JUMP_THR);
            const bool right_jump = (abs_i(d_right_dbg) >= LEFT_RB_EDGE_JUMP_THR);
            cross_like = left_jump && right_jump;
        }

        bool left_jump_any_row = false;
        int left_jump_row = -1;
        int left_jump_dL = 0;
        int left_jump_dR = 0;
        if(have_prev_rows)
        {
            for(int row = win_row_hi; row >= win_row_lo; --row)
            {
                const int l = (int)res.left_border[row];
                const int r = (int)res.right_border[row];
                const int dL = l - prev_left_row[row];
                const int dR = r - prev_right_row[row];
                if(abs_i(dL) >= LEFT_RB_EDGE_JUMP_THR && abs_i(dR) <= LEFT_RB_STABLE_THR)
                {
                    left_jump_any_row = true;
                    left_jump_row = row;
                    left_jump_dL = dL;
                    left_jump_dR = dR;
                    break;
                }
            }
        }

        if(state == State::Stage1_LoseLeft)
        {
            if(left_lost && right_stable) left_lost_frames++;
            else left_lost_frames = 0;
        }
        else
        {
            left_lost_frames = 0;
        }

        if(state == State::Stage3_NarrowThenWiden)
        {
            if(left_lost && right_stable) left_lost_again_frames++;
            else left_lost_again_frames = 0;
        }
        else
        {
            left_lost_again_frames = 0;
        }

        auto detect_narrow_then_widen = [&](int &widen_row_out, int &w_min_out, int &w_max_out) -> bool
        {
            const int sr = clamp_i32(START_SEARCH_ROW, 8, IMAGE_HEIGHT - 1);
            const int er = clamp_i32(sr - LEFT_RB_LOOKAHEAD_ROWS, 0, sr - 1);

            int down_run = 0;
            int up_run = 0;
            int best_turn_row = -1;
            int w_max = -1;
            int w_min = 9999;

            int prev_w = (int)res.right_border[sr] - (int)res.left_border[sr];
            for(int row = sr - 1; row >= er; --row)
            {
                const int w = (int)res.right_border[row] - (int)res.left_border[row];
                if(w > w_max) w_max = w;
                if(w < w_min) w_min = w;

                const int dw = w - prev_w;
                if(dw < 0)
                {
                    down_run++;
                    up_run = 0;
                }
                else if(dw > 0)
                {
                    if(down_run >= LEFT_RB_MIN_DOWN_RUN && best_turn_row < 0) best_turn_row = row;
                    up_run++;
                }
                prev_w = w;
            }

            if(best_turn_row >= 0 && up_run >= LEFT_RB_MIN_UP_RUN && (w_max - w_min) >= LEFT_RB_MIN_TOTAL_SWING)
            {
                widen_row_out = best_turn_row;
                w_min_out = w_min;
                w_max_out = w_max;
                return true;
            }
            return false;
        };

        auto detect_widen_then_narrow = [&](int &narrow_row_out, int &w_min_out, int &w_max_out) -> bool
        {
            const int sr = clamp_i32(START_SEARCH_ROW, 8, IMAGE_HEIGHT - 1);
            const int er = clamp_i32(sr - LEFT_RB_LOOKAHEAD_ROWS, 0, sr - 1);

            int up_run = 0;
            int down_run = 0;
            int best_turn_row = -1;
            int w_max = -1;
            int w_min = 9999;

            int prev_w = (int)res.right_border[sr] - (int)res.left_border[sr];
            for(int row = sr - 1; row >= er; --row)
            {
                const int w = (int)res.right_border[row] - (int)res.left_border[row];
                if(w > w_max) w_max = w;
                if(w < w_min) w_min = w;

                const int dw = w - prev_w;
                if(dw > 0)
                {
                    up_run++;
                    down_run = 0;
                }
                else if(dw < 0)
                {
                    if(up_run >= LEFT_RB_MIN_UP_RUN && best_turn_row < 0) best_turn_row = row;
                    down_run++;
                }
                prev_w = w;
            }

            if(best_turn_row >= 0 && down_run >= LEFT_RB_MIN_DOWN_RUN && (w_max - w_min) >= LEFT_RB_MIN_TOTAL_SWING)
            {
                narrow_row_out = best_turn_row;
                w_min_out = w_min;
                w_max_out = w_max;
                return true;
            }
            return false;
        };

        switch(state)
        {
        case State::Idle:
            if(++stage0_print_div >= LEFT_RB_PRINT_EVERY_N_FRAMES_IN_MODE)
            {
                stage0_print_div = 0;
                if(have_prev)
                {
                    left_rb_printf("[LEFT_RB] stage0 widen dW=%d W:%d->%d dL=%d dR=%d win=[%d..%d] start_row=%d\r\n",
                                   d_width_dbg, prev_width, width_now, d_left_dbg, d_right_dbg,
                                   win_row_lo, win_row_hi, start_row);
                }
                else
                {
                    left_rb_printf("[LEFT_RB] stage0 widen (no prev) W=%d L=%d R=%d win=[%d..%d] start_row=%d\r\n",
                                   width_now, left_now, right_now, win_row_lo, win_row_hi, start_row);
                }
            }
            if(left_jump_any_row)
            {
                f1_widen_left = true;
                state = State::Stage1_LoseLeft;
                stage1_print_div = 0;
                left_rb_printf("[LEFT_RB] stage0->stage1 left_jump row=%d dL=%d dR=%d win=[%d..%d]\r\n",
                               left_jump_row, left_jump_dL, left_jump_dR, win_row_lo, win_row_hi);
            }
            break;

        case State::Stage1_LoseLeft:
            if(++stage1_print_div >= LEFT_RB_PRINT_EVERY_N_FRAMES_IN_MODE)
            {
                stage1_print_div = 0;
                left_rb_printf("[LEFT_RB] stage1 left_edge_stats\r\n");
                print_left_edge_total(bin, res);
            }
            // 你确认的逻辑：stage1 -> stage2 看左丢线去抖
            if(left_lost_frames >= LEFT_RB_LOST_DEBOUNCE_FRAMES)
            {
                f2_left_lost = true;
                state = State::Stage2_WidenThenNarrow;
                stage2_print_div = 0;
                left_rb_printf("[LEFT_RB] stage1->stage2 left_lost_debounce=%d\r\n", left_lost_frames);
            }
            break;

        case State::Stage2_WidenThenNarrow:
            if(++stage2_print_div >= LEFT_RB_PRINT_EVERY_N_FRAMES_IN_MODE)
            {
                stage2_print_div = 0;
                int turn_row = -1, w_min = 0, w_max = 0;
                if(detect_widen_then_narrow(turn_row, w_min, w_max))
                {
                    left_rb_printf("[LEFT_RB] stage2 widen->narrow OK=1 w_min=%d w_max=%d swing=%d turn_row=%d thr(up=%d down=%d swing=%d)\r\n",
                                   w_min, w_max, (w_max - w_min), turn_row,
                                   LEFT_RB_MIN_UP_RUN, LEFT_RB_MIN_DOWN_RUN, LEFT_RB_MIN_TOTAL_SWING);
                }
                else
                {
                    left_rb_printf("[LEFT_RB] stage2 widen->narrow OK=0 thr(up=%d down=%d swing=%d)\r\n",
                                   LEFT_RB_MIN_UP_RUN, LEFT_RB_MIN_DOWN_RUN, LEFT_RB_MIN_TOTAL_SWING);
                }
            }
            {
                int turn_row = -1, w_min = 0, w_max = 0;
                if(detect_widen_then_narrow(turn_row, w_min, w_max))
                {
                    f3_widen_then_narrow = true;
                    // 按你的要求：进入 stage3 即判定进入左环岛
                    state = State::Entered;
                    in_left_roundabout_mode = true;
                    left_rb_printf("[LEFT_RB] stage2->stage3 widen_narrow_flag=1 turn_row=%d\r\n", turn_row);
                    left_rb_printf("[LEFT_RB] ENTER_LEFT_ROUNDABOUT(by_stage3) f1=%d f2=%d f3_widen_narrow=%d\r\n",
                                   (int)f1_widen_left, (int)f2_left_lost, (int)f3_widen_then_narrow);
                }
            }
            break;

        case State::Stage3_NarrowThenWiden:
            if(++stage3_print_div >= LEFT_RB_PRINT_EVERY_N_FRAMES_IN_MODE)
            {
                stage3_print_div = 0;
                int turn_row = -1, w_min = 0, w_max = 0;
                if(detect_narrow_then_widen(turn_row, w_min, w_max))
                {
                    widen_start_row = turn_row;
                    left_rb_printf("[LEFT_RB] stage3 narrow->widen OK=1 w_min=%d w_max=%d swing=%d widen_row=%d thr(down=%d up=%d swing=%d)\r\n",
                                   w_min, w_max, (w_max - w_min), widen_start_row,
                                   LEFT_RB_MIN_DOWN_RUN, LEFT_RB_MIN_UP_RUN, LEFT_RB_MIN_TOTAL_SWING);
                }
                else
                {
                    left_rb_printf("[LEFT_RB] stage3 narrow->widen OK=0 thr(down=%d up=%d swing=%d)\r\n",
                                   LEFT_RB_MIN_DOWN_RUN, LEFT_RB_MIN_UP_RUN, LEFT_RB_MIN_TOTAL_SWING);
                }
            }
            {
                int turn_row = -1, w_min = 0, w_max = 0;
                if(detect_narrow_then_widen(turn_row, w_min, w_max))
                {
                    f4_narrow_then_widen = true;
                    widen_start_row = turn_row;
                    state = State::Stage4_LoseLeftAgain;
                    stage4_print_div = 0;
                    left_rb_printf("[LEFT_RB] stage3->stage4 narrow_widen_flag=1 widen_row=%d\r\n", widen_start_row);
                }
            }
            break;

        case State::Stage4_LoseLeftAgain:
            if(++stage4_print_div >= LEFT_RB_PRINT_EVERY_N_FRAMES_IN_MODE)
            {
                stage4_print_div = 0;
                left_rb_printf("[LEFT_RB] stage4 left_lost_rows_again (waiting enter)\r\n");
                print_left_edge_total(bin, res);
            }
            if(have_prev && abs_i(width_now - prev_width) >= (LEFT_RB_WIDTH_JUMP_THR + 8))
            {
                f5_width_jump_end = true;
                state = State::Entered;
                in_left_roundabout_mode = true;
                if(left_lost_again_frames >= LEFT_RB_LOST_DEBOUNCE_FRAMES) f5_left_lost_again = true;
                left_rb_printf("[LEFT_RB] ENTER_LEFT_ROUNDABOUT f1=%d f2=%d f3_widen_narrow=%d f4_narrow_widen=%d f5_lost_again=%d f_end=%d\r\n",
                               (int)f1_widen_left, (int)f2_left_lost, (int)f3_widen_then_narrow,
                               (int)f4_narrow_then_widen, (int)f5_left_lost_again, (int)f5_width_jump_end);
            }
            break;

        case State::Entered:
            break;
        }

        have_prev = true;
        prev_left = left_now;
        prev_right = right_now;
        prev_width = width_now;
        for(int row = win_row_hi; row >= win_row_lo; --row)
        {
            prev_left_row[row] = (int)res.left_border[row];
            prev_right_row[row] = (int)res.right_border[row];
        }
        have_prev_rows = true;

        if(state != State::Idle && cross_like)
        {
            reset();
        }
    }
};

static LeftRoundaboutSM g_left_rb_sm;
// 临时开关：1=关闭左环岛相关逻辑，0=启用
#define TEMP_DISABLE_LEFT_ROUNDABOUT 0

} // namespace

static bool detect_single_corner(const uint8_t *border, bool is_left, int16 &corner_x, int16 &corner_y)
{
    // 只对 80 行以下（更靠近车体的区域）判角点，从下往上扫描
    for(int row = CORNER_MAX_ROW; row >= 2; --row)
    {
        const int curr = (int)border[row];
        const int prev = (int)border[row - 1];
        const int d = prev - curr;
        if(is_left)
        {
            // 左边界：x 变大表示向中线靠近
            if(d >= CORNER_JUMP_THRESH)
            {
                corner_x = (int16)prev;
                corner_y = (int16)(row - 1);
                return true;
            }
        }
        else
        {
            // 右边界：x 变小表示向中线靠近
            if(d <= -CORNER_JUMP_THRESH)
            {
                corner_x = (int16)prev;
                corner_y = (int16)(row - 1);
                return true;
            }
        }
    }
    return false;
}

static float abs_diff_f(float a, float b)
{
    return (a >= b) ? (a - b) : (b - a);
}

static bool regression_slope(const uint8_t *arr, int s1, int e1, int s2, int e2, float &k_out)
{
    if(!(s1 >= 0 && e1 <= IMAGE_HEIGHT && s2 >= 0 && e2 <= IMAGE_HEIGHT && s1 < e1 && s2 < e2))
    {
        k_out = 0.0f;
        return false;
    }
    int n = 0;
    float sx = 0.0f;
    float sy = 0.0f;
    float sxx = 0.0f;
    float sxy = 0.0f;
    for(int i = s1; i < e1; ++i)
    {
        sx += (float)i;
        sy += (float)arr[i];
        sxx += (float)i * (float)i;
        sxy += (float)i * (float)arr[i];
        ++n;
    }
    for(int i = s2; i < e2; ++i)
    {
        sx += (float)i;
        sy += (float)arr[i];
        sxx += (float)i * (float)i;
        sxy += (float)i * (float)arr[i];
        ++n;
    }
    if(n < 2)
    {
        k_out = 0.0f;
        return false;
    }
    const float denom = (float)n * sxx - sx * sx;
    if(denom == 0.0f)
    {
        k_out = 0.0f;
        return false;
    }
    k_out = ((float)n * sxy - sx * sy) / denom;
    return true;
}

static void extract_line_features(const LaneResult &res, line_track_features &f)
{
    float k1 = 0.0f;
    float k2 = 0.0f;
    regression_slope(res.center_line, 10, 25, 45, 55, k1);
    regression_slope(res.center_line, 60, 64, 68, 72, k2);
    const bool short_wrong = (abs_diff_f(0.0f, k1) >= 0.35f) || (abs_diff_f(0.0f, k2) >= 0.35f);
    if(!short_wrong && abs_diff_f(k1, k2) <= 0.1f)
    {
        g_shortflag = (uint8)clamp_int32((int32)g_shortflag + 1, 0, 2);
    }
    else
    {
        g_shortflag = 0;
    }
    f.trueshortflag = (g_shortflag >= 2) ? 1 : 0;

    regression_slope(res.left_border, 15, 30, 35, 45, k1);
    regression_slope(res.left_border, 60, 70, 90, 100, k2);
    if(abs_diff_f(k1, k2) <= 0.1f)
    {
        g_left_shortflag = (uint8)clamp_int32((int32)g_left_shortflag + 1, 0, 2);
    }
    else
    {
        g_left_shortflag = 0;
    }
    f.left_trueshortflag = (g_left_shortflag >= 2) ? 1 : 0;

    regression_slope(res.right_border, 15, 30, 35, 45, k1);
    regression_slope(res.right_border, 60, 70, 90, 100, k2);
    // 右边界“直线”判定：放宽斜率差阈值；同时允许两段斜率都较小的情况
    if(abs_diff_f(k1, k2) <= RIGHT_STRAIGHT_SLOPE_DIFF_THR ||
       (abs_diff_f(0.0f, k1) <= RIGHT_STRAIGHT_SLOPE_ABS_THR && abs_diff_f(0.0f, k2) <= RIGHT_STRAIGHT_SLOPE_ABS_THR))
    {
        g_right_shortflag = (uint8)clamp_int32((int32)g_right_shortflag + 1, 0, 2);
    }
    else
    {
        g_right_shortflag = 0;
    }
    f.right_trueshortflag = (g_right_shortflag >= 2) ? 1 : 0;

    int left_blank = 0;
    int right_blank = 0;
    for(int i = 35; i < 70; ++i)
    {
        if(res.left_border[i] <= 5) ++left_blank;
        if(res.right_border[i] >= 114) ++right_blank;
    }
    f.left_duan = (left_blank >= 10) ? 1 : 0;
    f.right_duan = (right_blank >= 10) ? 1 : 0;

    for(int i = 118; i > 40; --i)
    {
        if(std::abs((int)res.right_border[i] - (int)res.right_border[i + 1]) <= 5 &&
           std::abs((int)res.right_border[i + 1] - (int)res.right_border[i + 2]) <= 5 &&
           std::abs((int)res.right_border[i + 2] - (int)res.right_border[i + 3]) <= 5 &&
           ((int)res.right_border[i] - (int)res.right_border[i - 2]) <= -5 &&
           ((int)res.right_border[i] - (int)res.right_border[i - 3]) <= -5)
        {
            f.rightdown_x = (int16)res.right_border[i];
            f.rightdown_y = (int16)i;
            f.rightdown_flag = 1;
        }

        if(std::abs((int)res.left_border[i] - (int)res.left_border[i + 1]) <= 5 &&
           std::abs((int)res.left_border[i + 1] - (int)res.left_border[i + 2]) <= 5 &&
           std::abs((int)res.left_border[i + 2] - (int)res.left_border[i + 3]) <= 5 &&
           ((int)res.left_border[i] - (int)res.left_border[i - 2]) >= 5 &&
           ((int)res.left_border[i] - (int)res.left_border[i - 3]) >= 10)
        {
            f.leftdown_x = (int16)res.left_border[i];
            f.leftdown_y = (int16)i;
            f.leftdown_flag = 1;
        }
    }

    for(int i = 70; i > 15; --i)
    {
        if(std::abs((int)res.left_border[i] - (int)res.left_border[i - 1]) <= 3 &&
           std::abs((int)res.left_border[i - 1] - (int)res.left_border[i - 2]) <= 3 &&
           std::abs((int)res.left_border[i - 2] - (int)res.left_border[i - 3]) <= 3 &&
           ((int)res.left_border[i] - (int)res.left_border[i + 1]) >= 8 &&
           ((int)res.left_border[i] - (int)res.left_border[i + 2]) >= 10 &&
           ((int)res.left_border[i] - (int)res.left_border[i + 3]) >= 15)
        {
            f.leftup_x = (int16)res.left_border[i];
            f.leftup_y = (int16)i;
            f.leftup_flag = 1;
        }

        if(std::abs((int)res.right_border[i] - (int)res.right_border[i - 1]) <= 3 &&
           std::abs((int)res.right_border[i - 1] - (int)res.right_border[i - 2]) <= 3 &&
           std::abs((int)res.right_border[i - 2] - (int)res.right_border[i - 3]) <= 3 &&
           ((int)res.right_border[i] - (int)res.right_border[i + 1]) <= -8 &&
           ((int)res.right_border[i] - (int)res.right_border[i + 2]) <= -10 &&
           ((int)res.right_border[i] - (int)res.right_border[i + 3]) <= -15)
        {
            f.rightup_x = (int16)res.right_border[i];
            f.rightup_y = (int16)i;
            f.rightup_flag = 1;
        }
    }

    for(int i = 80; i > 5; --i)
    {
        const uint8_t v = res.left_border[i];
        bool same = true;
        bool maxv = true;
        for(int d = 1; d <= 5; ++d)
        {
            same = same && (v == res.left_border[i + d]) && (v == res.left_border[i - d]);
            maxv = maxv && (v >= res.left_border[i + d]) && (v >= res.left_border[i - d]);
        }
        if(!same && maxv && v >= 5)
        {
            f.lcenter_x = (int16)v;
            f.lcenter_y = (int16)i;
            f.lcenter_flag = 1;
            break;
        }
    }

    for(int i = 22; i < 80; ++i)
    {
        const uint8_t v = res.right_border[i];
        bool same = true;
        bool minv = true;
        for(int d = 1; d <= 5; ++d)
        {
            same = same && (v == res.right_border[i + d]) && (v == res.right_border[i - d]);
            minv = minv && (v <= res.right_border[i + d]) && (v <= res.right_border[i - d]);
        }
        if(!same && minv && v >= 80)
        {
            f.rcenter_x = (int16)v;
            f.rcenter_y = (int16)i;
            f.rcenter_flag = 1;
            break;
        }
    }

    // 角点检测：若在判定区“连续多行”跨越中线，则该帧跳过角点判定（抑制单点噪声）
    bool cross_midline = false;
    int32 cross_consec = 0;
    for(int row = 0; row <= CORNER_MAX_ROW; ++row)
    {
        const bool crossed = ((int)res.left_border[row] >= (int)IMAGE_CENTER_X) ||
                             ((int)res.right_border[row] <= (int)IMAGE_CENTER_X);
        if(crossed)
        {
            ++cross_consec;
            if(cross_consec >= MIDLINE_CROSS_CONSEC_ROWS)
            {
                cross_midline = true;
                break;
            }
        }
        else
        {
            cross_consec = 0;
        }
    }
    f.corner_frame_valid = cross_midline ? 0 : 1;
    if(!cross_midline)
    {
        f.left_corner_flag = detect_single_corner(res.left_border, true, f.left_corner_x, f.left_corner_y) ? 1 : 0;
        f.right_corner_flag = detect_single_corner(res.right_border, false, f.right_corner_x, f.right_corner_y) ? 1 : 0;
    }
}

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
    line_track_features features{};
    extract_line_features(g_lane_result, features);

    // ================= 左环岛点位/补线/拟合（需在叠加绘制前完成） =================
    // 这些变量在后续绘制与误差计算也会使用，必须放在公共作用域
    int supp_right[IMAGE_HEIGHT];
    for(int i = 0; i < IMAGE_HEIGHT; ++i) supp_right[i] = -1;
    uint8 center_fit[IMAGE_HEIGHT];
    for(int i = 0; i < IMAGE_HEIGHT; ++i) center_fit[i] = g_lane_result.center_line[i];
    bool have_supp_and_fit = false;
    int fit_y0 = -1;
    int fit_y1 = -1;
#if TEMP_DISABLE_LEFT_ROUNDABOUT
    // 临时屏蔽环岛：确保状态清零，后续流程走原始中心线
    g_left_rb_sm.reset();
#else
    if(g_lane_result.start_found)
    {
        g_left_rb_sm.update(g_binary_frame, g_lane_result);
    }
    else
    {
        g_left_rb_sm.reset();
    }

    // 进入左环岛后：识别并标记 1/2/3 点（当前为“随图像刷新”的点）
    g_left_rb_sm.detect_points_after_entered(g_lane_result, features.right_trueshortflag != 0);

    // 补线与拟合中线（随图像刷新）：黑线为补线右边界，绿线为左边界

    // 左环岛补线：
    // - 常规：p1+p2 都存在，用两点连线补右边界
    // - 需求变更：进入左环岛后若只识别到 p1、识别不到 p2/p3，仍按“1点补线”处理，
    //   但补线终点改为“从 p1 往下，右边界仍有赛道的最后一行”的右边界点（其余补线/拟合逻辑不变）
    int rb_p2_x = -1;
    int rb_p2_y = -1;
    bool rb_have_p2 = false;
    if(g_left_rb_sm.in_left_roundabout_mode && g_left_rb_sm.p1_found)
    {
        if(g_left_rb_sm.p2_found)
        {
            rb_p2_x = g_left_rb_sm.p2_x;
            rb_p2_y = g_left_rb_sm.p2_y;
            rb_have_p2 = true;
        }
        else
        {
            // 仅有 p1：找“右界有赛道的最后一行”（row 越大越靠近车体）
            // 右边界无效时通常会被置为 BORDER_MAX（见 scanline），因此这里用区间判定有效性。
            int last_row = -1;
            for(int row = IMAGE_HEIGHT - 1; row >= g_left_rb_sm.p1_y; --row)
            {
                const int r = (int)g_lane_result.right_border[row];
                if(r > BORDER_MIN && r < BORDER_MAX)
                {
                    last_row = row;
                    rb_p2_x = r;
                    rb_p2_y = row;
                    rb_have_p2 = true;
                    break; // 从底部向上扫，第一次命中即为“最后一行”
                }
            }
            (void)last_row;
        }
    }

    if(g_left_rb_sm.in_left_roundabout_mode && g_left_rb_sm.p1_found && rb_have_p2)
    {
        const int dy = (rb_p2_y - g_left_rb_sm.p1_y);
        float k = 0.0f;
        float b = 0.0f;
        if(dy != 0)
        {
            k = (float)(rb_p2_x - g_left_rb_sm.p1_x) / (float)dy;
            b = (float)g_left_rb_sm.p1_x - k * (float)g_left_rb_sm.p1_y;
            fit_y0 = std::min(g_left_rb_sm.p1_y, rb_p2_y);
            fit_y1 = std::max(g_left_rb_sm.p1_y, rb_p2_y);
        }
        else
        {
            // p1/p2 落在同一行时无法用两点求 x(row) 的斜率，这里用“常值右边界”兜底以便可视化补线/拟合
            k = 0.0f;
            b = (float)rb_p2_x;
            fit_y0 = clamp_i32(g_left_rb_sm.p1_y, 0, IMAGE_HEIGHT - 1);
            fit_y1 = clamp_i32(fit_y0 + 40, fit_y0, IMAGE_HEIGHT - 1);
        }

        for(int row = fit_y0; row <= fit_y1; ++row)
        {
            int xr = (int)std::lround(k * (float)row + b);
            xr = clamp_i32(xr, BORDER_MIN, BORDER_MAX);
            supp_right[row] = xr;
        }

        // 以“左边界(绿)”和“补线(黑)”构造中心点序列，再做最小二乘拟合
        // 注意：补线有时会落在左边界左侧（例如点2异常/左右交换），这里用 min/max 兜底，避免 n=0
        int ys[IMAGE_HEIGHT];
        int xs[IMAGE_HEIGHT];
        int n = 0;
        for(int row = fit_y0; row <= fit_y1; ++row)
        {
            const int xl_raw = (int)g_lane_result.left_border[row];
            const int xr_raw = supp_right[row];
            if(xr_raw < 0) continue;
            if(xl_raw <= BORDER_MIN || xl_raw >= BORDER_MAX) continue;
            if(xr_raw <= BORDER_MIN || xr_raw >= BORDER_MAX) continue;

            const int xL = std::min(xl_raw, xr_raw);
            const int xR = std::max(xl_raw, xr_raw);
            if(xR - xL < 3) continue;

            ys[n] = row;
            xs[n] = (xL + xR) / 2;
            ++n;
        }

        float k_fit = 0.0f, b_fit = 0.0f;
        if(fit_line_least_squares(ys, xs, n, k_fit, b_fit))
        {
            for(int row = fit_y0; row <= fit_y1; ++row)
            {
                int xc = (int)std::lround(k_fit * (float)row + b_fit);
                xc = clamp_i32(xc, BORDER_MIN, BORDER_MAX);
                center_fit[row] = (uint8)xc;
            }
            have_supp_and_fit = true;
        }
        left_rb_printf("[LEFT_RB][FIT] p1=(%d,%d) p2=(%d,%d) dy=%d win=[%d..%d] n=%d ok=%d\r\n",
                       g_left_rb_sm.p1_x, g_left_rb_sm.p1_y,
                       rb_p2_x, rb_p2_y,
                       dy, fit_y0, fit_y1, n, have_supp_and_fit ? 1 : 0);
    }
#endif

    // ============== 在屏幕上叠加：赛道边界 + 中心拟合线 ==============
    // 算法坐标：120x160（宽x高）；屏幕显示的是旋转180°后的 160x120 图像。
    // 映射关系（算法点 -> 原图点）：(x_alg, y_alg) -> (x_src = y_alg, y_src = x_alg)
    // err_x 统计窗口固定为 100±3 行
    const int row_begin = (int)clamp_int32(100 - 3, 0, IMAGE_HEIGHT - 1);
    const int row_end = (int)clamp_int32(100 + 3, row_begin, IMAGE_HEIGHT - 1);
    // 左环岛判定窗口边界（从下往上第5~100行）：在屏幕上标蓝线，便于调试
    const int win_row_hi = (int)clamp_int32((IMAGE_HEIGHT - 1) - 5, 0, IMAGE_HEIGHT - 1);
    const int win_row_lo = (int)clamp_int32((IMAGE_HEIGHT - 1) - 100, 0, win_row_hi);
    // row_begin 是算法的 y（对应原图 x），在屏幕上表现为一条竖线
    const uint16 x_lcd_row_begin = (uint16)((uint32)row_begin * (uint32)LCD_W / (uint32)UVC_WIDTH);
    const uint16 x_lcd_win_hi = (uint16)((uint32)win_row_hi * (uint32)LCD_W / (uint32)UVC_WIDTH);
    const uint16 x_lcd_win_lo = (uint16)((uint32)win_row_lo * (uint32)LCD_W / (uint32)UVC_WIDTH);
    for(uint16 y = 0; y < LCD_H; ++y)
    {
        lcd.draw_point(x_lcd_row_begin, y, RGB565_YELLOW);
        lcd.draw_point(x_lcd_win_hi, y, RGB565_BLUE);
        lcd.draw_point(x_lcd_win_lo, y, RGB565_BLUE);
    }

    bool have_prev_supp = false;
    uint16 prev_supp_x = 0;
    uint16 prev_supp_y = 0;
    for(int row = 0; row < IMAGE_HEIGHT; ++row) // row: y_alg
    {
        // 原图坐标：x_src=row, y_src=border(row)
        const uint16 x_lcd = (uint16)((uint32)row * (uint32)LCD_W / (uint32)UVC_WIDTH);
        const uint16 y_lcd_l = (uint16)((uint32)g_lane_result.left_border[row] * (uint32)LCD_H / (uint32)UVC_HEIGHT);
        const uint16 y_lcd_r = (uint16)((uint32)g_lane_result.right_border[row] * (uint32)LCD_H / (uint32)UVC_HEIGHT);
        const bool in_fit_win = have_supp_and_fit && (row >= fit_y0) && (row <= fit_y1);
        const uint8 center_to_draw = in_fit_win ? center_fit[row] : g_lane_result.center_line[row];
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
            // 清除区间内原本的中线：区间内只画“新拟合中线”，区间外画原中线
            if(yc >= 0 && yc < LCD_H) lcd.draw_point(x_lcd, (uint16)yc, in_fit_win ? RGB565_PURPLE : RGB565_CYAN);
        }

        // 补线右边界：黑色（仅在拟合窗口内有效）
        if(in_fit_win && supp_right[row] >= 0)
        {
            const uint16 y_lcd_supp_r = (uint16)((uint32)supp_right[row] * (uint32)LCD_H / (uint32)UVC_HEIGHT);
            // 用相邻点连线，显示为连续补线
            if(have_prev_supp)
            {
                lcd.draw_line(prev_supp_x, prev_supp_y, x_lcd, y_lcd_supp_r, RGB565_BLACK);
            }
            prev_supp_x = x_lcd;
            prev_supp_y = y_lcd_supp_r;
            have_prev_supp = true;
        }
        else
        {
            // 断开补线连线状态，避免跨空洞/跨窗口连接
            have_prev_supp = false;
        }
    }

    if(features.left_corner_flag)
    {
        const uint16 x_lcd = (uint16)((uint32)features.left_corner_y * (uint32)LCD_W / (uint32)UVC_WIDTH);
        const uint16 y_lcd = (uint16)((uint32)features.left_corner_x * (uint32)LCD_H / (uint32)UVC_HEIGHT);
        lcd.draw_point(x_lcd, y_lcd, RGB565_WHITE);
    }
    if(features.right_corner_flag)
    {
        const uint16 x_lcd = (uint16)((uint32)features.right_corner_y * (uint32)LCD_W / (uint32)UVC_WIDTH);
        const uint16 y_lcd = (uint16)((uint32)features.right_corner_x * (uint32)LCD_H / (uint32)UVC_HEIGHT);
        lcd.draw_point(x_lcd, y_lcd, RGB565_WHITE);
    }
    // 注意：点识别/补线/拟合已提前到叠加绘制前，这里只负责画点位标记
#if !TEMP_DISABLE_LEFT_ROUNDABOUT
    if(g_left_rb_sm.p1_found)
    {
        const uint16 x_lcd = (uint16)((uint32)g_left_rb_sm.p1_y * (uint32)LCD_W / (uint32)UVC_WIDTH);
        const uint16 y_lcd = (uint16)((uint32)g_left_rb_sm.p1_x * (uint32)LCD_H / (uint32)UVC_HEIGHT);
        left_rb_printf("[LEFT_RB][P1_DRAW] p1=(x=%d,y=%d) lcd=(x=%u,y=%u)\r\n",
                       g_left_rb_sm.p1_x, g_left_rb_sm.p1_y, (unsigned)x_lcd, (unsigned)y_lcd);
        draw_circle_filled(lcd, (int)x_lcd, (int)y_lcd, 6, RGB565_YELLOW);
    }
    if(g_left_rb_sm.p2_found)
    {
        const uint16 x_lcd = (uint16)((uint32)g_left_rb_sm.p2_y * (uint32)LCD_W / (uint32)UVC_WIDTH);
        const uint16 y_lcd = (uint16)((uint32)g_left_rb_sm.p2_x * (uint32)LCD_H / (uint32)UVC_HEIGHT);
        draw_circle_filled(lcd, (int)x_lcd, (int)y_lcd, 6, RGB565_YELLOW);
    }
    if(g_left_rb_sm.p3_found)
    {
        const uint16 x_lcd = (uint16)((uint32)g_left_rb_sm.p3_y * (uint32)LCD_W / (uint32)UVC_WIDTH);
        const uint16 y_lcd = (uint16)((uint32)g_left_rb_sm.p3_x * (uint32)LCD_H / (uint32)UVC_HEIGHT);
        draw_circle_filled(lcd, (int)x_lcd, (int)y_lcd, 6, RGB565_YELLOW);
    }
#endif


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
        x_sum += have_supp_and_fit ? center_fit[row] : g_lane_result.center_line[row];
        ++row_cnt;
    }

    const uint32 x_avg = (row_cnt == 0) ? (uint32)IMAGE_CENTER_X : (x_sum / row_cnt);
    err_x_out = ((int32)x_avg - (int32)IMAGE_CENTER_X) * (int32)ERR_X_SIGN;
}

