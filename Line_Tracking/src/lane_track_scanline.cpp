#include "lane_track_scanline.h"

#include <cstdint>
#include <cstring>

#include "config.h"

namespace {

int clamp_int(const int value, const int low, const int high) {
    if (value < low) {
        return low;
    }
    if (value > high) {
        return high;
    }
    return value;
}

std::uint8_t clamp_u8(const int value, const int low, const int high) {
    return static_cast<std::uint8_t>(clamp_int(value, low, high));
}

void reset_result(LaneResult* result) {
    std::memset(result, 0, sizeof(LaneResult));

    for (int row = 0; row < IMAGE_HEIGHT; ++row) {
        result->left_border[row] = BORDER_MIN;
        result->right_border[row] = BORDER_MAX;
        result->center_line[row] = IMAGE_CENTER_X;
    }

    result->left_start = {-1, -1};
    result->right_start = {-1, -1};
    result->highest_row = static_cast<std::uint8_t>(START_SEARCH_ROW);
}

void compute_center_line(LaneResult* result) {
    for (int row = 0; row < IMAGE_HEIGHT; ++row) {
        if (result->left_border[row] >= result->right_border[row]) {
            result->left_border[row] = BORDER_MIN;
            result->right_border[row] = BORDER_MAX;
        }
        result->center_line[row] = static_cast<std::uint8_t>(
            (result->left_border[row] + result->right_border[row]) >> 1);
    }
}

bool find_left_edge_on_row(const BinaryImage* bin, const int row, const int reference_center,
                           int* left_edge) {
    const int center = clamp_int(reference_center, BORDER_MIN + 1, BORDER_MAX - 1);
    for (int col = center; col > BORDER_MIN; --col) {
        if (bin->data[row][col] == PIXEL_WHITE && bin->data[row][col - 1] == PIXEL_BLACK) {
            *left_edge = col;
            return true;
        }
    }
    return false;
}

bool find_right_edge_on_row(const BinaryImage* bin, const int row, const int reference_center,
                            int* right_edge) {
    const int center = clamp_int(reference_center, BORDER_MIN + 1, BORDER_MAX - 1);
    for (int col = center; col < BORDER_MAX; ++col) {
        if (bin->data[row][col] == PIXEL_WHITE && bin->data[row][col + 1] == PIXEL_BLACK) {
            *right_edge = col;
            return true;
        }
    }
    return false;
}

void record_scanline_row(LaneResult* result, const int row, const int left_border,
                         const int right_border) {
    result->left_border[row] = clamp_u8(left_border, BORDER_MIN, BORDER_MAX);
    result->right_border[row] = clamp_u8(right_border, BORDER_MIN, BORDER_MAX);

    if (result->left_points_count < TRACK_MAX_POINTS) {
        result->left_points[result->left_points_count++] = {
            static_cast<std::int16_t>(result->left_border[row]), static_cast<std::int16_t>(row)};
    }
    if (result->right_points_count < TRACK_MAX_POINTS) {
        result->right_points[result->right_points_count++] = {
            static_cast<std::int16_t>(result->right_border[row]), static_cast<std::int16_t>(row)};
    }
}

}  // namespace

void lane_track_scanline_run(const BinaryImage* bin, LaneResult* result) {
    if (bin == nullptr || result == nullptr) {
        return;
    }

    reset_result(result);

    const int start_row = clamp_int(START_SEARCH_ROW, 1, IMAGE_HEIGHT - 1);
    int last_left = BORDER_MIN;
    int last_right = BORDER_MAX;
    int last_width = BORDER_MAX - BORDER_MIN;
    bool have_valid_row = false;
    int highest_row = start_row;

    for (int row = start_row; row >= 0; --row) {
        const int reference_center = have_valid_row ? ((last_left + last_right) >> 1) : IMAGE_CENTER_X;
        int left_edge = -1;
        int right_edge = -1;

        const bool left_found = find_left_edge_on_row(bin, row, reference_center, &left_edge);
        const bool right_found = find_right_edge_on_row(bin, row, reference_center, &right_edge);

        if (left_found && right_found) {
            if (row == start_row) {
                result->start_found = true;
                result->left_start = {static_cast<std::int16_t>(left_edge),
                                      static_cast<std::int16_t>(start_row)};
                result->right_start = {static_cast<std::int16_t>(right_edge),
                                       static_cast<std::int16_t>(start_row)};
            }
        } else if (row == start_row) {
            compute_center_line(result);
            return;
        } else if (have_valid_row && left_found) {
            right_edge = clamp_int(left_edge + last_width, left_edge + 1, BORDER_MAX);
        } else if (have_valid_row && right_found) {
            left_edge = clamp_int(right_edge - last_width, BORDER_MIN, right_edge - 1);
        } else if (have_valid_row) {
            left_edge = last_left;
            right_edge = last_right;
        } else {
            continue;
        }

        if (left_edge >= right_edge) {
            left_edge = last_left;
            right_edge = last_right;
        }

        record_scanline_row(result, row, left_edge, right_edge);
        last_left = result->left_border[row];
        last_right = result->right_border[row];
        last_width = last_right - last_left;
        have_valid_row = true;
        highest_row = row;
    }

    if (!result->start_found || !have_valid_row) {
        compute_center_line(result);
        return;
    }

    for (int row = start_row + 1; row < IMAGE_HEIGHT; ++row) {
        result->left_border[row] = result->left_border[start_row];
        result->right_border[row] = result->right_border[start_row];
    }

    result->highest_row = static_cast<std::uint8_t>(clamp_int(highest_row, 0, IMAGE_HEIGHT - 1));
    compute_center_line(result);
}
