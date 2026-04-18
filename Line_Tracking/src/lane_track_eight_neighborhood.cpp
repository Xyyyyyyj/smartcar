#include "lane_track_eight_neighborhood.h"

#include <cmath>
#include <cstdint>
#include <cstring>

#include "config.h"

namespace {

struct CandidatePoint {
    TrackPoint point;
    std::uint8_t direction;
};

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

int abs_int(const int value) {
    return (value >= 0) ? value : -value;
}

bool point_equal(const TrackPoint& left, const TrackPoint& right) {
    return left.x == right.x && left.y == right.y;
}

bool point_inside(const int x, const int y) {
    return x > 0 && x < IMAGE_WIDTH - 1 && y > 0 && y < IMAGE_HEIGHT - 1;
}

bool points_meet(const TrackPoint& left, const TrackPoint& right) {
    return abs_int(left.x - right.x) <= 1 && abs_int(left.y - right.y) <= 1;
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

bool find_start_points(const BinaryImage* bin, TrackPoint* left_start, TrackPoint* right_start) {
    const int search_row = clamp_int(START_SEARCH_ROW, 1, IMAGE_HEIGHT - 2);
    bool left_found = false;
    bool right_found = false;

    for (int col = IMAGE_CENTER_X; col > BORDER_MIN; --col) {
        if (bin->data[search_row][col] == PIXEL_WHITE &&
            bin->data[search_row][col - 1] == PIXEL_BLACK) {
            *left_start = {static_cast<std::int16_t>(col), static_cast<std::int16_t>(search_row)};
            left_found = true;
            break;
        }
    }

    for (int col = IMAGE_CENTER_X; col < BORDER_MAX; ++col) {
        if (bin->data[search_row][col] == PIXEL_WHITE &&
            bin->data[search_row][col + 1] == PIXEL_BLACK) {
            *right_start = {static_cast<std::int16_t>(col), static_cast<std::int16_t>(search_row)};
            right_found = true;
            break;
        }
    }

    return left_found && right_found;
}

bool search_next_point(const BinaryImage* bin, const TrackPoint& current,
                       const std::int8_t seeds[8][2], TrackPoint* next_point,
                       std::uint8_t* direction) {
    CandidatePoint candidates[8] = {};
    std::uint8_t candidate_count = 0;

    for (std::uint8_t index = 0; index < 8; ++index) {
        const int first_x = current.x + seeds[index][0];
        const int first_y = current.y + seeds[index][1];
        const int second_index = (index + 1) & 7;
        const int second_x = current.x + seeds[second_index][0];
        const int second_y = current.y + seeds[second_index][1];

        if (!point_inside(first_x, first_y) || !point_inside(second_x, second_y)) {
            continue;
        }

        if (bin->data[first_y][first_x] == PIXEL_BLACK &&
            bin->data[second_y][second_x] == PIXEL_WHITE) {
            candidates[candidate_count].point = {
                static_cast<std::int16_t>(first_x), static_cast<std::int16_t>(first_y)};
            candidates[candidate_count].direction = index;
            ++candidate_count;
        }
    }

    if (candidate_count == 0) {
        return false;
    }

    std::uint8_t best_index = 0;
    for (std::uint8_t index = 1; index < candidate_count; ++index) {
        const TrackPoint& candidate = candidates[index].point;
        const TrackPoint& best = candidates[best_index].point;
        if (candidate.y < best.y ||
            (candidate.y == best.y &&
             abs_int(candidate.x - current.x) < abs_int(best.x - current.x))) {
            best_index = index;
        }
    }

    *next_point = candidates[best_index].point;
    *direction = candidates[best_index].direction;
    return true;
}

void trace_boundaries(const BinaryImage* bin, LaneResult* result) {
    static constexpr std::int8_t kLeftSeeds[8][2] = {
        {0, 1}, {-1, 1}, {-1, 0}, {-1, -1}, {0, -1}, {1, -1}, {1, 0}, {1, 1}};
    static constexpr std::int8_t kRightSeeds[8][2] = {
        {0, 1}, {1, 1}, {1, 0}, {1, -1}, {0, -1}, {-1, -1}, {-1, 0}, {-1, 1}};

    TrackPoint left_current = result->left_start;
    TrackPoint right_current = result->right_start;
    int left_stagnation = 0;
    int right_stagnation = 0;

    for (int step = 0; step < TRACK_MAX_POINTS; ++step) {
        if (result->left_points_count < TRACK_MAX_POINTS) {
            result->left_points[result->left_points_count++] = left_current;
        }
        if (result->right_points_count < TRACK_MAX_POINTS) {
            result->right_points[result->right_points_count++] = right_current;
        }

        TrackPoint next_left = left_current;
        TrackPoint next_right = right_current;
        std::uint8_t left_dir = 0;
        std::uint8_t right_dir = 0;

        const bool left_ok = search_next_point(bin, left_current, kLeftSeeds, &next_left, &left_dir);
        const bool right_ok =
            search_next_point(bin, right_current, kRightSeeds, &next_right, &right_dir);

        if (!left_ok || !right_ok) {
            result->highest_row = static_cast<std::uint8_t>(
                clamp_int((left_current.y < right_current.y) ? left_current.y : right_current.y, 0,
                          IMAGE_HEIGHT - 1));
            break;
        }

        if (point_equal(next_left, left_current)) {
            ++left_stagnation;
        } else {
            left_stagnation = 0;
        }

        if (point_equal(next_right, right_current)) {
            ++right_stagnation;
        } else {
            right_stagnation = 0;
        }

        if (points_meet(next_left, next_right) || left_stagnation >= 2 || right_stagnation >= 2) {
            result->highest_row = static_cast<std::uint8_t>(
                clamp_int((next_left.y + next_right.y) / 2, 0, IMAGE_HEIGHT - 1));
            break;
        }

        left_current = next_left;
        right_current = next_right;
        result->highest_row = static_cast<std::uint8_t>(
            clamp_int((left_current.y < right_current.y) ? left_current.y : right_current.y, 0,
                      IMAGE_HEIGHT - 1));

        if (left_current.y <= 1 || right_current.y <= 1) {
            break;
        }
    }
}

void extract_borders(LaneResult* result) {
    bool left_seen[IMAGE_HEIGHT] = {false};
    bool right_seen[IMAGE_HEIGHT] = {false};

    for (std::uint16_t index = 0; index < result->left_points_count; ++index) {
        const TrackPoint& point = result->left_points[index];
        if (point.y < 0 || point.y >= IMAGE_HEIGHT) {
            continue;
        }

        const std::uint8_t border = clamp_u8(point.x + 1, BORDER_MIN, BORDER_MAX);
        if (!left_seen[point.y] || border > result->left_border[point.y]) {
            result->left_border[point.y] = border;
            left_seen[point.y] = true;
        }
    }

    for (std::uint16_t index = 0; index < result->right_points_count; ++index) {
        const TrackPoint& point = result->right_points[index];
        if (point.y < 0 || point.y >= IMAGE_HEIGHT) {
            continue;
        }

        const std::uint8_t border = clamp_u8(point.x - 1, BORDER_MIN, BORDER_MAX);
        if (!right_seen[point.y] || border < result->right_border[point.y]) {
            result->right_border[point.y] = border;
            right_seen[point.y] = true;
        }
    }

    std::uint8_t last_left = BORDER_MIN;
    std::uint8_t last_right = BORDER_MAX;
    for (int row = IMAGE_HEIGHT - 1; row >= 0; --row) {
        if (left_seen[row]) {
            last_left = result->left_border[row];
        } else {
            result->left_border[row] = last_left;
        }

        if (right_seen[row]) {
            last_right = result->right_border[row];
        } else {
            result->right_border[row] = last_right;
        }

        if (result->left_border[row] >= result->right_border[row]) {
            result->left_border[row] = BORDER_MIN;
            result->right_border[row] = BORDER_MAX;
        }
    }
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

}  // namespace

void lane_track_eight_neighborhood_run(const BinaryImage* bin, LaneResult* result) {
    if (bin == nullptr || result == nullptr) {
        return;
    }

    reset_result(result);

    result->start_found = find_start_points(bin, &result->left_start, &result->right_start);
    if (!result->start_found) {
        compute_center_line(result);
        return;
    }

    trace_boundaries(bin, result);
    extract_borders(result);
    compute_center_line(result);
}
