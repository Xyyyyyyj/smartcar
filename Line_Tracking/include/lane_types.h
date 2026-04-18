#ifndef LANE_TYPES_H_
#define LANE_TYPES_H_

#include <cstdint>

#include "config.h"

struct GrayImage {
    std::uint8_t data[IMAGE_HEIGHT][IMAGE_WIDTH];
};

struct BinaryImage {
    std::uint8_t data[IMAGE_HEIGHT][IMAGE_WIDTH];
};

struct TrackPoint {
    std::int16_t x;
    std::int16_t y;
};

struct LaneResult {
    std::uint8_t left_border[IMAGE_HEIGHT];
    std::uint8_t right_border[IMAGE_HEIGHT];
    std::uint8_t center_line[IMAGE_HEIGHT];

    TrackPoint left_points[TRACK_MAX_POINTS];
    TrackPoint right_points[TRACK_MAX_POINTS];

    std::uint16_t left_points_count;
    std::uint16_t right_points_count;

    TrackPoint left_start;
    TrackPoint right_start;

    std::uint8_t highest_row;
    bool start_found;
};

#endif  // LANE_TYPES_H_
