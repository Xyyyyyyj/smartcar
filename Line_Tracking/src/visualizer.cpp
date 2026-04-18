#include "visualizer.h"

#include <cstdio>
#include <cstring>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "config.h"

namespace {

constexpr char kSourceWindow[] = "source";
constexpr char kBinaryWindow[] = "binary";
constexpr char kOverlayEightWindow[] = "lane_overlay_eight";
constexpr char kOverlayScanlineWindow[] = "lane_overlay_scanline";

cv::Mat gray_to_mat(const GrayImage* gray) {
    cv::Mat image(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
    for (int row = 0; row < IMAGE_HEIGHT; ++row) {
        std::memcpy(image.ptr(row), gray->data[row], IMAGE_WIDTH);
    }
    return image;
}

cv::Mat binary_to_mat(const BinaryImage* binary) {
    cv::Mat image(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
    for (int row = 0; row < IMAGE_HEIGHT; ++row) {
        std::memcpy(image.ptr(row), binary->data[row], IMAGE_WIDTH);
    }
    return image;
}

void set_overlay_pixel(cv::Mat* overlay, const int x, const int y, const cv::Vec3b& color) {
    if (x < 0 || x >= IMAGE_WIDTH || y < 0 || y >= IMAGE_HEIGHT) {
        return;
    }
    overlay->at<cv::Vec3b>(y, x) = color;
}

void show_scaled(const char* window_name, const cv::Mat& image) {
    if (DISPLAY_SCALE <= 1) {
        cv::imshow(window_name, image);
        return;
    }

    cv::Mat scaled;
    cv::resize(image, scaled, cv::Size(image.cols * DISPLAY_SCALE, image.rows * DISPLAY_SCALE), 0.0,
               0.0, cv::INTER_NEAREST);
    cv::imshow(window_name, scaled);
}

void draw_lane_on_overlay(cv::Mat* overlay, const LaneResult* result, const char* title_line) {
    if (overlay == nullptr || result == nullptr || title_line == nullptr) {
        return;
    }

    for (std::uint16_t index = 0; index < result->left_points_count; ++index) {
        set_overlay_pixel(overlay, result->left_points[index].x, result->left_points[index].y,
                          cv::Vec3b(255, 0, 0));
    }
    for (std::uint16_t index = 0; index < result->right_points_count; ++index) {
        set_overlay_pixel(overlay, result->right_points[index].x, result->right_points[index].y,
                          cv::Vec3b(0, 0, 255));
    }

    for (int row = 0; row < IMAGE_HEIGHT; ++row) {
        set_overlay_pixel(overlay, result->left_border[row], row, cv::Vec3b(0, 255, 0));
        set_overlay_pixel(overlay, result->right_border[row], row, cv::Vec3b(0, 255, 0));
        set_overlay_pixel(overlay, result->center_line[row], row, cv::Vec3b(0, 255, 255));
    }

    if (result->start_found) {
        cv::drawMarker(*overlay, cv::Point(result->left_start.x, result->left_start.y),
                       cv::Scalar(255, 255, 0), cv::MARKER_CROSS, 6, 1);
        cv::drawMarker(*overlay, cv::Point(result->right_start.x, result->right_start.y),
                       cv::Scalar(0, 255, 255), cv::MARKER_CROSS, 6, 1);
    }

    char status_text[200] = {0};
    std::snprintf(status_text, sizeof(status_text),
                  "%s start:%s L:%u R:%u top:%u", title_line,
                  result->start_found ? "yes" : "no", result->left_points_count,
                  result->right_points_count, result->highest_row);
    cv::putText(*overlay, status_text, cv::Point(4, 14), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(0, 255, 255), 1, cv::LINE_AA);
}

}  // namespace

void visualizer_draw(const GrayImage* gray, const BinaryImage* bin,
                     const LaneResult* eight_neighborhood, const LaneResult* scanline) {
    if (gray == nullptr || bin == nullptr || eight_neighborhood == nullptr ||
        scanline == nullptr) {
        return;
    }

    const cv::Mat gray_image = gray_to_mat(gray);
    const cv::Mat binary_image = binary_to_mat(bin);

    if (SHOW_SOURCE_WINDOW) {
        show_scaled(kSourceWindow, gray_image);
    }

    if (SHOW_BINARY_WINDOW) {
        show_scaled(kBinaryWindow, binary_image);
    }

    if (!SHOW_OVERLAY_WINDOW) {
        return;
    }

    cv::Mat overlay_eight;
    cv::cvtColor(gray_image, overlay_eight, cv::COLOR_GRAY2BGR);
    draw_lane_on_overlay(&overlay_eight, eight_neighborhood, "[8-neighborhood]");
    show_scaled(kOverlayEightWindow, overlay_eight);

    cv::Mat overlay_scan;
    cv::cvtColor(gray_image, overlay_scan, cv::COLOR_GRAY2BGR);
    draw_lane_on_overlay(&overlay_scan, scanline, "[scanline]");
    show_scaled(kOverlayScanlineWindow, overlay_scan);
}

int visualizer_wait_key(const bool step_mode) {
    return cv::waitKey(step_mode ? 0 : PLAYBACK_DELAY_MS);
}

void visualizer_close() {
    cv::destroyAllWindows();
}
