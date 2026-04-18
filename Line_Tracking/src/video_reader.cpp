#include "video_reader.h"

#include <cstring>

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "config.h"

namespace {

cv::VideoCapture g_capture;

}  // namespace

bool video_reader_open(const char* path) {
    g_capture.release();
    return g_capture.open(path);
}

bool video_reader_read_frame(GrayImage* frame) {
    if (frame == nullptr || !g_capture.isOpened()) {
        return false;
    }

    cv::Mat input_frame;
    if (!g_capture.read(input_frame) || input_frame.empty()) {
        return false;
    }

    cv::Mat gray_frame;
    if (input_frame.channels() == 1) {
        gray_frame = input_frame;
    } else {
        cv::cvtColor(input_frame, gray_frame, cv::COLOR_BGR2GRAY);
    }

    cv::Mat resized_frame;
    cv::resize(gray_frame, resized_frame, cv::Size(IMAGE_WIDTH, IMAGE_HEIGHT), 0.0, 0.0,
               cv::INTER_AREA);

    for (int row = 0; row < IMAGE_HEIGHT; ++row) {
        std::memcpy(frame->data[row], resized_frame.ptr(row), IMAGE_WIDTH);
    }

    return true;
}
