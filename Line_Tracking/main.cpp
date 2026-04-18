#include <iostream>

#include "config.h"
#include "lane_track_eight_neighborhood.h"
#include "lane_track_scanline.h"
#include "preprocess.h"
#include "video_reader.h"
#include "visualizer.h"

int main() {
    if (!video_reader_open(DEFAULT_VIDEO_PATH)) {
        std::cerr << "Failed to open video: " << DEFAULT_VIDEO_PATH << '\n';
        return 1;
    }

    std::cout << "Lane demo controls: q/ESC quit, space pause/resume, n single-step when paused."
              << '\n';

    GrayImage gray_frame = {};
    BinaryImage binary_frame = {};
    LaneResult eight_result = {};
    LaneResult scanline_result = {};
    bool step_mode = PAUSE_ON_START;

    while (video_reader_read_frame(&gray_frame)) {
        preprocess_run(&gray_frame, &binary_frame);
        lane_track_eight_neighborhood_run(&binary_frame, &eight_result);
        lane_track_scanline_run(&binary_frame, &scanline_result);
        visualizer_draw(&gray_frame, &binary_frame, &eight_result, &scanline_result);

        const int key = visualizer_wait_key(step_mode);
        if (key == 27 || key == 'q' || key == 'Q') {
            break;
        }

        if (key == ' ') {
            step_mode = !step_mode;
            continue;
        }

        if (step_mode && key != 'n' && key != 'N') {
            continue;
        }
    }

    visualizer_close();
    return 0;
}
