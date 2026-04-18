#include "preprocess.h"

#include <cstdint>

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

std::uint8_t compute_otsu_threshold(const GrayImage* src) {
    std::uint32_t histogram[256] = {0};
    for (int row = 0; row < IMAGE_HEIGHT; ++row) {
        for (int col = 0; col < IMAGE_WIDTH; ++col) {
            ++histogram[src->data[row][col]];
        }
    }

    const std::uint32_t total_pixels = static_cast<std::uint32_t>(IMAGE_WIDTH * IMAGE_HEIGHT);
    std::uint64_t total_sum = 0;
    for (int level = 0; level < 256; ++level) {
        total_sum += static_cast<std::uint64_t>(level) * histogram[level];
    }

    std::uint32_t background_pixels = 0;
    std::uint64_t background_sum = 0;
    double best_variance = -1.0;
    std::uint8_t best_threshold = PREPROCESS_FIXED_THRESHOLD;

    for (int level = 0; level < 256; ++level) {
        background_pixels += histogram[level];
        if (background_pixels == 0) {
            continue;
        }

        const std::uint32_t foreground_pixels = total_pixels - background_pixels;
        if (foreground_pixels == 0) {
            break;
        }

        background_sum += static_cast<std::uint64_t>(level) * histogram[level];

        const double weight_background = static_cast<double>(background_pixels) / total_pixels;
        const double weight_foreground = static_cast<double>(foreground_pixels) / total_pixels;
        const double mean_background = static_cast<double>(background_sum) / background_pixels;
        const double mean_foreground =
            static_cast<double>(total_sum - background_sum) / foreground_pixels;
        const double delta = mean_background - mean_foreground;
        const double variance = weight_background * weight_foreground * delta * delta;

        if (variance > best_variance) {
            best_variance = variance;
            best_threshold = static_cast<std::uint8_t>(level);
        }
    }

    if (best_threshold == PIXEL_BLACK || best_threshold == PIXEL_WHITE) {
        return PREPROCESS_FIXED_THRESHOLD;
    }
    return best_threshold;
}

void apply_threshold(const GrayImage* src, BinaryImage* dst, const std::uint8_t threshold) {
    for (int row = 0; row < IMAGE_HEIGHT; ++row) {
        for (int col = 0; col < IMAGE_WIDTH; ++col) {
            dst->data[row][col] =
                (src->data[row][col] > threshold) ? PIXEL_WHITE : PIXEL_BLACK;
        }
    }
}

std::uint8_t median9(std::uint8_t values[9]) {
    for (int i = 0; i < 5; ++i) {
        int min_index = i;
        for (int j = i + 1; j < 9; ++j) {
            if (values[j] < values[min_index]) {
                min_index = j;
            }
        }
        if (min_index != i) {
            const std::uint8_t temp = values[i];
            values[i] = values[min_index];
            values[min_index] = temp;
        }
    }
    return values[4];
}

void median_filter3x3(const GrayImage* src, GrayImage* dst) {
    if (src == nullptr || dst == nullptr) {
        return;
    }
    *dst = *src;

    for (int row = 1; row < IMAGE_HEIGHT - 1; ++row) {
        for (int col = 1; col < IMAGE_WIDTH - 1; ++col) {
            std::uint8_t window[9];
            int index = 0;
            for (int dr = -1; dr <= 1; ++dr) {
                for (int dc = -1; dc <= 1; ++dc) {
                    window[index++] = src->data[row + dr][col + dc];
                }
            }
            dst->data[row][col] = median9(window);
        }
    }
}

void apply_median_filter(GrayImage* image, const int passes) {
    if (image == nullptr || passes <= 0) {
        return;
    }
    GrayImage temp = {};
    for (int i = 0; i < passes; ++i) {
        median_filter3x3(image, &temp);
        *image = temp;
    }
}

void erode3x3(const BinaryImage* src, BinaryImage* dst) {
    for (int row = 0; row < IMAGE_HEIGHT; ++row) {
        for (int col = 0; col < IMAGE_WIDTH; ++col) {
            dst->data[row][col] = PIXEL_BLACK;
        }
    }

    for (int row = 1; row < IMAGE_HEIGHT - 1; ++row) {
        for (int col = 1; col < IMAGE_WIDTH - 1; ++col) {
            bool all_white = true;
            for (int dr = -1; dr <= 1 && all_white; ++dr) {
                for (int dc = -1; dc <= 1; ++dc) {
                    if (src->data[row + dr][col + dc] != PIXEL_WHITE) {
                        all_white = false;
                        break;
                    }
                }
            }
            dst->data[row][col] = all_white ? PIXEL_WHITE : PIXEL_BLACK;
        }
    }
}

void dilate3x3(const BinaryImage* src, BinaryImage* dst) {
    for (int row = 0; row < IMAGE_HEIGHT; ++row) {
        for (int col = 0; col < IMAGE_WIDTH; ++col) {
            dst->data[row][col] = PIXEL_BLACK;
        }
    }

    for (int row = 1; row < IMAGE_HEIGHT - 1; ++row) {
        for (int col = 1; col < IMAGE_WIDTH - 1; ++col) {
            bool any_white = false;
            for (int dr = -1; dr <= 1 && !any_white; ++dr) {
                for (int dc = -1; dc <= 1; ++dc) {
                    if (src->data[row + dr][col + dc] == PIXEL_WHITE) {
                        any_white = true;
                        break;
                    }
                }
            }
            dst->data[row][col] = any_white ? PIXEL_WHITE : PIXEL_BLACK;
        }
    }
}

void apply_opening(BinaryImage* image, const int passes) {
    if (image == nullptr || passes <= 0) {
        return;
    }
    BinaryImage temp = {};
    BinaryImage opened = {};
    for (int i = 0; i < passes; ++i) {
        erode3x3(image, &temp);
        dilate3x3(&temp, &opened);
        *image = opened;
    }
}

void apply_closing(BinaryImage* image, const int passes) {
    if (image == nullptr || passes <= 0) {
        return;
    }
    BinaryImage temp = {};
    BinaryImage closed = {};
    for (int i = 0; i < passes; ++i) {
        dilate3x3(image, &temp);
        erode3x3(&temp, &closed);
        *image = closed;
    }
}

void clear_border(BinaryImage* image) {
    for (int row = 0; row < IMAGE_HEIGHT; ++row) {
        image->data[row][0] = PIXEL_BLACK;
        image->data[row][1] = PIXEL_BLACK;
        image->data[row][IMAGE_WIDTH - 2] = PIXEL_BLACK;
        image->data[row][IMAGE_WIDTH - 1] = PIXEL_BLACK;
    }

    for (int col = 0; col < IMAGE_WIDTH; ++col) {
        image->data[0][col] = PIXEL_BLACK;
        image->data[1][col] = PIXEL_BLACK;
    }
}

void remove_small_connected_components(BinaryImage* image, const int min_area) {
    if (image == nullptr || min_area <= 1) {
        return;
    }

    static std::uint8_t visited[IMAGE_HEIGHT][IMAGE_WIDTH];
    static std::uint16_t queue[IMAGE_HEIGHT * IMAGE_WIDTH];
    static std::uint16_t component_pixels[IMAGE_HEIGHT * IMAGE_WIDTH];

    for (int row = 0; row < IMAGE_HEIGHT; ++row) {
        for (int col = 0; col < IMAGE_WIDTH; ++col) {
            visited[row][col] = 0;
        }
    }

    for (int row = 0; row < IMAGE_HEIGHT; ++row) {
        for (int col = 0; col < IMAGE_WIDTH; ++col) {
            if (image->data[row][col] != PIXEL_WHITE || visited[row][col] != 0) {
                continue;
            }

            int q_head = 0;
            int q_tail = 0;
            int component_size = 0;
            const std::uint16_t start_index =
                static_cast<std::uint16_t>(row * IMAGE_WIDTH + col);
            queue[q_tail++] = start_index;
            visited[row][col] = 1;

            while (q_head < q_tail) {
                const std::uint16_t index = queue[q_head++];
                component_pixels[component_size++] = index;
                const int current_row = index / IMAGE_WIDTH;
                const int current_col = index % IMAGE_WIDTH;

                for (int dr = -1; dr <= 1; ++dr) {
                    for (int dc = -1; dc <= 1; ++dc) {
                        if (dr == 0 && dc == 0) {
                            continue;
                        }
                        const int next_row = current_row + dr;
                        const int next_col = current_col + dc;
                        if (next_row < 0 || next_row >= IMAGE_HEIGHT || next_col < 0 ||
                            next_col >= IMAGE_WIDTH) {
                            continue;
                        }
                        if (visited[next_row][next_col] != 0 ||
                            image->data[next_row][next_col] != PIXEL_WHITE) {
                            continue;
                        }
                        visited[next_row][next_col] = 1;
                        queue[q_tail++] = static_cast<std::uint16_t>(
                            next_row * IMAGE_WIDTH + next_col);
                    }
                }
            }

            if (component_size < min_area) {
                for (int i = 0; i < component_size; ++i) {
                    const std::uint16_t index = component_pixels[i];
                    const int pixel_row = index / IMAGE_WIDTH;
                    const int pixel_col = index % IMAGE_WIDTH;
                    image->data[pixel_row][pixel_col] = PIXEL_BLACK;
                }
            }
        }
    }
}

}  // namespace

void preprocess_run(const GrayImage* src, BinaryImage* dst) {
    if (src == nullptr || dst == nullptr) {
        return;
    }

    GrayImage filtered = *src;
    apply_median_filter(&filtered, clamp_int(PREPROCESS_MEDIAN_PASSES, 0, 3));
    std::uint8_t threshold = PREPROCESS_FIXED_THRESHOLD;
    if (PREPROCESS_USE_OTSU) {
        threshold = compute_otsu_threshold(&filtered);
    }

    threshold = static_cast<std::uint8_t>(clamp_int(threshold, 1, 254));
    apply_threshold(&filtered, dst, threshold);
    apply_opening(dst, clamp_int(PREPROCESS_OPENING_PASSES, 0, 3));
    apply_closing(dst, clamp_int(PREPROCESS_CLOSING_PASSES, 0, 3));
    remove_small_connected_components(dst, clamp_int(PREPROCESS_CC_MIN_AREA, 0, IMAGE_WIDTH * IMAGE_HEIGHT));
    clear_border(dst);
}
