#ifndef CONFIG_H_
#define CONFIG_H_

#include <cstdint>

// 注意：LS2K0300 上 UVC 原始图像是 160x120（宽x高）。
// 为了让巡线“从 160 这一侧作为底部向上搜索”，算法内部将图像坐标系旋转 90°，
// 使用 120x160（宽x高）的图像进行处理：算法的 y 方向长度为 160。
inline constexpr int IMAGE_WIDTH = 120;
inline constexpr int IMAGE_HEIGHT = 160;

inline constexpr std::uint8_t PIXEL_BLACK = 0;
inline constexpr std::uint8_t PIXEL_WHITE = 255;

inline constexpr int BORDER_MIN = 1;
inline constexpr int BORDER_MAX = IMAGE_WIDTH - 2;
inline constexpr int IMAGE_CENTER_X = IMAGE_WIDTH / 2;

inline constexpr int TRACK_MAX_POINTS = IMAGE_HEIGHT * 3;

inline constexpr bool PREPROCESS_USE_OTSU = false;
inline constexpr std::uint8_t PREPROCESS_FIXED_THRESHOLD = 120;
// 预处理强度参数
inline constexpr int PREPROCESS_MEDIAN_PASSES = 0;          // 中值滤波轮数（3x3）
inline constexpr int PREPROCESS_OPENING_PASSES = 3;         // 开运算轮数
inline constexpr int PREPROCESS_CLOSING_PASSES = 1;         // 闭运算轮数（连通断裂白线、填小黑洞）
inline constexpr int PREPROCESS_CC_MIN_AREA = 100;           // 连通域去噪：删除面积小于该值的白色连通域
// 八邻域起点：从下往上数第 10 行（0-based 行号 IMAGE_HEIGHT - 10）
inline constexpr int START_SEARCH_ROW = IMAGE_HEIGHT - 5;

inline constexpr const char* DEFAULT_VIDEO_PATH = "video/1.mp4";
inline constexpr bool SHOW_SOURCE_WINDOW = true;
inline constexpr bool SHOW_BINARY_WINDOW = true;
inline constexpr bool SHOW_OVERLAY_WINDOW = true;
inline constexpr bool PAUSE_ON_START = false;
inline constexpr int PLAYBACK_DELAY_MS = 2;
inline constexpr int DISPLAY_SCALE = 4;

#endif  // CONFIG_H_
