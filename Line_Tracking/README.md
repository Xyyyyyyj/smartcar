## 仓库结构

```
八邻域巡线/
├── CMakeLists.txt          # 工程与 OpenCV 依赖
├── main.cpp                # 主循环：读帧、预处理、两种巡线、可视化
├── include/                # 头文件
│   ├── config.h            # 图像尺寸、阈值、十字补线、显示与调试开关
│   ├── lane_types.h        # GrayImage / BinaryImage / LaneResult 等数据结构
│   ├── preprocess.h
│   ├── lane_track_eight_neighborhood.h
│   ├── lane_track_scanline.h
│   ├── video_reader.h
│   └── visualizer.h
├── src/                    # 实现（核心算法无 OpenCV 依赖，便于移植）
│   ├── preprocess.cpp
│   ├── lane_track_eight_neighborhood.cpp
│   ├── lane_track_scanline.cpp
│   ├── video_reader.cpp    # OpenCV 读视频、灰度、缩放到固定分辨率
│   └── visualizer.cpp      # OpenCV 窗口与巡线结果绘制
└── video/                  # 默认示例视频路径见 config.h 中 DEFAULT_VIDEO_PATH
```

`lane_core` 静态库包含预处理与两种巡线；`pc_debug_io` 包含读视频与可视化，可拆到 PC 端调试，嵌入式侧只链 `lane_core` 与自有采集接口。

## 算法与数据流（简要）

1. **预处理**（`preprocess.cpp`）  
   灰度图二值化（可选 Otsu 或固定阈值）、简单形态滤波、去掉图像边框处的黑边，得到 `BinaryImage`。

2. **八邻域巡线**（`lane_track_eight_neighborhood.cpp`）  
   - 在底部附近一行，从图像中心向左、向右各找「白→黑」的边界点作为左右起点。  
   - 沿车道边界向上跟踪：在 8 邻域内按「外侧黑、内侧白」的准则选下一像素，分别维护左右轨迹与方向序列。  
   - 将轨迹投影为每行的 `left_border` / `right_border`，缺失行用上一次的值插补。  
   - **十字补线**：根据方向序列模式检测疑似十字断开处；在条件满足时用断开行上方一段边界做**最小二乘直线拟合**，再向下延伸填补缺口。  
   - 每行中线：`center_line[row] = (left_border[row] + right_border[row]) / 2`。

3. **逐行扫描巡线**（`lane_track_scanline.cpp`）  
   从底部行起自下而上：以「上一行有效时左右边界的中心」为参考，在当前行左右扫描寻找白黑边界；若仅一侧找到，则用上一行的**车道宽度**推算另一侧。结构更简单，适合资源紧张的嵌入式快速部署（无十字补线逻辑）。

当前 **主程序每一帧会同时运行上述两种算法**，可视化里分别叠加显示（窗口名含 `eight` 与 `scanline`），便于对比效果。

## 构建

```bash
cmake -S . -B build
cmake --build build
```

需本机已安装 OpenCV，并在 CMake 中能正确找到 `OpenCVConfig.cmake`（`CMakeLists.txt` 中含常见 Windows 路径提示，可按环境修改）。

## 运行

默认读取 `video/1.mp4`（可在 `include/config.h` 中改 `DEFAULT_VIDEO_PATH`）。按键说明：

- `q` 或 `Esc`：退出  
- `空格`：暂停或继续  
- `n`：暂停时单步前进一帧  

显示相关开关（原图 / 二值 / 缩放倍数等）见 `include/config.h`。
