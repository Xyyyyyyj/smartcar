#ifndef VIDEO_READER_H_
#define VIDEO_READER_H_

#include "lane_types.h"

bool video_reader_open(const char* path);
bool video_reader_read_frame(GrayImage* frame);

#endif  // VIDEO_READER_H_
