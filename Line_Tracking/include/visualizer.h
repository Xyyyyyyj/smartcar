#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include "lane_types.h"

void visualizer_draw(const GrayImage* gray, const BinaryImage* bin,
                     const LaneResult* eight_neighborhood, const LaneResult* scanline);
int visualizer_wait_key(bool step_mode);
void visualizer_close();

#endif  // VISUALIZER_H_
