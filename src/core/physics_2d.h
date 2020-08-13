#ifndef __SIM_PHYSICS_H__
#define __SIM_PHYSICS_H__

#include <stdint.h>

namespace swarmnet_sim {

typedef struct position {
    float x;
    float y;
    float theta;
} position2d_t;

typedef struct color {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} color_t;

position2d_t calculate_future_pos(position2d_t start, float v, float seconds);
float calculate_dist(position2d_t pos1, position2d_t pos2);

}  // namespace swarmnet_sim

#endif