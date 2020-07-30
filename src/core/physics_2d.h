#ifndef __SIM_PHYSICS_H__
#define __SIM_PHYSICS_H__

namespace swarmnet_sim {

typedef struct position {
    float x;
    float y;
    float theta;
} position2d_t;

position2d_t calculate_future_pos(position2d_t start, float v, int ticks);

}  // namespace swarmnet_sim

#endif