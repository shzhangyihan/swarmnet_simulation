#include "physics_2d.h"

#include <math.h>

#include <iostream>

namespace swarmnet_sim {

position2d_t calculate_future_pos(position2d_t start, float v, int ticks) {
    position2d_t end;
    // std::cout << start.theta << std::endl;
    float x_diff = cos(((double)start.theta) / 180 * M_PI) * (v * ticks);
    float y_diff = sin(((double)start.theta) / 180 * M_PI) * (v * ticks);
    end.x = start.x + x_diff;
    end.y = start.y + y_diff;
    end.theta = start.theta;
    return end;
}

}  // namespace swarmnet_sim
