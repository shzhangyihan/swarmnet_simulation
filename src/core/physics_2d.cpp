#include "physics_2d.h"

#include <math.h>

#include <iostream>

namespace swarmnet_sim {

position2d_t calculate_future_pos(position2d_t start, float v, float seconds) {
    position2d_t end;
    // std::cout << start.theta << std::endl;
    float x_diff = cos(((double)start.theta) / 180 * M_PI) * (v * seconds);
    float y_diff = sin(((double)start.theta) / 180 * M_PI) * (v * seconds);
    end.x = start.x + x_diff;
    end.y = start.y + y_diff;
    end.theta = start.theta;
    return end;
}

float calculate_dist(position2d_t pos1, position2d_t pos2) {
    float dist;
    dist = sqrt((pos1.x - pos2.x) * (pos1.x - pos2.x) +
                (pos1.y - pos2.y) * (pos1.y - pos2.y));
    return dist;
}

bool operator==(const position2d_t& lhs, const position2d_t& rhs) {
    if (lhs.x == rhs.x && lhs.y == rhs.y && lhs.theta == rhs.theta)
        return true;
    else
        return false;
}

bool operator==(const color_t& lhs, const color_t& rhs) {
    if (lhs.red == rhs.red && lhs.green == rhs.green && lhs.blue == rhs.blue)
        return true;
    else
        return false;
}

}  // namespace swarmnet_sim
