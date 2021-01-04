#include "physics_2d.h"

#include <math.h>

#include <iostream>

namespace swarmnet_sim {

position2d_t calculate_future_pos(position2d_t start, double v,
                                  double seconds) {
    position2d_t end;
    // std::cout << start.theta << std::endl;
    // if (seconds < 0) throw "Moving back time!";
    double x_diff = cos(((double)start.theta) / 180 * M_PI) * (v * seconds);
    double y_diff = sin(((double)start.theta) / 180 * M_PI) * (v * seconds);
    end.x = start.x + x_diff;
    end.y = start.y + y_diff;
    end.theta = start.theta;
    return end;
}

double calculate_dist(position2d_t pos1, position2d_t pos2) {
    double dist;
    dist = sqrt((pos1.x - pos2.x) * (pos1.x - pos2.x) +
                (pos1.y - pos2.y) * (pos1.y - pos2.y));
    return dist;
}

bool if_collision(position2d_t pos_1, position2d_t pos_2, double radius) {
    double dist = calculate_dist(pos_1, pos_2);
    if (dist > radius * 2) {
        return false;
    } else {
        return true;
    }
}

bool if_out_of_bound(position2d_t pos, double radius, double x_max,
                     double y_max) {
    double x = pos.x;
    double y = pos.y;

    if (x - radius < 0 || x + radius > x_max) {
        return true;
    }
    if (y - radius < 0 || y + radius > y_max) {
        return true;
    }

    return false;
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
