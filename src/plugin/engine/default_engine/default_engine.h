#ifndef __SIM_DEFAULT_ENGINE_H__
#define __SIM_DEFAULT_ENGINE_H__

#include "../../../core/arena.h"

namespace swarmnet_sim {

#define TICK_PER_SECOND 1000

extern "C" {
float check_collision(Arena* arena, float future_time);
}
bool check_robot_collision(position2d_t pos_1, position2d_t pos_2, int radius);
bool check_out_of_bound(position2d_t pos, int radius, int x_max, int y_max);
}  // namespace swarmnet_sim

#endif
