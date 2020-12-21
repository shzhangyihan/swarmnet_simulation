#ifndef __SIM_ANALYTICAL_ENGINE_H__
#define __SIM_ANALYTICAL_ENGINE_H__

#include "../../../core/arena.h"

namespace swarmnet_sim {

#define TICK_PER_SECOND 1000

extern "C" {
Physics_engine* engine_builder(void* arena);
}

class Analytical_engine : public Physics_engine {
   public:
    float check_collision(float future_time);
    bool check_robot_collision(position2d_t pos_1, position2d_t pos_2,
                               int radius);
    float time_to_out_of_bound(position2d_t pos, float v, int radius, int x_max,
                               int y_max);
    float time_to_collide(position2d_t pos_1, float v_1, position2d_t pos_2,
                          float v_2, int radius);
    void init();
    Analytical_engine(void* arena);
};
}  // namespace swarmnet_sim

#endif
