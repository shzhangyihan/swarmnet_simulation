#ifndef __SIM_DEFAULT_ENGINE_H__
#define __SIM_DEFAULT_ENGINE_H__

#include "../../../core/arena.h"
#include "../../../core/physics_2d.h"

namespace swarmnet_sim {

#define TICK_PER_SECOND 1000

extern "C" {
Physics_engine* engine_builder(void* arena);
}

class Default_engine : public Physics_engine {
   public:
    float check_collision(float future_time);
    bool check_robot_collision(position2d_t pos_1, position2d_t pos_2,
                               int radius);
    bool check_out_of_bound(position2d_t pos, int radius, int x_max, int y_max);
    void init();
    Default_engine(void* arena);
};

}  // namespace swarmnet_sim

#endif
