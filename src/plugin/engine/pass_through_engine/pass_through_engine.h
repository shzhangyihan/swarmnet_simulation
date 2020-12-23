#ifndef __SIM_PASS_THROUGH_ENGINE_H__
#define __SIM_PASS_THROUGH_ENGINE_H__

#include "../../../core/arena.h"

namespace swarmnet_sim {

#define TICK_PER_SECOND 1000

extern "C" {
Physics_engine* engine_builder(void* arena);
}

class Pass_through_engine : public Physics_engine {
   public:
    double check_collision(double future_time);
    bool check_robot_collision(position2d_t pos_1, position2d_t pos_2,
                               int radius);
    bool check_out_of_bound(position2d_t pos, int radius, int x_max, int y_max);
    void init();
    Pass_through_engine(void* arena);
};

}  // namespace swarmnet_sim

#endif
