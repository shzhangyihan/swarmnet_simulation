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
    double check_collision(double future_time);
    double time_to_out_of_bound(position2d_t pos, double v, double radius,
                                double x_max, double y_max);
    double time_to_collide(position2d_t pos_1, double v_1, position2d_t pos_2,
                           double v_2, double radius);
    void init();
    Analytical_engine(void* arena);

   private:
    std::vector<bool> need_update;
    // pair of velocity and theta
    std::vector<std::pair<double, double>> motion_state;
    std::vector<long long> out_of_bound_time;
    std::vector<std::vector<long long>> collision_time;
};
}  // namespace swarmnet_sim

#endif
