#ifndef __SIM_NULL_ENGINE_H__
#define __SIM_NULL_ENGINE_H__

#include "../../../core/arena.h"

namespace swarmnet_sim {

#define TICK_PER_SECOND 1000

extern "C" {
Physics_engine* engine_builder(void* arena);
}

class Null_engine : public Physics_engine {
   public:
    float check_collision(float future_time);
    void init();
    Null_engine(void* arena);
};

}  // namespace swarmnet_sim

#endif
