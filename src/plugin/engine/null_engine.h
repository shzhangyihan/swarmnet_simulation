#ifndef __SIM_NULL_ENGINE_H__
#define __SIM_NULL_ENGINE_H__

#include "../../core/arena.h"

namespace swarmnet_sim {

#define TICK_PER_SECOND 1000

extern "C" {
float check_collision(Arena* arena, float future_time);
}
}  // namespace swarmnet_sim

#endif
