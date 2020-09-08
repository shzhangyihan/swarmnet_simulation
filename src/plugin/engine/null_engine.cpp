#include "null_engine.h"

namespace swarmnet_sim {

extern "C" {
float check_collision(Arena *arena, float future_time) { return -1; }
}

}  // namespace swarmnet_sim
