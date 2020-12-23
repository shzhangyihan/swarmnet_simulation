#include "null_engine.h"

namespace swarmnet_sim {

double Null_engine::check_collision(double future_time) { return -1; }

void Null_engine::init() {}

Null_engine::Null_engine(void* arena) { this->arena = arena; }

extern "C" {
Physics_engine* engine_builder(void* arena) {
    Physics_engine* engine = new Null_engine(arena);
    return engine;
}
}
}  // namespace swarmnet_sim
