#include "collision_event.h"

#include <iostream>

#include "../../core/arena.h"

namespace swarmnet_sim {

void Collision_event::exec() { ((Arena *)arena)->get_node(to_id)->collision(); }

}  // namespace swarmnet_sim