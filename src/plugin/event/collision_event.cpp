#include "collision_event.h"

#include <iostream>

namespace swarmnet_sim {

void Collision_event::exec() {
    // std::cout << "collision at " << exec_tick << " from " << from_id << " to
    // "
    //           << to_id << std::endl;
    ((Arena *)arena)->get_node(to_id)->collision();
}

}  // namespace swarmnet_sim