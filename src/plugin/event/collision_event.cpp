#include "collision_event.h"

#include <iostream>

#include "../../core/arena.h"

namespace swarmnet_sim {

void Collision_event::exec() {
    Arena* arena_ptr = (Arena*)arena;
    Node* target_node = arena_ptr->get_node(to_id);
    target_node->collision_wrapper();
    std::cout << "at " << arena_ptr->get_current_tick() << " from " << from_id
              << " to " << to_id << std::endl
              << std::flush;
    target_node->stop();
    this->log_node(to_id);
}

}  // namespace swarmnet_sim