#include "loop_event.h"

#include <iostream>

#include "../../medium/kilobot_CSMA/kilo_medium.h"
#include "../../robot/kilobot.h"

namespace swarmnet_sim {

void Loop_event::exec() {
    Arena* arena_ptr = (Arena*)arena;
    Node* target_node = arena_ptr->get_node(to_id);
    target_node->loop_wrapper();
}

Loop_event::Loop_event(void* arena, double exec_time, int to_id) {
    this->arena = arena;
    this->exec_time = exec_time;
    this->to_id = to_id;
}

}  // namespace swarmnet_sim