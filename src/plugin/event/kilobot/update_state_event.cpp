#include "update_state_event.h"

#include <iostream>

#include "../../robot/kilobot.h"
#include "stdlib.h"

namespace swarmnet_sim {

void Update_state_event::exec() {
    Arena* arena_ptr = (Arena*)arena;

    Node* target_node = arena_ptr->get_node(to_id);
    target_node->set_position(pos);
    target_node->set_color(color);
    target_node->set_velocity(velocity);

    if (velocity == 0) {
        std::cout << to_id << ": update state ";
        std::cout << "pos " << pos.x << " " << pos.y << " " << pos.theta << " ";
        std::cout << "color " << int(color.red) << " " << int(color.green)
                  << " " << int(color.blue) << " ";
        std::cout << "velocity " << velocity;
        std::cout << std::endl << std::flush;
    } else {
        std::cout << to_id << " resume" << std::endl << std::flush;
    }

    this->log_node(to_id);
}

void Update_state_event::update_position(position2d_t pos) { this->pos = pos; }

void Update_state_event::update_color(color_t color) { this->color = color; }

void Update_state_event::update_velocity(float velocity) {
    this->velocity = velocity;
}

Update_state_event::Update_state_event(void* arena, int exec_tick, int to_id) {
    this->arena = arena;
    this->exec_tick = exec_tick;
    this->from_id = -1;
    this->to_id = to_id;
    // for (int i = 0; i < ATTRIBUTES_MAX; i++) {
    //     updated[i] = false;
    // }
    Arena* arena_ptr = (Arena*)arena;
    // std::cout << "123 " << to_id << std::endl;
    Node* target_node = arena_ptr->get_node(to_id);
    this->color = target_node->get_color();
    this->velocity = target_node->get_velocity();
    this->pos = target_node->get_position();
}

}  // namespace swarmnet_sim