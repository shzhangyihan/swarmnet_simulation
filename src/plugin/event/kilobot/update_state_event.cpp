#include "update_state_event.h"

#include <iostream>

#include "../../robot/kilobot.h"
#include "stdlib.h"

namespace swarmnet_sim {

void Update_state_event::exec() {
    Arena* arena_ptr = (Arena*)arena;
    // std::cout << "update state ";

    Node* target_node = arena_ptr->get_node(to_id);
    // if (updated[Position]) {
    target_node->set_position(pos);
    //     std::cout << "pos ";
    // }
    // if (updated[Color]) {
    target_node->set_color(color);
    std::cout << "color ";
    // }
    // if (updated[Velocity]) {
    target_node->set_velocity(velocity);
    //     std::cout << "velo ";
    // }
    std::cout << std::endl << std::flush;
    this->log_node(to_id);
}

void Update_state_event::update_position(position2d_t pos) {
    // std::cout << "try pos" << std::endl << std::flush;

    // this->updated[Position] = true;
    this->pos = pos;
}

void Update_state_event::update_color(color_t color) {
    std::cout << "try color" << std::endl << std::flush;

    // this->updated[Color] = true;
    this->color = color;
}

void Update_state_event::update_velocity(float velocity) {
    // std::cout << "try velo" << std::endl << std::flush;

    // this->updated[Velocity] = true;
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