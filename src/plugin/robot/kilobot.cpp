#include "kilobot.h"

// #include <dlfcn.h>
#include <math.h>

#include <iostream>

#include "../../core/arena.h"
#include "../../util/lib_loader.h"
#include "../event/kilobot/rx_event.h"
#include "../event/kilobot/tx_event.h"
#include "../event/kilobot/update_state_event.h"

namespace swarmnet_sim {

void Kilobot::collision() {}

void Kilobot::message_rx(packet_t packet, situated_sensing_t sensing) {}

bool Kilobot::message_tx(packet_t* packet) { return false; }

void Kilobot::message_tx_success() {}

void Kilobot::init() {}

void Kilobot::start() {
    this->init();
    Arena* arena_ptr = (Arena*)arena;
    float tx_delay = MAX_RANDOM_DELAY_SECOND *
                     arena_ptr->get_config().get_ticks_per_second();
    TX_start_event* tx_start_event = new TX_start_event(
        arena_ptr, arena_ptr->get_current_tick() + tx_delay, node_id);
    // this->add_event(tx_start_event);
}

void Kilobot::go_forward() {
    Arena* arena_ptr = (Arena*)this->arena;
    Update_state_event* event = new Update_state_event(
        arena_ptr, arena_ptr->get_current_tick(), node_id);
    event->update_velocity(VELOCITY_PER_SECOND);
    this->add_event(event);
}

void Kilobot::go_forward(float seconds) {}

void Kilobot::turn(float angle) {
    Arena* arena_ptr = (Arena*)this->arena;
    Update_state_event* event = new Update_state_event(
        arena_ptr, arena_ptr->get_current_tick(), node_id);
    position2d_t new_pos;
    new_pos.x = this->pos.x;
    new_pos.y = this->pos.y;
    float new_theta = fmod(this->pos.theta + angle + 360, 360);
    new_pos.theta = new_theta;
    event->update_position(new_pos);
    this->add_event(event);
}

void Kilobot::change_color(color_t color) {
    Arena* arena_ptr = (Arena*)this->arena;
    Update_state_event* event = new Update_state_event(
        arena_ptr, arena_ptr->get_current_tick(), node_id);
    event->update_color(color);
    this->add_event(event);
}

Kilobot::Kilobot(void* arena, int node_id, position2d_t pos)
    : Node(arena, node_id, pos) {
    this->radius = ROBOT_RADIUS;
}

}  // namespace swarmnet_sim