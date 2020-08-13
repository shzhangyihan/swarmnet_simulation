#include "kilobot.h"

#include <dlfcn.h>

#include <iostream>

#include "../../core/arena.h"
#include "../../util/lib_loader.h"
#include "../event/kilobot_CSMA/rx_event.h"
#include "../event/kilobot_CSMA/tx_event.h"

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
    this->add_event(tx_start_event);
}

Kilobot::Kilobot(void* arena, int node_id, position2d_t pos)
    : Node(arena, node_id, pos) {
    this->radius = ROBOT_RADIUS;
}

}  // namespace swarmnet_sim