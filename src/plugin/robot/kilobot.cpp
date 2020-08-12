#include "kilobot.h"

#include <dlfcn.h>

#include <iostream>

#include "../../util/lib_loader.h"

namespace swarmnet_sim {

void Kilobot::collision() {}

void Kilobot::message_rx(packet_t packet, situated_sensing_t sensing) {}

bool Kilobot::message_tx(packet_t* packet) { return false; }

void Kilobot::message_tx_success() {}

void Kilobot::init() {}

Kilobot::Kilobot(void* arena, int node_id, position2d_t pos)
    : Node(arena, node_id, pos) {
    this->radius = ROBOT_RADIUS;
}

}  // namespace swarmnet_sim