#include "kilobot.h"

#include <dlfcn.h>

#include <iostream>

#include "../../util/lib_loader.h"

namespace swarmnet_sim {

void Kilobot::message_rx(packet_t packet, situated_sensing_t sensing) {}

bool Kilobot::message_tx(packet_t* packet) { return false; }

void Kilobot::message_tx_success() {}

void Kilobot::init() {}

}  // namespace swarmnet_sim