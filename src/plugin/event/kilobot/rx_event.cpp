#include "rx_event.h"

#include <iostream>

#include "../../medium/kilobot_CSMA/kilo_medium.h"
#include "../../robot/kilobot.h"

namespace swarmnet_sim {

void RX_start_event::exec() {
    Arena* arena_ptr = (Arena*)arena;
    arena_ptr->get_medium()->start_rx(to_id, this->pkt, this->sensing);
}

RX_start_event::RX_start_event(void* arena, int exec_tick, int to_id,
                               packet_t pkt, situated_sensing_t sensing) {
    // std::cout << "RX event created" << std::endl;
    this->arena = arena;
    this->exec_tick = exec_tick;
    this->to_id = to_id;
    this->pkt = pkt;
    this->sensing = sensing;
}

void RX_end_event::exec() {
    Arena* arena_ptr = (Arena*)arena;
    arena_ptr->get_medium()->end_rx(to_id);
}

RX_end_event::RX_end_event(void* arena, int exec_tick, int to_id) {
    this->arena = arena;
    this->exec_tick = exec_tick;
    this->to_id = to_id;
}

}  // namespace swarmnet_sim