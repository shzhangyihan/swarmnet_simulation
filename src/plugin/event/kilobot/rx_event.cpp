#include "rx_event.h"

#include <iostream>

#include "../../robot/kilobot.h"

namespace swarmnet_sim {

void RX_start_event::exec() {
    ((Kilobot*)((Arena*)this->arena)->get_node(to_id))
        ->message_rx_wrapper(pkt, sensing);
}

RX_start_event::RX_start_event(void* arena, int exec_tick, int to_id,
                               packet_t pkt, situated_sensing_t sensing) {
    std::cout << "RX event created" << std::endl;
    this->arena = arena;
    this->exec_tick = exec_tick;
    this->to_id = to_id;
    this->pkt = pkt;
    this->sensing = sensing;
}

}  // namespace swarmnet_sim