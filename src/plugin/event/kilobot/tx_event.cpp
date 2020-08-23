#include "tx_event.h"

#include <iostream>

#include "../../medium/kilobot_CSMA/kilo_medium.h"
#include "../../robot/kilobot.h"
#include "stdlib.h"

namespace swarmnet_sim {

void TX_start_event::exec() {
    // retrive sending packet
    Arena* arena_ptr = (Arena*)arena;
    Node* tx_node = arena_ptr->get_node(from_id);
    packet_t tx_packet;
    // std::cout << "tx start event for " << from_id;
    bool tx_state = ((Kilobot*)tx_node)->message_tx_wrapper(&tx_packet);
    // std::cout << " state " << tx_state << std::endl << std::flush;
    if (tx_state) {
        // packet ready to send
        arena_ptr->get_medium()->start_tx(from_id, tx_packet, COMM_RADIUS);
    } else {
        // nothing to send
        return;
    }
}

TX_start_event::TX_start_event(void* arena, int exec_tick, int from_id) {
    int ticks_per_second = ((Arena*)arena)->get_config().get_ticks_per_second();
    int send_delay =
        std::rand() % (int)(MAX_RANDOM_DELAY_SECOND * ticks_per_second);
    this->arena = arena;
    this->exec_tick = exec_tick + send_delay;
    this->from_id = from_id;
    this->to_id = -1;
}

void TX_end_event::exec() {
    // retrive sending packet
    Arena* arena_ptr = (Arena*)arena;
    arena_ptr->get_medium()->end_tx(from_id, success);
}

void TX_end_event::set_success(bool success) { this->success = success; }

TX_end_event::TX_end_event(void* arena, int exec_tick, int from_id) {
    this->arena = arena;
    this->exec_tick = exec_tick;
    this->from_id = from_id;
    this->to_id = -1;
    this->success = true;
}

}  // namespace swarmnet_sim