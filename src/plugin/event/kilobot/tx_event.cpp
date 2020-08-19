#include "tx_event.h"

#include "../../medium/kilobot_CSMA/kilo_medium.h"
#include "../../robot/kilobot.h"
#include "stdlib.h"

namespace swarmnet_sim {

void TX_start_event::exec() {
    // retrive sending packet
    Arena* arena_ptr = (Arena*)arena;
    Node* tx_node = arena_ptr->get_node(from_id);
    packet_t tx_packet;
    bool tx_state = ((Kilobot*)tx_node)->message_tx_wrapper(&tx_packet);
    if (tx_state) {
        // packet ready to send
        arena_ptr->get_medium()->send_packet(from_id, tx_packet, COMM_RADIUS);
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

}  // namespace swarmnet_sim