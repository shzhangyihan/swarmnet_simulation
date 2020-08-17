#include "kilo_medium.h"

#include <iostream>

#include "../../../core/arena.h"
#include "../../event/kilobot/rx_event.h"

namespace swarmnet_sim {

void Kilo_medium::send_packet(int tx_node_id, packet_t tx_packet,
                              float comm_radius) {
    Arena* arena_ptr = (Arena*)(this->arena);
    int num_node = arena_ptr->get_config().get_num_robots();
    Node* tx_node = arena_ptr->get_node(tx_node_id);
    std::cout << "send!!" << std::endl;
    for (int i = 0; i < num_node; i++) {
        if (i != tx_node_id) {
            // not self
            Node* rx_node = arena_ptr->get_node(i);
            float dist = calculate_dist(tx_node->get_position(),
                                        rx_node->get_position());
            std::cout << "dist " << dist << std::endl;
            if (dist > comm_radius) {
                // too far
            } else {
                // ok to send
                situated_sensing_t sensing;
                RX_start_event* rx_start_event =
                    new RX_start_event(arena_ptr, arena_ptr->get_current_tick(),
                                       i, tx_packet, sensing);
                arena_ptr->add_event(rx_start_event);
            }
        }
    }
}

void Kilo_medium::recv_packet(packet_t tx_packet, situated_sensing_t sensing) {
    std::cout << "recv!!" << std::endl;
}

Kilo_medium::Kilo_medium(void* arena) {
    std::cout << "MEDIUM" << std::endl;
    this->arena = arena;
}

extern "C" {
Medium* medium_builder(void* arena) {
    Medium* medium = new Kilo_medium(arena);
    return medium;
}
}

}  // namespace swarmnet_sim