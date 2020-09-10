#include "kilo_medium.h"

#include <math.h>

#include <iostream>

#include "../../../core/arena.h"
#include "../../event/kilobot/rx_event.h"
#include "../../event/kilobot/tx_event.h"

namespace swarmnet_sim {

void Kilo_medium::start_tx(int tx_node_id, packet_t tx_packet,
                           float comm_radius) {
    Arena* arena_ptr = (Arena*)(this->arena);
    // int ticks_per_second = arena_ptr->get_config().get_ticks_per_second();
    int tx_time = sizeof(packet_t) * SPEED_BYTE_PER_SECOND;
    // std::cout << "tx ticks " << tx_ticks << std::endl;

    // std::cout << "start tx for " << tx_node_id << std::endl << std::flush;
    if (this->rx_counter_vector[tx_node_id] != 0) {
        // rx busy, end_tx
        TX_end_event* tx_end_event = new TX_end_event(
            arena_ptr, arena_ptr->get_sim_time() + tx_time, tx_node_id);
        tx_end_event->set_success(false);
        arena_ptr->add_event(tx_end_event);
        return;
    }

    int num_node = arena_ptr->get_config().get_num_robots();
    Node* tx_node = arena_ptr->get_node(tx_node_id);
    position2d_t tx_node_pos = tx_node->get_position();
    // std::cout << "send!!" << std::endl;
    for (int i = 0; i < num_node; i++) {
        if (i != tx_node_id) {
            // not self
            Node* rx_node = arena_ptr->get_node(i);
            position2d_t rx_node_pos = rx_node->get_position();
            float dist = calculate_dist(tx_node_pos, rx_node_pos);
            // std::cout << "dist " << dist << std::endl;
            if (dist > comm_radius) {
                // too far
            } else {
                // ok to send
                situated_sensing_t sensing;
                sensing.distance = dist;
                int theta_diff = atan2(tx_node_pos.y - rx_node_pos.y,
                                       tx_node_pos.x - rx_node_pos.x) *
                                 180 / M_PI;
                sensing.bearing =
                    fmod(theta_diff - rx_node_pos.theta + 360, 360);
                RX_start_event* rx_start_event =
                    new RX_start_event(arena_ptr, arena_ptr->get_sim_time(), i,
                                       tx_packet, sensing);
                arena_ptr->add_event(rx_start_event);
            }
        }
    }

    TX_end_event* tx_end_event = new TX_end_event(
        arena_ptr, arena_ptr->get_sim_time() + tx_time, tx_node_id);
    tx_end_event->set_success(true);
    arena_ptr->add_event(tx_end_event);
    return;
}

void Kilo_medium::end_tx(int tx_node_id, bool success) {
    // register the next tx
    Arena* arena_ptr = (Arena*)(this->arena);
    float next_tx_time_noise = ((float)std::rand() / (float)RAND_MAX - 0.5) *
                               TX_PERIOD_NOISE_RANGE_SECOND;
    float next_tx_time_local = TX_PERIOD_SECOND + next_tx_time_noise;
    Node* tx_node = arena_ptr->get_node(tx_node_id);
    float next_tx_time =
        ((Kilobot*)tx_node)->local_time_to_global_time(next_tx_time_local);
    // std::cout << "next_tx_time local " << next_tx_time_local << " global "
    //           << next_tx_time << std::endl;

    TX_start_event* tx_start_event = new TX_start_event(
        arena_ptr, arena_ptr->get_sim_time() + next_tx_time, tx_node_id);
    arena_ptr->add_event(tx_start_event);

    if (success) {
        ((Kilobot*)tx_node)->message_tx_success_wrapper();
    }
}

void Kilo_medium::start_rx(int rx_node_id, packet_t rx_packet,
                           situated_sensing_t sensing) {
    this->rx_counter_vector[rx_node_id]++;
    // check if rx already in use
    if (this->rx_counter_vector[rx_node_id] > 1) {
        // corrupt the current rx
        this->rx_buffer[rx_node_id].corrupted = true;
    } else {
        this->rx_buffer[rx_node_id].packet = rx_packet;
        this->rx_buffer[rx_node_id].sensing = sensing;
        this->rx_buffer[rx_node_id].corrupted = false;
    }

    Arena* arena_ptr = (Arena*)(this->arena);
    int rx_time = sizeof(packet_t) * SPEED_BYTE_PER_SECOND;
    // std::cout << "rx ticks " << rx_ticks << std::endl;

    RX_end_event* rx_end_event = new RX_end_event(
        arena_ptr, arena_ptr->get_sim_time() + rx_time, rx_node_id);
    arena_ptr->add_event(rx_end_event);
}

void Kilo_medium::end_rx(int rx_node_id) {
    Arena* arena_ptr = (Arena*)(this->arena);
    this->rx_counter_vector[rx_node_id]--;

    if (this->rx_buffer[rx_node_id].corrupted == false) {
        // not corrupted
        // drop with probability
        float dice_roll = rand() / double(RAND_MAX);
        int distance = this->rx_buffer[rx_node_id].sensing.distance;
        float threshold = MAX_SUCCESS_RATE * (1 + 1 / (distance - COMM_RADIUS));
        if (dice_roll <= threshold) {
            // no drop
            Node* rx_node = arena_ptr->get_node(rx_node_id);
            ((Kilobot*)rx_node)
                ->message_rx_wrapper(this->rx_buffer[rx_node_id].packet,
                                     this->rx_buffer[rx_node_id].sensing);
        }
    }
}

Kilo_medium::Kilo_medium(void* arena) {
    std::cout << "MEDIUM" << std::endl;
    this->arena = arena;
    for (int i = 0; i < ((Arena*)arena)->get_config().get_num_robots(); i++) {
        rx_buffer_t empty_entry;
        empty_entry.corrupted = false;
        this->rx_counter_vector.push_back(0);
        this->rx_buffer.push_back(empty_entry);
    }
}

extern "C" {
Medium* medium_builder(void* arena) {
    Medium* medium = new Kilo_medium(arena);
    return medium;
}
}

}  // namespace swarmnet_sim