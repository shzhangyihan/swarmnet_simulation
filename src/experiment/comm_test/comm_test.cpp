#include <iostream>

#include "../../plugin/robot/kilobot.h"

namespace swarmnet_sim {

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    int tx_counter;
    unsigned int rand_seed;

   public:
    void collision() { turn(rand_r(&rand_seed) % 360 - 180); }

    void loop() {
        std::cout << "Robot " << node_id << " loop at time "
                  << get_global_time() << std::endl;
    }

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        int rx_counter = packet.payload[1] * 255 + packet.payload[2];
        std::cout << "Robot " << node_id << " rx at time " << get_global_time()
                  << " (global) " << get_local_time() << " (local) from robot "
                  << int(packet.payload[0]) << " with counter " << rx_counter
                  << std::endl;
    }

    bool message_tx(packet_t* packet) {
        // std::cout << node_id << " tx" << std::endl;
        packet->payload[0] = node_id;
        packet->payload[1] = (int)(tx_counter / 255);
        packet->payload[2] = tx_counter % 255;
        return true;
    }

    void message_tx_success() {
        std::cout << "Robot " << node_id << " tx_success at time "
                  << get_global_time() << " (global) " << get_local_time()
                  << " (local) "
                  << " with counter " << tx_counter << std::endl;
        tx_counter++;
    }

    void init() {
        std::cout << "Init robot " << node_id << " at " << pos.x << ", "
                  << pos.y << std::endl;
        color_t c;
        c.blue = 0;
        c.red = 255;
        c.green = 0;
        change_color(c);
        tx_counter = 0;
        rand_seed = node_id;
        turn(rand_r(&rand_seed) % 360 - 180);
        go_forward();
    }
};

// Make sure to include this line, with the argument the same as the class name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
