#include <iostream>

#include "../../plugin/robot/kilobot.h"

namespace swarmnet_sim {

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    int tx_counter;

   public:
    void collision() {}

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        std::cout << node_id << ": rx " << get_global_time() << " "
                  << int(packet.payload[0]) << " counter "
                  << int(packet.payload[1]) << std::endl
                  << std::flush;
    }

    bool message_tx(packet_t* packet) {
        // std::cout << node_id << " tx" << std::endl;
        packet->payload[0] = node_id;
        packet->payload[1] = tx_counter;

        return true;
    }

    void message_tx_success() {
        std::cout << node_id << ": tx_s " << get_global_time()
                  << " with counter " << tx_counter << std::endl
                  << std::flush;
        tx_counter++;
        tx_counter = tx_counter % 256;
    }

    void init() {
        std::cout << "init " << node_id << " at " << pos.x << ", " << pos.y
                  << std::endl;
        color_t c;
        c.blue = 0;
        c.red = 255;
        c.green = 0;
        change_color(c);
        tx_counter = 0;
    }
};

// Make sure to include this line, with the argument the same as the class name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
