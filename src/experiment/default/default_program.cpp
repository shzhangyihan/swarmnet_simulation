#include <iostream>

#include "../../plugin/robot/kilobot.h"

namespace swarmnet_sim {

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;

   public:
    void collision() {
        float old_theta = pos.theta;
        change_theta(rand() % 360);
    }

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        std::cout << "tx" << std::endl;
    }

    bool message_tx(packet_t* packet) { std::cout << "tx" << std::endl; }

    void message_tx_success() { std::cout << "tx_s" << std::endl; }

    void init() {
        std::cout << "init " << node_id << " at " << pos.x << ", " << pos.y
                  << std::endl;
        velocity = 1;
    }
};

// Make sure to include this line, with the argument the same as the class name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
