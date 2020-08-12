#include <iostream>

#include "../../plugin/robot/kilobot.h"

namespace swarmnet_sim {

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    long counter;

   public:
    void collision() {
        counter++;
        if (counter == 50000) {
            counter = 0;
            float old_theta = pos.theta;
            // std::cout << node_id << " collision" << std::endl << std::flush;
            // if (node_id == 1) change_theta(rand() % 360);
            change_theta(rand() % 360);
        }
    }

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        std::cout << "tx" << std::endl;
    }

    bool message_tx(packet_t* packet) { std::cout << "tx" << std::endl; }

    void message_tx_success() { std::cout << "tx_s" << std::endl; }

    void init() {
        counter = 0;
        std::cout << "init " << node_id << " at " << pos.x << ", " << pos.y
                  << std::endl;
        if (node_id == 1) {
            color_t c;
            c.blue = 0;
            c.red = 255;
            c.green = 0;
            set_color(c);
        }
        velocity = 1;
    }
};

// Make sure to include this line, with the argument the same as the class name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
