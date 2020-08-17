#include <iostream>

#include "../../plugin/robot/kilobot.h"

namespace swarmnet_sim {

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    long counter;

   public:
    void collision() {
        // counter++;
        // if (counter == 50000) {
        //     counter = 0;
        // float old_theta = pos.theta;
        std::cout << node_id << " collision" << std::endl << std::flush;
        if (node_id == 1) {
            turn(rand() % 360 - 180);
        } else {
            counter++;
            if (counter == 50000) {
                turn(rand() % 360 - 180);
            }
        }
        // go_forward();
    }

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        std::cout << node_id << " rx " << int(packet.payload[0]) << std::endl;
    }

    bool message_tx(packet_t* packet) {
        std::cout << node_id << " tx" << std::endl;
        packet->payload[0] = node_id;
        return true;
    }

    void message_tx_success() { std::cout << "tx_s" << std::endl; }

    void init() {
        counter = 0;
        std::cout << "init " << node_id << " at " << pos.x << ", " << pos.y
                  << std::endl;
        // if (node_id == 1) {
        color_t c;
        c.blue = 0;
        c.red = 255;
        c.green = 0;
        change_color(c);
        go_forward();
        // }
        // velocity = 1;
    }
};

// Make sure to include this line, with the argument the same as the class name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
