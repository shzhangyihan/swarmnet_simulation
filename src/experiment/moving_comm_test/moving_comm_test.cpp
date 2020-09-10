#include <iostream>

#include "../../plugin/robot/kilobot.h"

namespace swarmnet_sim {

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    int tx_counter;

   public:
    void collision() {
        turn(rand() % 360 - 180);
        int select_color;
        color_t cur_color = this->get_color();
        while (true) {
            select_color = rand() % 3;
            color_t next_color;

            switch (select_color) {
                case 0:
                    next_color.red = 0;
                    next_color.blue = 0;
                    next_color.green = 255;
                    break;
                case 1:
                    next_color.red = 0;
                    next_color.blue = 255;
                    next_color.green = 0;
                    break;
                case 2:
                    next_color.red = 255;
                    next_color.blue = 0;
                    next_color.green = 0;
                    break;
            }

            if (next_color == cur_color)
                continue;
            else {
                this->change_color(next_color);
                // std::cout << "change color to " << int(next_color.red) << " "
                //           << int(next_color.green) << " "
                //           << int(next_color.blue) << " from "
                //           << int(cur_color.red) << " " <<
                //           int(cur_color.green)
                //           << " " << int(cur_color.blue) << std::endl
                //           << std::flush;
                break;
            }
        }
    }

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        int rx_counter = packet.payload[1] * 255 + packet.payload[2];
        std::cout << node_id << ": rx " << get_global_time() << " "
                  << int(packet.payload[0]) << " counter " << rx_counter
                  << std::endl
                  << std::flush;
    }

    bool message_tx(packet_t* packet) {
        // std::cout << node_id << " tx" << std::endl;
        packet->payload[0] = node_id;
        packet->payload[1] = (int)(tx_counter / 255);
        packet->payload[2] = tx_counter % 255;
        return true;
        // if (node_id == 0)
        //     return true;
        // else
        //     return false;
    }

    void message_tx_success() {
        std::cout << node_id << ": tx_s " << get_global_time()
                  << " with counter " << tx_counter << std::endl
                  << std::flush;
        tx_counter++;
        // tx_counter = tx_counter % 256;
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
        turn(rand() % 360 - 180);
        go_forward();
    }
};

// Make sure to include this line, with the argument the same as the class name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
