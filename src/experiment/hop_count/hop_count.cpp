#include <iostream>

#include "../../plugin/robot/kilobot.h"

namespace swarmnet_sim {

#define UPDATE_PERIOD 2
#define NUM_COLOR 5
#define SEED_ID 0

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    int hop_count;
    int min_hop;
    float last_update_time;

   public:
    void collision() {}

    void display_color_on_hop(int hop) {
        int color_index = hop % NUM_COLOR;
        color_t next_color;
        switch (color_index) {
            case 0:
                next_color.red = 255;
                next_color.green = 0;
                next_color.blue = 0;
                break;
            case 1:
                next_color.red = 0;
                next_color.green = 255;
                next_color.blue = 0;
                break;
            case 2:
                next_color.red = 0;
                next_color.green = 0;
                next_color.blue = 255;
                break;
            case 3:
                next_color.red = 255;
                next_color.green = 255;
                next_color.blue = 0;
                break;
            case 4:
                next_color.red = 255;
                next_color.green = 255;
                next_color.blue = 255;
                break;
        }
        change_color(next_color);
    }

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        if (node_id == SEED_ID) return;
        int in_hop = packet.payload[0];
        float local_time = get_local_time();
        if (local_time - last_update_time > UPDATE_PERIOD) {
            if (min_hop != 0) {
                hop_count = min_hop + 1;
                // std::cout << node_id << ": " << hop_count << std::endl
                //   << std::flush;
                display_color_on_hop(hop_count);
                min_hop = 0;
            }
            last_update_time = local_time;
        }

        if (in_hop == 0) return;
        if (min_hop == 0) {
            min_hop = in_hop;
            return;
        }
        if (min_hop > in_hop) {
            min_hop = in_hop;
            return;
        }
    }

    bool message_tx(packet_t* packet) {
        packet->payload[0] = hop_count;
        return true;
        // if (hop_count > 0)
        //     return true;
        // else
        //     return false;
    }

    void message_tx_success() {
        // if (node_id == 0) return;
        // std::cout << node_id << std::endl << std::flush;
    }

    void init() {
        color_t c;

        hop_count = 0;
        min_hop = 0;
        if (node_id == SEED_ID) {
            // std::cout << node_id << std::endl;
            hop_count = 1;
            c.blue = 255;
            c.red = 255;
            c.green = 255;
        }
        c.blue = 0;
        c.red = 0;
        c.green = 0;
        change_color(c);
        last_update_time = get_local_time();
    }
};

// Make sure to include this line, with the argument the same as the class
// name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
