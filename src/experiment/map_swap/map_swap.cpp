#include <iostream>

#include "../../plugin/robot/kilobot.h"

namespace swarmnet_sim {

using namespace std;

typedef struct {
    int x;
    int y;
} state_t;

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    state_t local_state, rx_state;

   public:
    void collision() {}

    void message_rx(packet_t packet, situated_sensing_t sensing) {}

    bool message_tx(packet_t* packet) { return true; }

    void message_tx_success() {}

    void init() {
        switch (node_id) {
            case 0:
                local_state = {1, 1};
                change_color({0, 0, 255});
                break;
            case 1:
                local_state = {1, 0};
                change_color({0, 255, 0});
                break;
            case 2:
                local_state = {0, 1};
                change_color({255, 0, 0});
                break;
            case 3:
                local_state = {0, 0};
                change_color({255, 255, 255});
                break;
        }
    }
};

// Make sure to include this line, with the argument the same as the class name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
