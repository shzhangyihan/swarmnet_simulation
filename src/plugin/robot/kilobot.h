#ifndef __SIM_KILOBOT_H__
#define __SIM_KILOBOT_H__

#include "../../core/node.h"

namespace swarmnet_sim {

typedef struct packet {
    unsigned char payload[9];
} packet_t;

typedef struct situated_sensing {
    int distance;
    int bearing;
} situated_sensing_t;

class Kilobot : public Node {
   public:
    void message_rx(packet_t packet, situated_sensing_t sensing);
    bool message_tx(packet_t* packet);
    void message_tx_success();
    virtual void init();
    using Node::Node;
};

}  // namespace swarmnet_sim

#endif