#ifndef __SIM_KILOBOT_H__
#define __SIM_KILOBOT_H__

#include "../../core/node.h"

namespace swarmnet_sim {

#define ROBOT_RADIUS 8

typedef struct packet {
    unsigned char payload[9];
} packet_t;

typedef struct situated_sensing {
    int distance;
    int bearing;
} situated_sensing_t;

class Kilobot : public Node {
   public:
    virtual void collision();
    virtual void message_rx(packet_t packet, situated_sensing_t sensing);
    virtual bool message_tx(packet_t* packet);
    virtual void message_tx_success();
    virtual void init();

    Kilobot(void* arena, int node_id, position2d_t pos);
};

}  // namespace swarmnet_sim

#endif