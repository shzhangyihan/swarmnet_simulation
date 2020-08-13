#ifndef __SIM_MEDIUM_H__
#define __SIM_MEDIUM_H__

#include "node.h"

namespace swarmnet_sim {

class Medium {
   public:
    virtual void send_packet(int tx_node_id, packet_t tx_packet,
                             float comm_radius) = 0;
    virtual void recv_packet(packet_t tx_packet,
                             situated_sensing_t sensing) = 0;
    Medium(void* arena) { this->arena = arena; };
    Medium(){};

   protected:
    void* arena;
};

}  // namespace swarmnet_sim
#endif