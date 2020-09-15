#ifndef __SIM_MEDIUM_H__
#define __SIM_MEDIUM_H__

#include "node.h"

namespace swarmnet_sim {

class Medium {
   public:
    virtual void start_tx(int tx_node_id) = 0;
    virtual void end_tx(int tx_node_id, bool success) = 0;
    virtual void start_rx(int rx_node_id, packet_t tx_packet,
                          situated_sensing_t sensing) = 0;
    virtual void end_rx(int rx_node_id) = 0;
    virtual void init() = 0;
    Medium(void* arena) { this->arena = arena; };
    Medium(){};

   protected:
    void* arena;
};

}  // namespace swarmnet_sim
#endif