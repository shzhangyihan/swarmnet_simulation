#ifndef __SIM_KILO_MEDIUM_H__
#define __SIM_KILO_MEDIUM_H__

#include "../../../core/medium.h"
#include "../../robot/kilobot.h"

namespace swarmnet_sim {

class Kilo_medium : public Medium {
   public:
    void send_packet(int tx_node_id, packet_t tx_packet, float comm_radius);
    void recv_packet(packet_t tx_packet, situated_sensing_t sensing);
    Kilo_medium(void* arena);

   protected:
    void* arena;
};

extern "C" {
Medium* medium_builder(void* arena);
}

}  // namespace swarmnet_sim

#endif
