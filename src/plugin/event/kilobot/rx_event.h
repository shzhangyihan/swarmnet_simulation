#ifndef __SIM_KILO_RX_EVENT_H__
#define __SIM_KILO_RX_EVENT_H__

#include "../../../core/arena.h"
#include "../../../core/event.h"
#include "../../robot/kilobot.h"

namespace swarmnet_sim {

class RX_start_event : public Event {
   public:
    void exec();
    RX_start_event(void* arena, double exec_time, int to_id, packet_t pkt,
                   situated_sensing_t sensing);

   private:
    packet_t pkt;
    situated_sensing_t sensing;
};

class RX_end_event : public Event {
   public:
    void exec();
    RX_end_event(void* arena, double exec_time, int to_id);

   private:
    packet_t pkt;
    situated_sensing_t sensing;
};

}  // namespace swarmnet_sim

#endif
