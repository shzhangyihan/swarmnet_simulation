#ifndef __SIM_KILO_TX_EVENT_H__
#define __SIM_KILO_TX_EVENT_H__

#include "../../../core/arena.h"
#include "../../../core/event.h"

namespace swarmnet_sim {

#define MAX_RANDOM_DELAY_SECOND 0.01

class TX_start_event : public Event {
   public:
    void exec();
    TX_start_event(void* arena, int exec_tick, int from_id);
    // using Event::Event;
};

}  // namespace swarmnet_sim

#endif
