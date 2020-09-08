#ifndef __SIM_KILO_TX_EVENT_H__
#define __SIM_KILO_TX_EVENT_H__

#include "../../../core/arena.h"
#include "../../../core/event.h"

namespace swarmnet_sim {

#define MAX_RANDOM_DELAY_SECOND 0.01

class TX_start_event : public Event {
   public:
    void exec();
    TX_start_event(void* arena, float exec_time, int from_id);
};

class TX_end_event : public Event {
   public:
    void exec();
    void set_success(bool success);
    TX_end_event(void* arena, float exec_time, int from_id);

   private:
    bool success;
};

}  // namespace swarmnet_sim

#endif
