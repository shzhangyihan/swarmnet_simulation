#ifndef __SIM_KILO_LOOP_EVENT_H__
#define __SIM_KILO_LOOP_EVENT_H__

#include "../../../core/arena.h"
#include "../../../core/event.h"
#include "../../robot/kilobot.h"

namespace swarmnet_sim {

class Loop_event : public Event {
   public:
    void exec();
    Loop_event(void* arena, double exec_time, int to_id);
};

}  // namespace swarmnet_sim

#endif
