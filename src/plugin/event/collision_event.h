#ifndef __SIM_COLLISION_EVENT_H__
#define __SIM_COLLISION_EVENT_H__

#include "../../core/arena.h"
#include "../../core/event.h"

namespace swarmnet_sim {

class Collision_event : public Event {
   public:
    void exec();
    using Event::Event;
};

}  // namespace swarmnet_sim

#endif
