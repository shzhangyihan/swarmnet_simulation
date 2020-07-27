#ifndef __SIM_EVENT_QUEUE_H__
#define __SIM_EVENT_QUEUE_H__

#include <queue>
#include <vector>

#include "event.h"

namespace swarmnet_sim {

class Event_queue {
   public:
    void push(Event *e);
    Event *pop();
    Event *top();
    bool empty();

   private:
    std::priority_queue<Event *, std::vector<Event *>, CmpEventPtrs> queue;
};

}  // namespace swarmnet_sim
#endif
