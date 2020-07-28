#ifndef __SIM_EVENT_H__
#define __SIM_EVENT_H__

#include <functional>

namespace swarmnet_sim {

class Event {
   public:
    int get_exec_tick() const;
    int get_from_id() const;
    int get_to_id() const;
    virtual void exec();
    Event(int exec_tick, int from_id, int to_id);

   protected:
    int exec_tick;
    int from_id;
    int to_id;
};

struct CmpEventPtrs {
    bool operator()(const Event* lhs, const Event* rhs) const;
};

}  // namespace swarmnet_sim
#endif