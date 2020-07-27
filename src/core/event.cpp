#include "event.h"

namespace swarmnet_sim {
int Event::get_exec_tick() const { return this->exec_tick; }

int Event::get_from_id() const { return this->from_id; }

int Event::get_to_id() const { return this->to_id; }

void Event::exec() {}

Event::Event(int exec_tick, int from_id, int to_id) {
    this->exec_tick = exec_tick;
    this->from_id = from_id;
    this->to_id = to_id;
    // this->event_fn = event_fn;
}

bool CmpEventPtrs::operator()(const Event* lhs, const Event* rhs) const {
    return lhs->get_exec_tick() > rhs->get_exec_tick();
}

}  // namespace swarmnet_sim
