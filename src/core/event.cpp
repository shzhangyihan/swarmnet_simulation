#include "event.h"

#include "arena.h"

namespace swarmnet_sim {
double Event::get_exec_time() const { return this->exec_time; }

int Event::get_from_id() const { return this->from_id; }

int Event::get_to_id() const { return this->to_id; }

void Event::exec() {}

void Event::log_node(int id) {
    ((Arena*)this->arena)->log_node(this->exec_time, id);
}

Event::Event(void* arena, double exec_time, int from_id, int to_id) {
    this->arena = arena;
    this->exec_time = exec_time;
    this->from_id = from_id;
    this->to_id = to_id;
}

Event::Event() {}

bool CmpEventPtrs::operator()(const Event* lhs, const Event* rhs) const {
    return lhs->get_exec_time() > rhs->get_exec_time();
}

}  // namespace swarmnet_sim
