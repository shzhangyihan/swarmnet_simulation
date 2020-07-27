#include "event_queue.h"

#include <iostream>

namespace swarmnet_sim {

void Event_queue::push(Event* e) { this->queue.push(e); }

Event* Event_queue::pop() {
    Event* e = this->queue.top();
    this->queue.pop();
    return e;
}

Event* Event_queue::top() {
    Event* e = this->queue.top();
    return e;
}

bool Event_queue::empty() { return this->queue.empty(); }

}  // namespace swarmnet_sim
