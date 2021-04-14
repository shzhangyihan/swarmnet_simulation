#include "event_queue.h"

#include <chrono>
#include <iostream>

namespace swarmnet_sim {

void Event_queue::push(Event* e) {
    auto start = std::chrono::high_resolution_clock::now();

    this->queue.push(e);

    auto end = std::chrono::high_resolution_clock::now();
    queue_operation_time +=
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count();
}

Event* Event_queue::pop() {
    auto start = std::chrono::high_resolution_clock::now();

    Event* e = this->queue.top();
    this->queue.pop();

    auto end = std::chrono::high_resolution_clock::now();
    queue_operation_time +=
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count();

    return e;
}

Event* Event_queue::top() {
    auto start = std::chrono::high_resolution_clock::now();

    Event* e = this->queue.top();

    auto end = std::chrono::high_resolution_clock::now();
    queue_operation_time +=
        std::chrono::duration_cast<std::chrono::microseconds>(end - start)
            .count();
    return e;
}

bool Event_queue::empty() const { return this->queue.empty(); }

int Event_queue::size() const { return this->queue.size(); }

long Event_queue::get_queue_operation_time() const {
    return this->queue_operation_time;
}

}  // namespace swarmnet_sim
