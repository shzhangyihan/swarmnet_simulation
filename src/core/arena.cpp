#include "arena.h"

#include <dlfcn.h>

#include <iostream>

namespace swarmnet_sim {

Sim_config Arena::get_config() const { return this->conf; }

int Arena::get_current_tick() const { return this->current_tick; }

void Arena::run() {
    // start the sim
    int max_tick =
        this->conf.get_ticks_per_second() * this->conf.get_duration();

    Event* end_event = new Event(max_tick, -1, -1);
    while (this->current_tick < max_tick) {
        if (this->event_queue.empty()) {
            this->event_queue.push(end_event);
        }
        Event* next_event = this->event_queue.pop();
    }
}

Arena::Arena(Sim_config conf) {
    std::cout << "init" << std::endl;
    this->conf = conf;
    this->current_tick = 0;
    srand(this->conf.get_rand_seed());

    void* placement_handle = this->conf.get_robot_placement_dl_handle();
    typedef std::vector<position2d_t>* (*placement_fn_t)(
        int arena_max_x, int arena_max_y, int num_nodes);
    std::cout << "before load" << std::endl;
    placement_fn_t placement_fn =
        (placement_fn_t)dlsym(placement_handle, "robot_placement");
    std::cout << "before call" << std::endl;
    std::vector<position2d_t>* placement =
        placement_fn(this->conf.get_arena_max_x(), this->conf.get_arena_max_y(),
                     this->conf.get_num_robots());
    for (int i = 0; i < this->conf.get_num_robots(); i++) {
        std::cout << i << " " << placement->at(i).x << ", "
                  << placement->at(i).y << std::endl;
    }
    delete placement;
}

}  // namespace swarmnet_sim