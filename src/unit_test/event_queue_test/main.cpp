#include <iostream>

#include "../../core/event_queue.h"

int main(int argc, char *argv[]) {
    swarmnet_sim::Event *e1 = new swarmnet_sim::Event(NULL, 1, 2, 3);
    swarmnet_sim::Event *e2 = new swarmnet_sim::Event(NULL, 10, 2, 3);
    swarmnet_sim::Event *e3 = new swarmnet_sim::Event(NULL, -11, 2, 3);
    swarmnet_sim::Event *e4 = new swarmnet_sim::Event(NULL, 100, 2, 3);
    swarmnet_sim::Event *e5 = new swarmnet_sim::Event(NULL, -21, 2, 3);
    swarmnet_sim::Event *e6 = new swarmnet_sim::Event(NULL, 2, 2, 3);

    swarmnet_sim::Event_queue e;
    e.push(e1);
    e.push(e2);
    e.push(e3);
    e.push(e4);
    e.push(e5);
    e.push(e6);

    while (!e.empty()) {
        swarmnet_sim::Event *ev = e.pop();
        std::cout << ev->get_exec_time() << std::endl;
    }
}