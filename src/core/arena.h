#ifndef __SIM_ARENA_H__
#define __SIM_ARENA_H__

#include <vector>

#include "config.h"
#include "event_queue.h"
#include "node.h"

namespace swarmnet_sim {
class Arena {
   public:
    void run();
    Sim_config get_config() const;
    int get_current_tick() const;
    Arena(Sim_config conf);

   private:
    void init_nodes();

    Sim_config conf;
    int current_tick;
    std::vector<Node *> node_vector;
    Event_queue event_queue;
};
}  // namespace swarmnet_sim

#endif