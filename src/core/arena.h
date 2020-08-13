#ifndef __SIM_ARENA_H__
#define __SIM_ARENA_H__

#include <vector>

#include "../util/motion_log.h"
#include "config.h"
#include "event_queue.h"
#include "node.h"
#include "medium.h"

namespace swarmnet_sim {
class Arena {
   public:
    void run();
    Sim_config get_config() const;
    Node* get_node(int id) const;
    Medium* get_medium() const;
    int get_current_tick() const;
    void add_event(Event* event);
    void log_node(int id);
    void log_metadata();
    void update_simulation(int ticks);
    void move_robot(int id, position2d_t pos);
    Arena(Sim_config conf);
    ~Arena();

   private:
    void init_nodes();

    Sim_config conf;
    int current_tick;
    std::vector<Node*> node_vector;
    Event_queue event_queue;
    Motion_log* motion_log;
    Medium* comm_medium;
    typedef int (*collision_checker_t)(Arena*, int);
    collision_checker_t check_collision;
};
}  // namespace swarmnet_sim

#endif