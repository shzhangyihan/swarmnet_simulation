#ifndef __SIM_ARENA_H__
#define __SIM_ARENA_H__

#include <vector>

#include "../util/motion_log.h"
#include "config.h"
#include "event_queue.h"
#include "medium.h"
#include "node.h"

namespace swarmnet_sim {
class Arena {
   public:
    void run();
    Sim_config get_config() const;
    Node* get_node(int id) const;
    Medium* get_medium() const;
    float get_sim_time() const;
    void add_event(Event* event);
    void log_node(int id);
    // void log_node(int tick, int id);
    void log_node(float time, int id);
    void log_metadata();
    // void update_simulation(int ticks);
    void update_simulation(float sim_time_diff);
    void move_robot(int id, position2d_t pos);
    Arena(Sim_config conf);
    ~Arena();

   private:
    void init_nodes();

    Sim_config conf;
    // int current_tick;
    float sim_time;
    std::vector<Node*> node_vector;
    Event_queue event_queue;
    Motion_log* motion_log;
    Medium* comm_medium;
    typedef float (*collision_checker_t)(Arena*, float);
    collision_checker_t check_collision;
    long physics_checking_time;
    long event_counter;
};
}  // namespace swarmnet_sim

#endif