#ifndef __SIM_ARENA_H__
#define __SIM_ARENA_H__

#include <chrono>
#include <vector>

#include "../util/motion_log.h"
#include "config.h"
#include "event_queue.h"
#include "medium.h"
#include "node.h"

#define LOG_STATUS_INTERVAL 100

namespace swarmnet_sim {
class Arena {
   public:
    void run();
    void stop();
    Sim_config get_config() const;
    Node* get_node(int id) const;
    Medium* get_medium() const;
    double get_sim_time() const;
    void add_event(Event* event);
    void log_node(int id);
    // void log_node(int tick, int id);
    void log_node(double time, int id);
    void log_metadata();
    // void update_simulation(int ticks);
    void update_simulation(double sim_time_diff);
    void move_robot(int id, position2d_t pos);
    Arena(Sim_config conf);
    ~Arena();

   private:
    void init_nodes();

    Sim_config conf;
    // int current_tick;
    double sim_time;
    std::vector<Node*> node_vector;
    Event_queue event_queue;
    Motion_log* motion_log;
    Medium* comm_medium;
    Physics_engine* physics_engine;
    std::chrono::time_point<std::chrono::high_resolution_clock> sim_start_time;
    long physics_checking_time;
    long event_exec_time;
    long event_counter;
};
}  // namespace swarmnet_sim

#endif