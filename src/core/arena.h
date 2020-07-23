#ifndef __ARENA_H__
#define __ARENA_H__

#include <math.h>

#include <cstdint>
#include <fstream>
#include <string>
#include <vector>

#include "experiments/hopcount.h"
#include "robot.h"
#include "util.h"

#define MAX_PREDICT_TICKS 1000
#define LOG_BUFFER_SIZE 40
#define COMM_RADIUS 20
#define ROBOT_SPACING 16
#define MAX_DIST_SUCCESS_RATE 0.05
#define MIN_DIST_SUCCESS_RATE 1.0
#define FALLOFF_SHARPNESS 0.3

namespace SIM_NAMESPACE {
class state_log {
   public:
    void update_sim_tick(std::int32_t sim_tick);
    std::int32_t get_sim_tick();
    void update_log(std::string log);
    std::string get_log();
    // state_log(std::int32_t sim_tick, std::string log);
   private:
    std::int32_t sim_tick;
    std::string log;
};

class Arena {
   public:
    std::int32_t get_sim_tick();
    std::int32_t get_size_x();
    std::int32_t get_size_y();
    std::int32_t get_num_robots();
    std::int32_t get_event_version();
    std::int32_t get_ticks_per_second();
    std::vector<Robot_base *> get_robot_vector();
    void increment_event_version();
    void run();
    // proceed the simulation for "time" ticks
    void update_simulation_state(std::int32_t time);
    void log_simulation_state();
    void flush_state_log();
    void add_dummy_future_events();
    void robot_send_packet_event(std::int32_t robot_id);
    void robot_init_recv_packet_event(std::int32_t robot_id,
                                      std::int32_t distance, packet_t packet,
                                      situated_sensing_t sensing);
    void robot_end_recv_event(packet_t packet, situated_sensing_t sensing,
                              std::int32_t robot_id, std::int32_t collided);
    void register_event(std::int32_t sim_time, std::int32_t node_id,
                        std::int32_t version,
                        std::function<void(void)> event_fn);
    Arena(Event_queue *event_queue, std::int32_t num_robots,
          std::int32_t size_x, std::int32_t size_y, std::string log_file_name,
          std::int32_t rand_seed, std::int32_t ticks_per_second,
          std::int32_t experiment_duration);

   private:
    std::int32_t sim_tick;
    std::int32_t size_x, size_y;
    std::int32_t num_robots;
    std::vector<Robot_base *> robot_vector;
    Event_queue *event_queue;
    std::int32_t event_version;
    std::ofstream log_file;
    state_log log_buffer[LOG_BUFFER_SIZE];
    std::int32_t log_index;
    std::int32_t ticks_per_second;
    std::int32_t experiment_duration;
};
}  // namespace SIM_NAMESPACE

#endif