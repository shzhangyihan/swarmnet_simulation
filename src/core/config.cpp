#include "config.h"

namespace swarmnet_sim {
int Sim_config::get_num_robots() const { return num_robots; }

int Sim_config::get_arena_max_x() const { return arena_max_x; }

int Sim_config::get_arena_max_y() const { return arena_max_y; }

int Sim_config::get_rand_seed() const { return rand_seed; }

// int Sim_config::get_ticks_per_second() const { return ticks_per_second; }

int Sim_config::get_duration() const { return duration; }

int Sim_config::get_log_buf_size() const { return log_buf_size; }

void *Sim_config::get_robot_placement_dl_handle() const {
    return robot_placement_dl_handle;
}

void *Sim_config::get_robot_program_dl_handle() const {
    return robot_program_dl_handle;
}

void *Sim_config::get_physics_engine_dl_handle() const {
    return physics_engine_dl_handle;
}

void *Sim_config::get_medium_dl_handle() const { return medium_dl_handle; }

std::string Sim_config::get_motion_log_name() const { return motion_log_name; }

void Sim_config::set_num_robots(int val) { num_robots = val; }

void Sim_config::set_arena_max_x(int val) { arena_max_x = val; }

void Sim_config::set_arena_max_y(int val) { arena_max_y = val; }

void Sim_config::set_rand_seed(int val) { rand_seed = val; }

// void Sim_config::set_ticks_per_second(int val) { ticks_per_second = val; }

void Sim_config::set_duration(int val) { duration = val; }

void Sim_config::set_log_buf_size(int val) { log_buf_size = val; }

void Sim_config::set_robot_placement_dl_handle(void *val) {
    robot_placement_dl_handle = val;
}

void Sim_config::set_robot_program_dl_handle(void *val) {
    robot_program_dl_handle = val;
}

void Sim_config::set_physics_engine_dl_handle(void *val) {
    physics_engine_dl_handle = val;
}

void Sim_config::set_medium_dl_handle(void *val) { medium_dl_handle = val; }

void Sim_config::set_motion_log_name(std::string val) { motion_log_name = val; }

}  // namespace swarmnet_sim
