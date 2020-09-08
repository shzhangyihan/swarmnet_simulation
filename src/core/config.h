#ifndef __SIM_CONFIG_H__
#define __SIM_CONFIG_H__

#include <string>

namespace swarmnet_sim {
class Sim_config {
   public:
    int get_num_robots() const;
    int get_arena_max_x() const;
    int get_arena_max_y() const;
    int get_rand_seed() const;
    // int get_ticks_per_second() const;
    int get_duration() const;
    int get_log_buf_size() const;
    void *get_robot_placement_dl_handle() const;
    void *get_robot_program_dl_handle() const;
    void *get_physics_engine_dl_handle() const;
    void *get_medium_dl_handle() const;
    std::string get_motion_log_name() const;

    void set_num_robots(int val);
    void set_arena_max_x(int val);
    void set_arena_max_y(int val);
    void set_rand_seed(int val);
    // void set_ticks_per_second(int val);
    void set_duration(int val);
    void set_log_buf_size(int val);
    void set_robot_placement_dl_handle(void *val);
    void set_robot_program_dl_handle(void *val);
    void set_physics_engine_dl_handle(void *val);
    void set_medium_dl_handle(void *val);
    void set_motion_log_name(std::string val);

   private:
    int num_robots;
    int arena_max_x;
    int arena_max_y;
    int rand_seed;
    // int ticks_per_second;
    int duration;
    int log_buf_size;
    void *robot_placement_dl_handle;
    void *robot_program_dl_handle;
    void *physics_engine_dl_handle;
    void *medium_dl_handle;
    std::string motion_log_name;
    // void *robot_plugin_dl_handle;
};

}  // namespace swarmnet_sim

#endif
