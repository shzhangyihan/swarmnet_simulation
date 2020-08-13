#include "config_parser.h"

#include <fstream>
#include <iostream>

namespace swarmnet_sim {

Sim_config parse_config(char* config_file) {
    std::ifstream config_doc;
    std::string config_dir =
        std::string(CMAKE_ROOT_DIR) + "/" + std::string(config_file);
    config_doc.open(config_dir, std::ifstream::in);
    Json::Value json_config;
    config_doc >> json_config;
    config_doc.close();

    int num_robot = json_config.get("robots", 1).asInt();
    int rand_seed = json_config.get("rand_seed", (uint32_t)time(NULL)).asInt();
    int arena_max_x = json_config.get("arena_max_x", 700).asInt();
    int arena_max_y = json_config.get("arena_max_y", 700).asInt();
    int duration = json_config.get("experiment_duration", 60).asInt();
    int ticks_per_second = json_config.get("ticks_per_second", 1000).asInt();
    int log_buf_size = json_config.get("log_buf_size", 50).asInt();
    std::string motion_log_name =
        json_config.get("motion_log", "./motion_log/default_log.txt")
            .asString();

    std::string robot_placement_dl =
        BUILD_DIR "/" +
        json_config
            .get("robot_placement",
                 "./src/experiment/default/libdefault_placement.so")
            .asString();
    void* robot_placement_dl_handle = get_dl_handle(robot_placement_dl.c_str());

    std::string robot_program_dl =
        BUILD_DIR "/" +
        json_config
            .get("robot_placement",
                 "./src/experiment/default/libdefault_program.so")
            .asString();
    void* robot_program_dl_handle = get_dl_handle(robot_program_dl.c_str());

    std::string physics_engine_dl =
        BUILD_DIR "/" +
        json_config
            .get("robot_placement", "./src/plugin/engine/libdefault_engine.so")
            .asString();
    void* physics_engine_dl_handle = get_dl_handle(physics_engine_dl.c_str());

    std::string medium_dl =
        BUILD_DIR "/" +
        json_config
            .get("comm_medium",
                 "./src/plugin/medium/kilobot_CSMA/libkilobot_medium.so")
            .asString();
    void* medium_dl_handle = get_dl_handle(medium_dl.c_str());

    std::cout << "Starting simulation with " << num_robot << " robots for "
              << duration << " seconds." << std::endl;

    Sim_config conf;
    conf.set_num_robots(num_robot);
    conf.set_arena_max_x(arena_max_x);
    conf.set_arena_max_y(arena_max_y);
    conf.set_duration(duration);
    conf.set_ticks_per_second(ticks_per_second);
    conf.set_rand_seed(rand_seed);
    conf.set_log_buf_size(log_buf_size);
    conf.set_robot_placement_dl_handle(robot_placement_dl_handle);
    conf.set_robot_program_dl_handle(robot_program_dl_handle);
    conf.set_physics_engine_dl_handle(physics_engine_dl_handle);
    conf.set_medium_dl_handle(medium_dl_handle);
    conf.set_motion_log_name(motion_log_name);

    return conf;
}

}  // namespace swarmnet_sim
