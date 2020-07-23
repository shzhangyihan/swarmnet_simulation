#include "simulation.h"

#include <fstream>
#include <iostream>

namespace swarmnet_sim {

void start_simulation(char* config_file) {
    std::ifstream config_doc;
    // const char * config_dir = CMAKE_ROOT_DIR "/config.json";
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
    // std::string log_file_name =
    //     CMAKE_ROOT_DIR "/" +
    //     json_config.get("log_file", "default.txt").asString();

    std::cout << "Starting simulation with " << num_robot << " robots for "
              << duration << " seconds." << std::endl;

    Sim_config conf;
    conf.set_num_robots(num_robot);
    conf.set_arena_max_x(arena_max_x);
    conf.set_arena_max_y(arena_max_y);
    conf.set_duration(duration);
    conf.set_ticks_per_second(ticks_per_second);
    conf.set_rand_seed(rand_seed);
    conf.set_robot_placement_dl_handle(robot_placement_dl_handle);
    conf.set_robot_program_dl_handle(robot_program_dl_handle);
    conf.set_physics_engine_dl_handle(physics_engine_dl_handle);

    // Arena arena(conf);

    // std::cout << num_robot << std::endl;

    // SIM_NAMESPACE::Event_queue EQ;
    // SIM_NAMESPACE::Arena arena(&EQ, num_robot, 700, 700, log_file_name,
    //                            rand_seed, ticks_per_second,
    //                            experiment_duration);

    // SIM_NAMESPACE::position_t start_1;
    // start_1.x = 0;
    // start_1.y = 0;
    // SIM_NAMESPACE::position_t start_2;
    // start_2.x = 70;
    // start_2.y = 0;
    // SIM_NAMESPACE::position_t end_1 = calculate_end_pos(start_1, 45, 1, 100);
    // SIM_NAMESPACE::position_t end_2 = calculate_end_pos(start_2, 135, 1,
    // 100); double t_1, t_2; check_collision(start_1, end_1, start_2, end_2,
    // &t_1, &t_2); std::cout << t_1 << std::endl; std::cout << t_2 <<
    // std::endl;
}

}  // namespace swarmnet_sim
