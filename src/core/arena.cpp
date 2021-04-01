#include "arena.h"

#include <dlfcn.h>

#include <iostream>

namespace swarmnet_sim {

Sim_config Arena::get_config() const { return this->conf; }

double Arena::get_sim_time() const { return this->sim_time; }

Node* Arena::get_node(int id) const { return this->node_vector[id]; }

Medium* Arena::get_medium() const { return this->comm_medium; }

void Arena::update_simulation(double sim_time_diff) {
    int num_robots = this->conf.get_num_robots();
    for (int i = 0; i < num_robots; i++) {
        position2d_t cur_pos = this->node_vector[i]->get_position();
        position2d_t end_pos = calculate_future_pos(
            cur_pos, this->node_vector[i]->get_velocity(), sim_time_diff);

        this->node_vector[i]->set_position(end_pos);
        // std::cout << "Set robot " << i << " from (" << rob_pos.x << ", " <<
        // rob_pos.y << ") to (" << end_pos.x << ", " << end_pos.y << ") with t
        // = " << time << std::endl;
    }
}

void Arena::move_robot(int id, position2d_t pos) {
    this->node_vector[id]->set_position(pos);
}

void Arena::log_node(int id) {
    std::string log = "";
    Node* node = node_vector[id];
    position2d_t pos = node->get_position();
    color_t color = node->get_color();
    // add time
    log = log + std::to_string(this->sim_time) + " ";
    // add id
    log = log + std::to_string(id) + " ";
    // add position
    log = log + std::to_string(pos.x) + " " + std::to_string(pos.y) + " " +
          std::to_string(pos.theta) + " ";
    // add speed
    log = log + std::to_string(node->get_velocity()) + " ";
    // add color
    log = log + std::to_string(color.red) + " " + std::to_string(color.green) +
          " " + std::to_string(color.blue) + "\n";

    motion_log->log(log);
}

void Arena::log_node(double time, int id) {
    std::string log = "";
    Node* node = node_vector[id];
    position2d_t pos = node->get_position();
    color_t color = node->get_color();

    if (node->get_skip_logging_flag()) return;

    // add time
    log = log + std::to_string(time) + " ";
    // add id
    log = log + std::to_string(id) + " ";
    // add position
    log = log + std::to_string(pos.x) + " " + std::to_string(pos.y) + " " +
          std::to_string(pos.theta) + " ";
    // add speed
    double velocity = node->get_velocity();
    if (node->get_collision_flag()) velocity = 0;
    log = log + std::to_string(velocity) + " ";
    // add color
    log = log + std::to_string(color.red) + " " + std::to_string(color.green) +
          " " + std::to_string(color.blue) + "\n";

    motion_log->log(log);
}

void Arena::run() {
    // start the sim
    double prev_status_log_time = 0;
    std::cout << physics_engine->status_report();
    std::cout << comm_medium->status_report();

    sim_start_time = std::chrono::high_resolution_clock::now();
    double max_time = this->conf.get_duration();
    // std::cout << "start run" << std::endl;
    Event* end_event = new Event(this, max_time, -1, -1);
    while (this->sim_time < max_time) {
        if (this->event_queue.empty()) {
            this->event_queue.push(end_event);
        }

        if (this->sim_time - prev_status_log_time >= LOG_STATUS_INTERVAL) {
            std::cout << physics_engine->status_report();
            std::cout << comm_medium->status_report();
            prev_status_log_time = this->sim_time;
        }

        Event* next_event = this->event_queue.top();
        double next_event_time = next_event->get_exec_time();
        // std::cout << "current time " << sim_time << " next event time "
        //           << next_event_time << std::endl;
        if (next_event_time - sim_time == 0) {
            // don't need to check physics
            this->event_queue.pop();
            auto start = std::chrono::high_resolution_clock::now();
            next_event->exec();
            auto end = std::chrono::high_resolution_clock::now();
            event_exec_time +=
                std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                      start)
                    .count();
            delete next_event;
        } else {
            auto start = std::chrono::high_resolution_clock::now();
            double collision_time = this->physics_engine->check_collision(
                next_event_time - sim_time);
            auto end = std::chrono::high_resolution_clock::now();
            physics_checking_time +=
                std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                      start)
                    .count();
            // std::cout << sim_time << " check " << collision_time <<
            // std::endl;
            if (this->sim_time < 0) {
                std::cout << "Sim time going to negative. Abort!" << std::endl
                          << std::flush;
                exit(-1);
            }
            if (collision_time != -1) {
                // collision happened, loop again
                // update tick
                sim_time = sim_time + collision_time;
                // counter++;
                // if (counter == 10) break;
                continue;
            } else {
                // no collision, execuate current event
                this->event_queue.pop();
                // update_simulation(next_event_tick - current_tick);
                sim_time = next_event_time;
                auto start = std::chrono::high_resolution_clock::now();
                next_event->exec();
                auto end = std::chrono::high_resolution_clock::now();
                event_exec_time +=
                    std::chrono::duration_cast<std::chrono::microseconds>(end -
                                                                          start)
                        .count();
                event_counter++;
                // int exec_node_id = next_event->get_to_id();
                delete next_event;
            }
        }
        // std::cout << current_tick << std::endl;
        // counter++;
        // if (counter > 10) break;
    }
    // std::cout << "finished" << std::endl;
    this->stop();
}

void Arena::stop() {
    auto sim_end_time = std::chrono::high_resolution_clock::now();
    long sim_run_time = std::chrono::duration_cast<std::chrono::microseconds>(
                            sim_end_time - sim_start_time)
                            .count();
    for (Node* n : node_vector) {
        n->stop();
    }
    std::cout << "Simulation runs for: " << sim_run_time << " μs" << std::endl;
    std::cout << "Time spent in physics checking: " << physics_checking_time
              << " μs" << std::endl;
    std::cout << "Time spent in event execution: " << event_exec_time << " μs"
              << std::endl;
    std::cout << "Time spent in queue operation: "
              << event_queue.get_queue_operation_time() << " μs" << std::endl;
    std::cout << "Event count: " << event_counter << std::endl;
    std::cout << physics_engine->status_report();
    std::cout << comm_medium->status_report();
    motion_log->flush();
}

void Arena::init_nodes() {
    double arena_max_x = this->conf.get_arena_max_x();
    double arena_max_y = this->conf.get_arena_max_y();
    int num_robots = this->conf.get_num_robots();

    void* placement_handle = this->conf.get_robot_placement_dl_handle();
    typedef std::vector<position2d_t>* (*placement_fn_t)(
        int arena_max_x, int arena_max_y, int num_nodes);
    placement_fn_t placement_fn =
        (placement_fn_t)dlsym(placement_handle, "robot_placement");
    std::vector<position2d_t>* placement =
        placement_fn(arena_max_x, arena_max_y, num_robots);

    typedef Node* (*robot_builder_t)(void*, int, position2d_t);
    robot_builder_t robot_builder = (robot_builder_t)dlsym(
        this->conf.get_robot_program_dl_handle(), "robot_builder");

    std::cout << "Validate robot placement" << std::endl << std::flush;
    Node* tmp_node = robot_builder(this, 0, {0, 0, 0});
    double robot_radius = tmp_node->get_radius();
    delete tmp_node;
    bool valid_placement = true;
    for (int i = 0; i < num_robots; i++) {
        position2d_t pos_1 = placement->at(i);
        if (if_out_of_bound(pos_1, robot_radius, arena_max_x, arena_max_y)) {
            valid_placement = false;
            break;
        }
        for (int j = i + 1; j < num_robots; j++) {
            position2d_t pos_2 = placement->at(j);
            if (if_collision(pos_1, pos_2, robot_radius)) {
                valid_placement = false;
                break;
            }
        }
        if (!valid_placement) break;
    }
    if (!valid_placement) {
        std::cout << "Robot placement not valid. Abort!" << std::endl
                  << std::flush;
        exit(-1);
    }

    for (int i = 0; i < num_robots; i++) {
        Node* new_node = robot_builder(this, i, placement->at(i));
        node_vector.push_back(new_node);
        new_node->init_wrapper();
        log_node(i);
    }
    delete placement;
    std::cout << "Robot init finished" << std::endl << std::flush;
}

void Arena::add_event(Event* event) { this->event_queue.push(event); }

void Arena::log_metadata() {
    std::string metadata = "";
    metadata = metadata + std::to_string(this->conf.get_arena_max_x()) + " ";
    metadata = metadata + std::to_string(this->conf.get_arena_max_y()) + " ";
    metadata = metadata + std::to_string(this->conf.get_num_robots()) + " ";
    metadata = metadata + std::to_string(this->conf.get_duration()) + " ";
    metadata = metadata + std::to_string(this->conf.get_rand_seed()) + "\n";
    motion_log->log(metadata);
}

Arena::Arena(Sim_config conf) {
    this->conf = conf;
    this->physics_checking_time = 0;
    this->event_counter = 0;
    this->event_exec_time = 0;
    this->sim_time = 0;
    srand(this->conf.get_rand_seed());
    // setup motion log
    motion_log = new Motion_log(this->conf.get_log_buf_size(),
                                this->conf.get_motion_log_name());
    log_metadata();
    // get the physics engine from handle
    typedef Physics_engine* (*engine_builder_t)(void*);
    engine_builder_t engine_builder = (engine_builder_t)dlsym(
        this->conf.get_physics_engine_dl_handle(), "engine_builder");
    this->physics_engine = engine_builder(this);
    // get the medium from handle
    typedef Medium* (*medium_builder_t)(void*);
    medium_builder_t medium_builder = (medium_builder_t)dlsym(
        this->conf.get_medium_dl_handle(), "medium_builder");
    this->comm_medium = medium_builder(this);
    init_nodes();
    this->comm_medium->init();
    this->physics_engine->init();
}

Arena::~Arena() {
    delete motion_log;
    for (int i = 0; i < this->conf.get_num_robots(); i++) {
        delete node_vector[i];
    }
    node_vector.clear();
}

}  // namespace swarmnet_sim