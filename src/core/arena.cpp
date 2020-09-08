#include "arena.h"

#include <dlfcn.h>

#include <iostream>

namespace swarmnet_sim {

Sim_config Arena::get_config() const { return this->conf; }

float Arena::get_sim_time() const { return this->sim_time; }

Node* Arena::get_node(int id) const { return this->node_vector[id]; }

Medium* Arena::get_medium() const { return this->comm_medium; }

void Arena::update_simulation(float sim_time_diff) {
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

void Arena::log_node(float time, int id) {
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
    float velocity = node->get_velocity();
    if (node->get_collision_flag()) velocity = 0;
    log = log + std::to_string(velocity) + " ";
    // add color
    log = log + std::to_string(color.red) + " " + std::to_string(color.green) +
          " " + std::to_string(color.blue) + "\n";

    motion_log->log(log);
}

void Arena::run() {
    // start the sim
    int counter = 0;
    float max_time = this->conf.get_duration();

    Event* end_event = new Event(this, max_time, -1, -1);
    while (this->sim_time < max_time) {
        if (this->event_queue.empty()) {
            this->event_queue.push(end_event);
        }
        Event* next_event = this->event_queue.top();
        float next_event_time = next_event->get_exec_time();
        // std::cout << "current time " << sim_time << " next event time "
        //           << next_event_time << std::endl;

        float collision_time =
            this->check_collision(this, next_event_time - sim_time);
        // std::cout << "check return " << collision_tick << std::endl;
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
            next_event->exec();
            // int exec_node_id = next_event->get_to_id();
            delete next_event;
            // if (exec_node_id != -1) log_node(exec_node_id);
        }
        // std::cout << current_tick << std::endl;
        counter++;
        // if (counter > 10) break;
    }
    // std::cout << "finished" << std::endl;
    motion_log->flush();
}

void Arena::init_nodes() {
    void* placement_handle = this->conf.get_robot_placement_dl_handle();
    typedef std::vector<position2d_t>* (*placement_fn_t)(
        int arena_max_x, int arena_max_y, int num_nodes);
    placement_fn_t placement_fn =
        (placement_fn_t)dlsym(placement_handle, "robot_placement");
    std::vector<position2d_t>* placement =
        placement_fn(this->conf.get_arena_max_x(), this->conf.get_arena_max_y(),
                     this->conf.get_num_robots());

    typedef Node* (*robot_builder_t)(void*, int, position2d_t);
    robot_builder_t robot_builder = (robot_builder_t)dlsym(
        this->conf.get_robot_program_dl_handle(), "robot_builder");

    for (int i = 0; i < this->conf.get_num_robots(); i++) {
        Node* new_node = robot_builder(this, i, placement->at(i));
        node_vector.push_back(new_node);
        new_node->init_wrapper();
        log_node(i);
    }
    delete placement;
}

void Arena::add_event(Event* event) { this->event_queue.push(event); }

void Arena::log_metadata() {
    std::string metadata = "";
    metadata = metadata + std::to_string(this->conf.get_arena_max_x()) + " ";
    metadata = metadata + std::to_string(this->conf.get_arena_max_y()) + " ";
    metadata = metadata + std::to_string(this->conf.get_num_robots()) + " ";
    metadata = metadata + std::to_string(this->conf.get_duration()) + "\n";
    motion_log->log(metadata);
}

Arena::Arena(Sim_config conf) {
    this->conf = conf;
    this->sim_time = 0;
    srand(this->conf.get_rand_seed());
    // setup motion log
    motion_log = new Motion_log(this->conf.get_log_buf_size(),
                                this->conf.get_motion_log_name());
    log_metadata();
    // get the physics engine function from handle
    check_collision = (collision_checker_t)dlsym(
        this->conf.get_physics_engine_dl_handle(), "check_collision");
    // get the medium from handle
    typedef Medium* (*medium_builder_t)(void*);
    medium_builder_t medium_builder = (medium_builder_t)dlsym(
        this->conf.get_medium_dl_handle(), "medium_builder");
    this->comm_medium = medium_builder(this);
    init_nodes();
}

Arena::~Arena() {
    delete motion_log;
    for (int i = 0; i < this->conf.get_num_robots(); i++) {
        delete node_vector[i];
    }
    node_vector.clear();
}

}  // namespace swarmnet_sim