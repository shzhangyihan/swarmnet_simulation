#include "arena.h"

#include <dlfcn.h>

#include <iostream>

namespace swarmnet_sim {

Sim_config Arena::get_config() const { return this->conf; }

int Arena::get_current_tick() const { return this->current_tick; }

Node* Arena::get_node(int id) const { return this->node_vector[id]; }

Medium* Arena::get_medium() const { return this->comm_medium; }

void Arena::update_simulation(int ticks) {
    int num_robots = this->conf.get_num_robots();
    for (int i = 0; i < num_robots; i++) {
        position2d_t cur_pos = this->node_vector[i]->get_position();
        position2d_t end_pos = calculate_future_pos(
            cur_pos, this->node_vector[i]->get_velocity(),
            (float)ticks / this->conf.get_ticks_per_second());

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
    log = log + std::to_string(this->current_tick) + " ";
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

void Arena::log_node(int tick, int id) {
    std::string log = "";
    Node* node = node_vector[id];
    position2d_t pos = node->get_position();
    color_t color = node->get_color();

    if (node->get_skip_logging_flag()) return;

    // add time
    log = log + std::to_string(tick) + " ";
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
    int max_tick =
        this->conf.get_ticks_per_second() * this->conf.get_duration();

    Event* end_event = new Event(this, max_tick, -1, -1);
    while (this->current_tick < max_tick) {
        if (this->event_queue.empty()) {
            this->event_queue.push(end_event);
        }
        Event* next_event = this->event_queue.top();
        int next_event_tick = next_event->get_exec_tick();
        // std::cout << "current tick " << current_tick << " next tick "
        //           << next_event_tick << std::endl;

        int collision_tick =
            this->check_collision(this, next_event_tick - current_tick);
        // std::cout << "check return " << collision_tick << std::endl;
        if (collision_tick != -1) {
            // collision happened, loop again
            // update tick
            current_tick = current_tick + collision_tick;
            // counter++;
            // if (counter == 10) break;
            continue;
        } else {
            // no collision, execuate current event
            this->event_queue.pop();
            // update_simulation(next_event_tick - current_tick);
            current_tick = next_event_tick;
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
    metadata =
        metadata + std::to_string(this->conf.get_ticks_per_second()) + " ";
    metadata = metadata + std::to_string(this->conf.get_duration()) + "\n";
    motion_log->log(metadata);
}

Arena::Arena(Sim_config conf) {
    this->conf = conf;
    this->current_tick = 0;
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