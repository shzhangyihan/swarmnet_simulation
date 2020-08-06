#include "arena.h"

#include <dlfcn.h>

#include <iostream>

namespace swarmnet_sim {

Sim_config Arena::get_config() const { return this->conf; }

int Arena::get_current_tick() const { return this->current_tick; }

Node* Arena::get_node(int id) const { return this->node_vector[id]; }

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

void Arena::run() {
    // start the sim
    int max_tick =
        this->conf.get_ticks_per_second() * this->conf.get_duration();

    Event* end_event = new Event(this, max_tick, -1, -1);
    while (this->current_tick < max_tick) {
        if (this->event_queue.empty()) {
            this->event_queue.push(end_event);
        }
        Event* next_event = this->event_queue.top();
        int next_event_tick = next_event->get_exec_tick();
        if (this->check_collision(this, next_event_tick - current_tick)) {
            // collision happened, loop again
            continue;
        } else {
            // no collision, execuate current event
            this->event_queue.pop();
            update_simulation(next_event_tick - current_tick);
            next_event->exec();
            int exec_node_id = next_event->get_to_id();
            delete next_event;
            current_tick = next_event_tick;
            if (exec_node_id != -1) log_node(exec_node_id);
        }
        // std::cout << current_tick << std::endl;
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
        new_node->init();
        node_vector.push_back(new_node);
        log_node(i);
    }
    delete placement;
}

void Arena::add_event(Event* event) { this->event_queue.push(event); }

Arena::Arena(Sim_config conf) {
    this->conf = conf;
    this->current_tick = 0;
    srand(this->conf.get_rand_seed());
    // setup motion log
    motion_log = new Motion_log(this->conf.get_log_buf_size(),
                                this->conf.get_motion_log_name());
    std::string metadata = "";
    metadata = metadata + std::to_string(this->conf.get_arena_max_x()) + " ";
    metadata = metadata + std::to_string(this->conf.get_arena_max_y()) + " ";
    metadata = metadata + std::to_string(this->conf.get_num_robots()) + " ";
    metadata =
        metadata + std::to_string(this->conf.get_ticks_per_second()) + " ";
    metadata = metadata + std::to_string(this->conf.get_duration()) + "\n";
    motion_log->log(metadata);
    // get the physics engine function from handle
    check_collision = (collision_checker_t)dlsym(
        this->conf.get_physics_engine_dl_handle(), "check_collision");
    init_nodes();
}

Arena::~Arena() { delete motion_log; }

}  // namespace swarmnet_sim