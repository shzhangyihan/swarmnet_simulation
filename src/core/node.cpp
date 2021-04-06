#include "node.h"

#include <iostream>

#include "arena.h"

namespace swarmnet_sim {
Node::Node(void* arena, int node_id, position2d_t pos) {
    this->arena = arena;
    this->node_id = node_id;
    this->pos = pos;
    this->velocity = 0;
    this->color = {.red = 255, .green = 255, .blue = 255};
    this->collision_flag = false;
    this->skip_logging_flag = false;
    this->with_control_loop = true;
}

position2d_t Node::get_position() const { return this->pos; }

color_t Node::get_color() const { return this->color; }

int Node::get_node_id() const { return this->node_id; }

double Node::get_velocity() const { return this->velocity; }

double Node::get_radius() const { return this->radius; }

bool Node::get_collision_flag() const { return this->collision_flag; }

bool Node::get_skip_logging_flag() const { return this->skip_logging_flag; }

void Node::set_velocity(double velocity) { this->velocity = velocity; }

void Node::set_position(position2d_t pos) { this->pos = pos; }

void Node::set_color(color_t color) { this->color = color; }

void Node::set_collision_flag(bool collision_flag) {
    this->collision_flag = collision_flag;
}

void Node::set_skip_logging_flag(bool skip_logging_flag) {
    this->skip_logging_flag = skip_logging_flag;
}

void Node::change_theta(double theta_delta) {
    position2d_t new_pos;
    new_pos.x = pos.x;
    new_pos.y = pos.y;
    new_pos.theta = (int)(pos.theta + theta_delta) % 360;
    set_position(new_pos);
}

void Node::add_event(Event* event) { ((Arena*)this->arena)->add_event(event); }

void Node::end() {
    std::cout << node_id << " end" << std::endl << std::flush;
    this->velocity = 0;
}

void Node::collision_wrapper() {}

void Node::init_wrapper() {}

void Node::loop_wrapper() { this->with_control_loop = false; }

}  // namespace swarmnet_sim