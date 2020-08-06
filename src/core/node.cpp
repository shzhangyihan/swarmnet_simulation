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
}

position2d_t Node::get_position() const { return this->pos; }

color_t Node::get_color() const { return this->color; }

int Node::get_node_id() const { return this->node_id; }

float Node::get_velocity() const { return this->velocity; }

float Node::get_radius() const { return this->radius; }

void Node::set_velocity(float velocity) { this->velocity = velocity; }

void Node::set_position(position2d_t pos) { this->pos = pos; }

void Node::set_color(color_t color) { this->color = color; }

void Node::change_theta(float theta_delta) {
    position2d_t new_pos;
    new_pos.x = pos.x;
    new_pos.y = pos.y;
    new_pos.theta = (int)(pos.theta + theta_delta) % 360;
    set_position(new_pos);
}

void Node::collision() {}

void Node::init() {}

}  // namespace swarmnet_sim