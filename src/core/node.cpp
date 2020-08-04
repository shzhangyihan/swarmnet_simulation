#include "node.h"

#include <iostream>

#include "arena.h"

namespace swarmnet_sim {
Node::Node(void* arena, int node_id, position2d_t pos) {
    this->arena = arena;
    this->node_id = node_id;
    this->pos = pos;
    this->velocity = 0;
}

position2d_t Node::get_position() const { return this->pos; }

int Node::get_node_id() const { return this->node_id; }

float Node::get_velocity() const { return this->velocity; }

float Node::get_radius() const { return this->radius; }

void Node::set_velocity(float velocity) { this->velocity = velocity; }

void Node::set_position(position2d_t pos) {
    // std::cout << node_id << " set pos from " << this->pos.x << ", "
    //   << this->pos.y << " to " << pos.x << ", " << pos.y << std::endl;
    this->pos = pos;
}

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