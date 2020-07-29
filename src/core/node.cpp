#include "node.h"

#include <iostream>

#include "arena.h"

namespace swarmnet_sim {
Node::Node(void* arena, int node_id, position2d_t pos) {
    this->arena = arena;
    this->node_id = node_id;
    this->pos = pos;
}

position2d_t Node::get_position() const { return this->pos; }

int Node::get_node_id() const { return this->node_id; }

void Node::set_position(position2d_t pos) { this->pos = pos; }

void Node::init() {}

}  // namespace swarmnet_sim