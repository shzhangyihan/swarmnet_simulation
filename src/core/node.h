#ifndef __SIM_NODE_H__
#define __SIM_NODE_H__

#include "physics_2d.h"

namespace swarmnet_sim {
class Node {
   public:
    position2d_t get_position() const;
    int get_node_id() const;
    void set_position(position2d_t pos);
    virtual void init() = 0;

    Node(void* arena, int node_id, position2d_t pos);

   protected:
    position2d_t pos;
    int node_id;
    void* arena;
};
}  // namespace swarmnet_sim

#endif