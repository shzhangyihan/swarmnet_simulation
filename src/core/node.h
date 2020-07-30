#ifndef __SIM_NODE_H__
#define __SIM_NODE_H__

#include <string>

#include "physics_2d.h"

#define REGISTER_PROGRAM(NAME)                                        \
    extern "C" {                                                      \
    Node* robot_builder(void* arena, int node_id, position2d_t pos) { \
        Node* node = new NAME(arena, node_id, pos);                   \
        return node;                                                  \
    }                                                                 \
    }

typedef struct packet packet_t;
typedef struct situated_sensing situated_sensing_t;

namespace swarmnet_sim {

class Node {
   public:
    position2d_t get_position() const;
    int get_node_id() const;
    float get_velocity() const;
    float get_radius() const;
    void set_velocity(float velocity);
    void set_position(position2d_t pos);
    void change_theta(float theta_delta);
    virtual void init();
    virtual void collision();

    Node(void* arena, int node_id, position2d_t pos);

   protected:
    position2d_t pos;
    int node_id;
    float velocity;
    float radius;
    void* arena;
};

}  // namespace swarmnet_sim

#endif