#ifndef __SIM_NODE_H__
#define __SIM_NODE_H__

#include <string>

#include "event.h"
#include "physics_2d.h"

#define REGISTER_PROGRAM(NAME)                                        \
    extern "C" {                                                      \
    Node* robot_builder(void* arena, int node_id, position2d_t pos) { \
        Node* node = new NAME(arena, node_id, pos);                   \
        return node;                                                  \
    }                                                                 \
    }

namespace swarmnet_sim {

typedef struct packet packet_t;
typedef struct situated_sensing situated_sensing_t;

class Node {
   public:
    position2d_t get_position() const;
    color_t get_color() const;
    int get_node_id() const;
    float get_velocity() const;
    float get_radius() const;
    void set_velocity(float velocity);
    void set_position(position2d_t pos);
    void set_color(color_t color);
    void change_theta(float theta_delta);
    void add_event(Event* event);
    void stop();
    virtual void init_wrapper();
    virtual void collision_wrapper();

    Node(void* arena, int node_id, position2d_t pos);

   protected:
    position2d_t pos;
    color_t color;
    int node_id;
    float velocity;
    float radius;
    bool collision_flag;
    bool skip_logging_flag;
    void* arena;
};

}  // namespace swarmnet_sim

#endif