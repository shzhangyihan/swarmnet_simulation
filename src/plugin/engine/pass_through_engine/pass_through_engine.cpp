#include "pass_through_engine.h"

#include <math.h>

#include <algorithm>
#include <iostream>
#include <vector>

#include "../../event/collision_event.h"

namespace swarmnet_sim {

float Pass_through_engine::check_collision(float future_time) {
    Arena* arena_ptr = (Arena*)this->arena;
    Sim_config config = arena_ptr->get_config();
    int num_robots = config.get_num_robots();
    position2d_t future_pos[num_robots];
    std::vector<int> collided_node;
    bool violated = false;
    int future_ticks = TICK_PER_SECOND * future_time;
    for (int t = 1; t <= future_ticks; t++) {
        if (t > 1) {
            // no collision up front, update the flags
            for (int i = 0; i < num_robots; i++) {
                if (arena_ptr->get_node(i)->get_collision_flag()) {
                    arena_ptr->log_node(i);
                }
                arena_ptr->get_node(i)->set_collision_flag(false);
                arena_ptr->get_node(i)->set_skip_logging_flag(false);
            }
        }

        for (int i = 0; i < num_robots; i++) {
            future_pos[i] =
                calculate_future_pos(arena_ptr->get_node(i)->get_position(),
                                     arena_ptr->get_node(i)->get_velocity(),
                                     (float)1 / TICK_PER_SECOND);
        }

        for (int i = 0; i < num_robots; i++) {
            if (check_out_of_bound(
                    future_pos[i], arena_ptr->get_node(i)->get_radius(),
                    config.get_arena_max_x(), config.get_arena_max_y())) {
                // need to add collison event
                violated = true;
                Collision_event* new_event = new Collision_event(
                    arena_ptr,
                    arena_ptr->get_sim_time() + (float)t / TICK_PER_SECOND, -1,
                    i);
                arena_ptr->add_event(new_event);
                collided_node.push_back(i);
            }
        }

        // need to update location
        for (int i = 0; i < num_robots; i++) {
            if (std::find(collided_node.begin(), collided_node.end(), i) ==
                collided_node.end()) {
                // node not collided, update
                arena_ptr->move_robot(i, future_pos[i]);
            }
        }

        if (violated) {
            // std::cout << "violated at " << t << std::endl << std::flush;
            return (float)t / TICK_PER_SECOND;
        }
    }

    return -1;
}

bool Pass_through_engine::check_robot_collision(position2d_t pos_1,
                                                position2d_t pos_2,
                                                int radius) {
    float dist = calculate_dist(pos_1, pos_2);
    if (dist > radius * 2) {
        return false;
    } else {
        return true;
    }
}

bool Pass_through_engine::check_out_of_bound(position2d_t pos, int radius,
                                             int x_max, int y_max) {
    float x = pos.x;
    float y = pos.y;

    if (x - radius < 0 || x + radius > x_max) {
        return true;
    }
    if (y - radius < 0 || y + radius > y_max) {
        return true;
    }

    return false;
}

void Pass_through_engine::init() {}

Pass_through_engine::Pass_through_engine(void* arena) { this->arena = arena; }

extern "C" {
Physics_engine* engine_builder(void* arena) {
    Physics_engine* engine = new Pass_through_engine(arena);
    return engine;
}
}

}  // namespace swarmnet_sim
