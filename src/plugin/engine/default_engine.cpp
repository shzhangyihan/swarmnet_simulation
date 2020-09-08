#include "default_engine.h"

#include <math.h>

#include <algorithm>
#include <iostream>
#include <vector>

#include "../event/collision_event.h"

namespace swarmnet_sim {

extern "C" {
float check_collision(Arena *arena, float future_time) {
    Sim_config config = arena->get_config();
    int num_robots = config.get_num_robots();
    position2d_t future_pos[num_robots];
    std::vector<int> collided_node;
    bool violated = false;
    int future_ticks = TICK_PER_SECOND * future_time;
    for (int t = 1; t <= future_ticks; t++) {
        if (t > 1) {
            // no collision up front, update the flags
            for (int i = 0; i < num_robots; i++) {
                if (arena->get_node(i)->get_collision_flag()) {
                    arena->log_node(i);
                }
                arena->get_node(i)->set_collision_flag(false);
                arena->get_node(i)->set_skip_logging_flag(false);
            }
        }

        for (int i = 0; i < num_robots; i++) {
            // future_pos[i] = calculate_future_pos(
            //     arena->get_node(i)->get_position(),
            //     arena->get_node(i)->get_velocity(),
            //     (float)t / arena->get_config().get_ticks_per_second());
            // if (t == 0)
            //     future_pos[i] = arena->get_node(i)->get_position();
            // else
            future_pos[i] = calculate_future_pos(
                arena->get_node(i)->get_position(),
                arena->get_node(i)->get_velocity(), (float)1 / TICK_PER_SECOND);
        }

        for (int i = 0; i < num_robots; i++) {
            if (check_out_of_bound(
                    future_pos[i], arena->get_node(i)->get_radius(),
                    config.get_arena_max_x(), config.get_arena_max_y())) {
                // if (t == 0) {
                //     // for nodes already out of bound at start, don't update
                //     // their pos
                //     // std::cout << "col at start" << std::endl <<
                //     std::flush; collided_node.push_back(i);
                // if (t != 0) {
                // need to add collison event
                violated = true;
                Collision_event *new_event = new Collision_event(
                    arena, arena->get_sim_time() + (float)t / TICK_PER_SECOND,
                    -1, i);
                arena->add_event(new_event);
                // std::cout << "pos of ofb " << future_pos[i].x << ", "
                //           << future_pos[i].y << std::endl
                //           << std::flush;
                // }
                collided_node.push_back(i);
            }
        }

        for (int i = 0; i < num_robots; i++) {
            for (int j = i + 1; j < num_robots; j++) {
                if (check_robot_collision(future_pos[i], future_pos[j],
                                          arena->get_node(i)->get_radius())) {
                    // if (t == 0) {
                    //     // for nodes already out of bound at start, don't
                    //     update
                    //     // their pos
                    //     collided_node.push_back(i);
                    //     collided_node.push_back(j);
                    // if (t != 0) {
                    // need to add collison event
                    violated = true;
                    Collision_event *new_event = new Collision_event(
                        arena,
                        arena->get_sim_time() + (float)t / TICK_PER_SECOND, i,
                        j);
                    arena->add_event(new_event);
                    new_event = new Collision_event(
                        arena,
                        arena->get_sim_time() + (float)t / TICK_PER_SECOND, j,
                        i);
                    arena->add_event(new_event);
                    // }
                    collided_node.push_back(i);
                    collided_node.push_back(j);
                }
            }
        }

        // if (t != 0) {
        // need to update location
        for (int i = 0; i < num_robots; i++) {
            if (std::find(collided_node.begin(), collided_node.end(), i) ==
                collided_node.end()) {
                // node not collided, update
                arena->move_robot(i, future_pos[i]);
            }
        }
        // }

        if (violated) {
            // std::cout << "violated at " << t << std::endl << std::flush;
            return (float)t / TICK_PER_SECOND;
        }
    }

    return -1;
}
}

bool check_robot_collision(position2d_t pos_1, position2d_t pos_2, int radius) {
    float dist = calculate_dist(pos_1, pos_2);
    if (dist > radius * 2) {
        return false;
    } else {
        return true;
    }
}

bool check_out_of_bound(position2d_t pos, int radius, int x_max, int y_max) {
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

}  // namespace swarmnet_sim
