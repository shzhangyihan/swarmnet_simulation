#include "default_engine.h"

#include <math.h>

#include <iostream>

#include "../event/collision_event.h"

namespace swarmnet_sim {

extern "C" {
bool check_collision(Arena *arena, int future_ticks) {
    Sim_config config = arena->get_config();
    int num_robots = config.get_num_robots();
    position2d_t future_pos[num_robots];
    bool violated = false;
    for (int t = 1; t <= future_ticks; t++) {
        for (int i = 0; i < num_robots; i++) {
            future_pos[i] =
                calculate_future_pos(arena->get_node(i)->get_position(),
                                     arena->get_node(i)->get_velocity(), t);
        }

        for (int i = 0; i < num_robots; i++) {
            if (check_out_of_bound(
                    future_pos[i], arena->get_node(i)->get_radius(),
                    config.get_arena_max_x(), config.get_arena_max_y())) {
                violated = true;
                Collision_event *new_event = new Collision_event(
                    arena, arena->get_current_tick() + t - 1, -1, i);
                arena->add_event(new_event);
            }
        }

        for (int i = 0; i < num_robots; i++) {
            for (int j = i + 1; j < num_robots; j++) {
                if (check_robot_collision(future_pos[i], future_pos[j],
                                          arena->get_node(i)->get_radius())) {
                    violated = true;
                    Collision_event *new_event = new Collision_event(
                        arena, arena->get_current_tick() + t - 1, i, j);
                    arena->add_event(new_event);
                    new_event = new Collision_event(
                        arena, arena->get_current_tick() + t - 1, j, i);
                    arena->add_event(new_event);
                }
            }
        }

        if (violated) return true;
    }

    return false;
}
}

bool check_robot_collision(position2d_t pos_1, position2d_t pos_2, int radius) {
    float x_1, y_1;
    float x_2, y_2;

    // assume x_1 always smaller than x_2
    if (pos_1.x < pos_2.x) {
        x_1 = pos_1.x;
        x_2 = pos_2.x;
    } else {
        x_1 = pos_2.x;
        x_2 = pos_1.x;
    }

    // assume y_1 always smaller than y_2
    if (pos_1.y < pos_2.y) {
        y_1 = pos_1.y;
        y_2 = pos_2.y;
    } else {
        y_1 = pos_2.y;
        y_2 = pos_1.y;
    }

    if (x_1 + radius > x_2 - radius) {
        // x overlaps
        if (y_1 + radius > y_2 - radius) {
            // y overlaps as well
            return true;
        }
    }

    return false;
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
