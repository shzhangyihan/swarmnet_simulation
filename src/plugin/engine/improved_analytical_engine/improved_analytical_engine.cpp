#include "improved_analytical_engine.h"

#include <float.h>
#include <math.h>

#include <algorithm>
#include <iostream>
#include <set>
#include <vector>

#include "../../event/collision_event.h"

// 1 / PRECISION second precision
#define PRECISION 10000000
#define EPSILON 0.00001

namespace swarmnet_sim {

// assume a >= b
bool double_are_same(double a, double b) { return a - b < EPSILON; }

double Analytical_engine::check_collision(double future_time) {
    Arena *arena_ptr = (Arena *)this->arena;

    if (double_are_same(future_time, 0)) return -1;

    Sim_config config = arena_ptr->get_config();
    int max_x = config.get_arena_max_x();
    int max_y = config.get_arena_max_y();
    int num_robots = config.get_num_robots();

    // std::cout << "engine start" << std::endl;
    for (int i = 0; i < num_robots; i++) {
        Node *node = arena_ptr->get_node(i);
        std::pair<double, double> state =
            std::make_pair(node->get_velocity(), node->get_position().theta);
        need_update[i] = need_update[i] | (state != motion_state[i]);
        motion_state[i] = state;
    }

    double cur_time = arena_ptr->get_sim_time();
    long long min_collision_time = future_time * PRECISION;
    for (int i = 0; i < num_robots; i++) {
        if (need_update[i]) {
            Node *node = arena_ptr->get_node(i);
            double t =
                time_to_out_of_bound(node->get_position(), node->get_velocity(),
                                     node->get_radius(), max_x, max_y);
            // std::cout << cur_time << " | " << t << std::endl;
            if (t == -1) {
                out_of_bound_time[i] = -1;
                exit(-1);
            } else {
                out_of_bound_time[i] = (long long)(cur_time * PRECISION) +
                                       (long long)(t * PRECISION);
            }
        }
        if (out_of_bound_time[i] != -1) {
            long long t = out_of_bound_time[i] - cur_time * PRECISION;
            if (t >= 0 && t <= future_time * PRECISION) {
                min_collision_time = std::min(min_collision_time, t);
            }
        }
    }
    // exit(-1);
    // std::cout << "collision" << std::endl;

    for (int i = 0; i < num_robots; i++) {
        Node *node_1 = arena_ptr->get_node(i);
        for (int j = i + 1; j < num_robots; j++) {
            if (need_update[i] || need_update[j]) {
                Node *node_2 = arena_ptr->get_node(j);
                double t = time_to_collide(
                    node_1->get_position(), node_1->get_velocity(),
                    node_2->get_position(), node_2->get_velocity(),
                    node_1->get_radius());
                if (t == -1) {
                    collision_time[i][j] = -1;
                } else {
                    collision_time[i][j] = (long long)(cur_time * PRECISION) +
                                           (long long)(t * PRECISION);
                }
            }
            if (collision_time[i][j] != -1) {
                long long t = collision_time[i][j] - cur_time * PRECISION;
                if (t >= 0 && t <= future_time * PRECISION) {
                    min_collision_time = std::min(min_collision_time, t);
                }
            }
        }
    }

    std::set<int> collided_node;

    for (int i = 0; i < num_robots; i++) {
        if (out_of_bound_time[i] == -1) continue;
        if (out_of_bound_time[i] - cur_time * PRECISION <=
            min_collision_time + PRECISION * EPSILON) {
            collided_node.insert(i);
        }
    }
    for (int i = 0; i < num_robots; i++) {
        for (int j = i + 1; j < num_robots; j++) {
            if (collision_time[i][j] == -1) continue;
            if (collision_time[i][j] - cur_time * PRECISION <=
                min_collision_time + PRECISION * EPSILON) {
                collided_node.insert(i);
                collided_node.insert(j);
            }
        }
    }

    for (int i = 0; i < num_robots; i++) {
        arena_ptr->move_robot(
            i, calculate_future_pos(arena_ptr->get_node(i)->get_position(),
                                    arena_ptr->get_node(i)->get_velocity(),
                                    (double)min_collision_time / PRECISION));
    }

    for (int i : collided_node) {
        Collision_event *new_event = new Collision_event(
            arena_ptr, cur_time + (double)min_collision_time / PRECISION, -1,
            i);
        arena_ptr->add_event(new_event);
    }

    need_update = std::vector<bool>(num_robots, false);
    if (collided_node.size() == 0) {
        return -1;
    } else {
        return (double)min_collision_time / PRECISION;
    }
}

double Analytical_engine::time_to_out_of_bound(position2d_t pos, double v,
                                               int radius, int x_max,
                                               int y_max) {
    double v_x = cos(((double)pos.theta) / 180 * M_PI) * v;
    double v_y = sin(((double)pos.theta) / 180 * M_PI) * v;
    double min_time = DBL_MAX;

    if (v_x > 0) {
        double time = (x_max - radius - pos.x) / v_x;
        min_time = std::min(min_time, time);
    } else if (v_x < 0) {
        double time = (pos.x - radius) / v_x * -1;
        min_time = std::min(min_time, time);
    }

    if (v_y > 0) {
        double time = (y_max - radius - pos.y) / v_y;
        min_time = std::min(min_time, time);
    } else if (v_y < 0) {
        double time = (pos.y - radius) / v_y * -1;
        min_time = std::min(min_time, time);
    }

    if (double_are_same(fabs(min_time), 0)) {
        return 0;
    }

    if (min_time < DBL_MAX && min_time >= 0) {
        return min_time;
    } else {
        return -1;
    }
}

bool Analytical_engine::check_robot_collision(position2d_t pos_1,
                                              position2d_t pos_2, int radius) {
    double dist = calculate_dist(pos_1, pos_2);
    std::cout << dist << std::endl;
    if (dist >= radius * 2) {
        return false;
    } else {
        std::cout << dist << " " << radius * 2 << std::endl;
        // exit(-1);
        return true;
    }
}

double Analytical_engine::time_to_collide(position2d_t pos_1, double v_1,
                                          position2d_t pos_2, double v_2,
                                          int radius) {
    double v_1_x = cos(((double)pos_1.theta) / 180 * M_PI) * v_1;
    double v_1_y = sin(((double)pos_1.theta) / 180 * M_PI) * v_1;
    double v_2_x = cos(((double)pos_2.theta) / 180 * M_PI) * v_2;
    double v_2_y = sin(((double)pos_2.theta) / 180 * M_PI) * v_2;
    double delta_v_x = v_1_x - v_2_x;
    double delta_v_y = v_1_y - v_2_y;
    double delta_x = pos_1.x - pos_2.x;
    // Vieta's formula for the quadratic inequality
    double delta_y = pos_1.y - pos_2.y;
    // in the form of a*t^2 + b*t + c <= 0
    double a = delta_v_x * delta_v_x + delta_v_y * delta_v_y;
    double b = 2 * delta_x * delta_v_x + 2 * delta_y * delta_v_y;
    double c = delta_x * delta_x + delta_y * delta_y - 4 * radius * radius;

    if (a == 0) {
        // not quadratic function
        if (b == 0) {
            if (c <= 0) {
                // collided
                // std::cout << "gg" << std::endl;
                return 0;
            } else {
                // don't worry
                return -1;
            }
        }

        double t = -c / b;
        if (t < 0) {
            return -1;
        } else {
            return t;
        }
    }

    double function_delta = b * b - 4 * a * c;
    if (function_delta < 0) {
        // no solution to this quadratic inequation
        return -1;
    }
    double sqrt_delta = sqrt(function_delta);
    // std::cout << "sqrt!!!! " << sqrt_delta << " " << function_delta
    //           << std::endl;
    double t_1 = (-b - sqrt_delta) / (2 * a);
    double t_2 = (-b + sqrt_delta) / (2 * a);
    // make sure t_1 <= t_2
    if (t_1 > t_2) {
        double tmp = t_1;
        t_1 = t_2;
        t_2 = tmp;
    }
    // std::cout << t_1 << " " << t_2 << std::endl;
    if (double_are_same(fabs(t_1), 0)) {
        // std::cout << "ggwp " << t_1 << " " << t_2 << std::endl;
        return 0;
    }
    if (double_are_same(fabs(t_2), 0)) {
        // std::cout << "wpgg " << t_1 << " " << t_2 << std::endl;
        return -1;
    }
    if (t_2 < 0) {
        return -1;
    } else if (t_1 < 0) {
        // collided already
        // std::cout << "wp " << t_1 << " " << t_2 << std::endl;
        return 0;
    } else {
        return t_1;
    }
}

void Analytical_engine::init() {
    Arena *arena_ptr = (Arena *)this->arena;
    Sim_config config = arena_ptr->get_config();
    int num_robots = config.get_num_robots();
    need_update = std::vector<bool>(num_robots, true);
    motion_state = std::vector<std::pair<double, double>>(num_robots);
    out_of_bound_time = std::vector<long long>(num_robots, 0);
    collision_time = std::vector<std::vector<long long>>(
        num_robots, std::vector<long long>(num_robots, 0));
}

Analytical_engine::Analytical_engine(void *arena) { this->arena = arena; }

extern "C" {
Physics_engine *engine_builder(void *arena) {
    Physics_engine *engine = new Analytical_engine(arena);
    return engine;
}
}
}  // namespace swarmnet_sim
