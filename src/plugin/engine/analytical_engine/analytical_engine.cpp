#include "analytical_engine.h"

#include <float.h>
#include <math.h>

#include <algorithm>
#include <iostream>
#include <set>
#include <vector>

#include "../../event/collision_event.h"

// 1 / PRECISION second precision
// #define PRECISION 1000
#define EPSILON 0.001

namespace swarmnet_sim {

// assume a >= b
bool float_are_same(float a, float b) { return a - b < EPSILON; }

extern "C" {
float check_collision(Arena *arena, float future_time) {
    if (float_are_same(future_time, 0)) return -1;

    Sim_config config = arena->get_config();
    int max_x = config.get_arena_max_x();
    int max_y = config.get_arena_max_y();
    int num_robots = config.get_num_robots();
    std::set<int> collided_node;
    float min_collision_time = future_time;
    std::vector<float> out_of_bound_time(num_robots);
    std::vector<std::vector<float>> collide_time(
        num_robots, std::vector<float>(num_robots));

    for (int i = 0; i < num_robots; i++) {
        Node *node = arena->get_node(i);
        float t =
            time_to_out_of_bound(node->get_position(), node->get_velocity(),
                                 node->get_radius(), max_x, max_y);
        out_of_bound_time[i] = t;
        if (t != -1 && t <= future_time) {
            min_collision_time = std::min(min_collision_time, t);
        }
        // std::cout << "old t " << t << " new t ";
        // t = (float)((int)(t * PRECISION)) / PRECISION;
        // std::cout << t << std::endl;
        // if (t != -1 && t <= future_time) {
        //     // std::cout << arena->get_sim_time() << " | " << i
        //     //           << " out of bound in " << t << std::endl;
        //     if (float_are_same(t, min_collision_time)) {
        //         min_collision_time = std::min(min_collision_time, t);
        //         collided_node.push_back(i);
        //     } else if (t < min_collision_time) {
        //         min_collision_time = t;
        //         collided_node.clear();
        //         collided_node.push_back(i);
        //     }
        // }
    }

    for (int i = 0; i < num_robots; i++) {
        Node *node_1 = arena->get_node(i);
        for (int j = i + 1; j < num_robots; j++) {
            Node *node_2 = arena->get_node(j);
            float t =
                time_to_collide(node_1->get_position(), node_1->get_velocity(),
                                node_2->get_position(), node_2->get_velocity(),
                                node_1->get_radius());
            collide_time[i][j] = t;
            if (t != -1 && t <= future_time) {
                min_collision_time = std::min(min_collision_time, t);
            }
            // std::cout << "old t " << t << " new t ";
            // t = (float)((int)(t * PRECISION)) / PRECISION;
            // // std::cout << t << std::endl;

            // if (t != -1 && t <= future_time) {
            //     // std::cout << arena->get_sim_time() << " | " << i
            //     //           << " colliding with " << j << " in " << t
            //     //           << std::endl;
            //     if (float_are_same(t, min_collision_time)) {
            //         min_collision_time = std::min(min_collision_time, t);
            //         collided_node.push_back(i);
            //         collided_node.push_back(j);
            //     } else if (t < min_collision_time) {
            //         min_collision_time = t;
            //         collided_node.clear();
            //         collided_node.push_back(i);
            //         collided_node.push_back(j);
            //     }
            // }
        }
    }
    // std::cout << "? " << collided_node.size() << " " << min_collision_time
    //           << std::endl;

    for (int i = 0; i < num_robots; i++) {
        // std::cout << out_of_bound_time[i] << " ";
        if (out_of_bound_time[i] == -1) continue;
        if (float_are_same(out_of_bound_time[i], min_collision_time)) {
            collided_node.insert(i);
        }
    }
    // std::cout << std::endl;
    for (int i = 0; i < num_robots; i++) {
        for (int j = i + 1; j < num_robots; j++) {
            // std::cout << collide_time[i][j] << " ";
            if (collide_time[i][j] == -1) continue;
            if (float_are_same(collide_time[i][j], min_collision_time)) {
                collided_node.insert(i);
                collided_node.insert(j);
            }
        }
        // std::cout << std::endl;
    }
    // if (min_collision_time == 0) {
    // std::cout << arena->get_sim_time() << " | " << min_collision_time
    //           << std::endl;
    //     // exit(-1);
    // }

    for (int i = 0; i < num_robots; i++) {
        arena->move_robot(
            i, calculate_future_pos(arena->get_node(i)->get_position(),
                                    arena->get_node(i)->get_velocity(),
                                    min_collision_time));
    }

    for (int i : collided_node) {
        Collision_event *new_event = new Collision_event(
            arena, arena->get_sim_time() + min_collision_time, -1, i);
        arena->add_event(new_event);
    }

    // std::cout << "# " << collided_node.size() << std::endl;

    if (collided_node.size() == 0) {
        return -1;
    } else {
        return min_collision_time;
    }
}
}

float time_to_out_of_bound(position2d_t pos, float v, int radius, int x_max,
                           int y_max) {
    float v_x = cos(((double)pos.theta) / 180 * M_PI) * v;
    float v_y = sin(((double)pos.theta) / 180 * M_PI) * v;
    // std::cout << v_x << " " << v_y << std::endl;
    float min_time = FLT_MAX;

    if (v_x > 0) {
        float time = (x_max - radius - pos.x) / v_x;
        min_time = std::min(min_time, time);
    } else if (v_x < 0) {
        float time = (pos.x - radius) / v_x * -1;
        min_time = std::min(min_time, time);
    }

    if (v_y > 0) {
        float time = (y_max - radius - pos.y) / v_y;
        min_time = std::min(min_time, time);
    } else if (v_y < 0) {
        float time = (pos.y - radius) / v_y * -1;
        min_time = std::min(min_time, time);
    }

    if (min_time < FLT_MAX && min_time >= 0) {
        return min_time;
    } else {
        return -1;
    }
}

bool check_robot_collision(position2d_t pos_1, position2d_t pos_2, int radius) {
    float dist = calculate_dist(pos_1, pos_2);
    std::cout << dist << std::endl;
    if (dist >= radius * 2) {
        return false;
    } else {
        std::cout << dist << " " << radius * 2 << std::endl;
        // exit(-1);
        return true;
    }
}

float time_to_collide(position2d_t pos_1, float v_1, position2d_t pos_2,
                      float v_2, int radius) {
    // if (check_robot_collision(pos_1, pos_2, radius)) {
    //     std::cout << "already collided" << std::endl;
    // }
    // bool check = check_robot_collision(pos_1, pos_2, radius);

    float v_1_x = cos(((double)pos_1.theta) / 180 * M_PI) * v_1;
    float v_1_y = sin(((double)pos_1.theta) / 180 * M_PI) * v_1;
    float v_2_x = cos(((double)pos_2.theta) / 180 * M_PI) * v_2;
    float v_2_y = sin(((double)pos_2.theta) / 180 * M_PI) * v_2;
    float delta_v_x = v_1_x - v_2_x;
    float delta_v_y = v_1_y - v_2_y;
    float delta_x = pos_1.x - pos_2.x;
    // Vieta's formula for the quadratic inequation
    float delta_y = pos_1.y - pos_2.y;
    // in the form of a*t^2 + b*t + c <= 0
    float a = delta_v_x * delta_v_x + delta_v_y * delta_v_y;
    float b = 2 * delta_x * delta_v_x + 2 * delta_y * delta_v_y;
    float c = delta_x * delta_x + delta_y * delta_y - 4 * radius * radius;

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

        float t = -c / b;
        if (t < 0) {
            return -1;
        } else {
            return t;
        }
    }

    float function_delta = b * b - 4 * a * c;
    if (function_delta < 0) {
        // no solution to this quadratic inequation
        return -1;
    }
    float sqrt_delta = sqrt(function_delta);
    // std::cout << "sqrt!!!! " << sqrt_delta << " " << function_delta
    //           << std::endl;
    float t_1 = (-b - sqrt_delta) / (2 * a);
    float t_2 = (-b + sqrt_delta) / (2 * a);
    // make sure t_1 <= t_2
    if (t_1 > t_2) {
        float tmp = t_1;
        t_1 = t_2;
        t_2 = tmp;
    }
    // std::cout << t_1 << " " << t_2 << std::endl;
    if (float_are_same(fabs(t_1), 0)) {
        // std::cout << "ggwp " << t_1 << " " << t_2 << std::endl;
        return 0;
    }
    if (float_are_same(fabs(t_2), 0)) {
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
    // std::cout << t_1 << " " << t_2 << " a " << a << " b " << b << " c " << c
    //           << std::endl;
    // HACK decrease precision
    // t_1 = (float)((int)(t_1 * 10000)) / 10000;
    // t_2 = (float)((int)(t_2 * 10000)) / 10000;

    // if (check) {
    //     std::cout << pos_1.x << ", " << pos_1.y << " & " << pos_2.x << ", "
    //               << pos_2.y << std::endl;
    //     std::cout << v_1_x << ", " << v_1_y << " & " << v_2_x << ", " <<
    //     v_2_y
    //               << std::endl;
    //     std::cout << t_1 << " " << t_2 << std::endl;
    //     exit(-1);
    // }
    // if (t_1 < 0) {
    //     if (t_2 < 0) {
    //         // no positive solution
    //         return -1;
    //     }
    //     // std::cout << "hmmm " << calculate_dist(pos_1, pos_2) << " " << t_1
    //     //           << " " << t_2 << std::endl;
    //     // exit(-1);
    //     return t_2;
    //     // return 0;
    // } else {
    //     if (t_2 < 0) {
    //         // std::cout << "hmmm " << calculate_dist(pos_1, pos_2) << " " <<
    //         // t_1
    //         //           << " " << t_2 << std::endl;
    //         // exit(-1);
    //         return t_1;
    //         // return 0;
    //     }
    //     return std::min(t_1, t_2);
    // }
}

}  // namespace swarmnet_sim
