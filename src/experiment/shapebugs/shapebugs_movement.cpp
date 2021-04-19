#include <math.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <tuple>
#include <unordered_map>
#include <utility>
#include <vector>

#include "../../plugin/robot/kilobot.h"

namespace swarmnet_sim {

#define LOCATION_UPDATE_STEP_SIZE 0.01
#define SHAPE_SCALE 50
#define TIME_STEP_SECOND 2
#define REPULSIVE_RADIUS 75
#define FORWARD_OOS_CHECK 5
#define PI 3.14159265

using namespace std;

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    vector<int> anchors{0, 1, 2, 3, 10, 11, 12, 13, 20, 21, 22, 23};
    unordered_map<int, tuple<double, double, double>> rx_per_step;
    unsigned int rand_seed;
    bool if_in_shape;
    double prev_step_time;
    const bool shape[20][20] = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    };

   public:
    bool in_shape(double x, double y) {
        int x_index = x / SHAPE_SCALE;
        int y_index = y / SHAPE_SCALE;
        if (x_index < 0 || x_index >= 20 || y_index < 0 || y_index >= 20)
            return false;
        return shape[x_index][y_index];
    }

    void collision() { turn(rand_r(&rand_seed) % 360 - 180); }

    void loop() {
        double cur_theta = this->pos.theta;
        bool need_to_turn = false;
        if (get_local_time() - prev_step_time >= TIME_STEP_SECOND) {
            if (in_shape(pos.x, pos.y)) {
                if_in_shape = true;
                change_color((color_t){0, 0, 255});
                double cur_x = pos.x;
                double cur_y = pos.y;
                double mov_x = 0;
                double mov_y = 0;
                bool rx_update = false;
                for (auto rx : rx_per_step) {
                    double rx_x = get<0>(rx.second);
                    double rx_y = get<1>(rx.second);
                    double rx_d = get<2>(rx.second);
                    if (!in_shape(rx_x, rx_y)) continue;
                    if (rx_d >= REPULSIVE_RADIUS) continue;
                    double step_x =
                        (rx_x - cur_x) * (REPULSIVE_RADIUS - rx_d) / rx_d;
                    double step_y =
                        (rx_y - cur_y) * (REPULSIVE_RADIUS - rx_d) / rx_d;
                    mov_x += step_x;
                    mov_y += step_y;
                    rx_update = true;
                }
                if (rx_update) {
                    double mov_d = sqrt(mov_x * mov_x + mov_y * mov_y);
                    mov_x = mov_x / mov_d * -1;
                    mov_y = mov_y / mov_d * -1;
                    double target_theta = atan(mov_y / mov_x) * 180 / PI;
                    if (mov_x < 0) target_theta += 180;
                    cur_theta = target_theta;
                    need_to_turn = true;
                }
            }
            rx_per_step = unordered_map<int, tuple<double, double, double>>();
            prev_step_time = get_local_time();
        }
        if (in_shape(pos.x, pos.y)) {
            if_in_shape = true;
            change_color((color_t){0, 0, 255});
            while (true) {
                double mov_y = sin(cur_theta * PI / 180.0);
                double mov_x = cos(cur_theta * PI / 180.0);
                double new_x = pos.x + mov_x * LOOP_PERIOD_SECOND *
                                           VELOCITY_PER_SECOND *
                                           FORWARD_OOS_CHECK;
                double new_y = pos.y + mov_y * LOOP_PERIOD_SECOND *
                                           VELOCITY_PER_SECOND *
                                           FORWARD_OOS_CHECK;
                if (in_shape(new_x, new_y)) {
                    break;
                } else {
                    need_to_turn = true;
                    cur_theta = rand_r(&rand_seed) % 360 - 180;
                    continue;
                }
            }
        } else {
            if_in_shape = false;
            change_color((color_t){255, 0, 0});
        }
        go_forward();
        if (need_to_turn) {
            turn(cur_theta - pos.theta);
        }
    }

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        int rx_id = packet.payload[0];
        float rx_x_f, rx_y_f;
        memcpy(&rx_x_f, packet.payload + 1, sizeof(float));
        memcpy(&rx_y_f, packet.payload + 1 + sizeof(float), sizeof(float));
        double rx_x = rx_x_f;
        double rx_y = rx_y_f;
        double rx_d = sensing.distance;
        rx_per_step[rx_id] = make_tuple(rx_x, rx_y, rx_d);
    }

    bool message_tx(packet_t* packet) {
        packet->payload[0] = node_id;
        float my_x = pos.x, my_y = pos.y;
        memcpy(packet->payload + 1, &my_x, sizeof(float));
        memcpy(packet->payload + 1 + sizeof(float), &my_y, sizeof(float));

        return true;
    }

    void message_tx_success() {}

    void init() {
        if_in_shape = false;
        rand_seed = node_id;
        turn(rand_r(&rand_seed) % 360 - 180);
        go_forward();
        change_color((color_t){255, 0, 0});
        prev_step_time = get_local_time();
        rx_per_step = unordered_map<int, tuple<double, double, double>>();
    }
};

// Make sure to include this line, with the argument the same as the class
// name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
