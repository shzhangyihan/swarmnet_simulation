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
#define SHAPE_SCALE 25
#define TIME_STEP_SECOND 1
#define REPULSIVE_RADIUS 75

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

    void collision() {}

    void message_rx(packet_t packet, situated_sensing_t sensing) {}

    bool message_tx(packet_t* packet) { return false; }

    void message_tx_success() {}

    void init() {
        // cout << "Init robot " << node_id << " at " << pos.x << ", " << pos.y
        // << endl;
        if_in_shape = false;
        rand_seed = node_id;
        if (in_shape(pos.x, pos.y)) {
            if_in_shape = true;
            change_color((color_t){0, 0, 255});
        } else {
            if_in_shape = false;
            change_color((color_t){255, 0, 0});
        }
    }
};

// Make sure to include this line, with the argument the same as the class name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
