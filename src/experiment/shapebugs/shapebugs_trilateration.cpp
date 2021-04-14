#include <math.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <iostream>
#include <utility>
#include <vector>

#include "../../plugin/robot/kilobot.h"

namespace swarmnet_sim {

#define LOCATION_UPDATE_STEP_SIZE 0.01

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    std::vector<int> anchors{0, 1, 2, 3, 10, 11, 12, 13, 20, 21, 22, 23};
    unsigned int rand_seed;
    double x, y;
    bool bootstrapped;
    bool is_anchor;
    // [id, x, y, d]
    std::vector<std::pair<int, std::vector<double>>> bootstrap_candidate;

   public:
    void collision() { turn(rand_r(&rand_seed) % 360 - 180); }

    std::vector<double> cal_circle_intersection(std::vector<double> C1,
                                                std::vector<double> C2) {
        // C1 (x1, y1, r1)
        // C2 (x2, y2, r2)
        // credit to https://math.stackexchange.com/a/1033561

        double x1 = C1[0], y1 = C1[1], r1 = C1[2];
        double x2 = C2[0], y2 = C2[1], r2 = C2[2];
        double d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
        double l = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
        double h = sqrt(r1 * r1 - l * l);
        std::vector<double> res;
        // p1.x
        res.push_back(l / d * (x2 - x1) + h / d * (y2 - y1) + x1);
        // p1.y
        res.push_back(l / d * (y2 - y1) - h / d * (x2 - x1) + y1);
        // p2.x
        res.push_back(l / d * (x2 - x1) - h / d * (y2 - y1) + x1);
        // p2.y
        res.push_back(l / d * (y2 - y1) + h / d * (x2 - x1) + y1);
        return res;
    }

    std::vector<double> cal_line_intersection(std::vector<double> L1,
                                              std::vector<double> L2) {
        // L1 (x1, y1) (x2, y2)
        // L2 (x3, y3) (x4, y4)
        // credit to
        // https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection

        double x1 = L1[0], x2 = L1[2], x3 = L2[0], x4 = L2[2];
        double y1 = L1[1], y2 = L1[3], y3 = L2[1], y4 = L2[3];
        double D = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
        std::vector<double> res;
        // p.x
        res.push_back(((x1 * y2 - y1 * x2) * (x3 - x4) -
                       (x1 - x2) * (x3 * y4 - y3 * x4)) /
                      D);
        // p.y
        res.push_back(((x1 * y2 - y1 * x2) * (y3 - y4) -
                       (y1 - y2) * (x3 * y4 - y3 * x4)) /
                      D);
        return res;
    }

    double cal_point_dist(std::vector<double> P1, std::vector<double> P2) {
        double x1 = P1[0], y1 = P1[1];
        double x2 = P2[0], y2 = P2[1];
        double d = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
        return d;
    }

    void bootstrap() {
        std::vector<double> P = bootstrap_candidate[0].second;
        std::vector<double> Q = bootstrap_candidate[1].second;
        std::vector<double> R = bootstrap_candidate[2].second;
        std::vector<double> PQ = cal_circle_intersection(P, Q);
        std::vector<double> PR = cal_circle_intersection(P, R);
        std::vector<double> L_intersection = cal_line_intersection(PQ, PR);
        std::vector<double> PQ1 =
            std::vector<double>(PQ.begin(), PQ.begin() + 2);
        std::vector<double> PQ2 = std::vector<double>(PQ.begin() + 2, PQ.end());
        double dist1 = cal_point_dist(PQ1, L_intersection);
        double dist2 = cal_point_dist(PQ2, L_intersection);
        // std::cout << PQ1[0] << " " << PQ1[1] << std::endl;
        // std::cout << PQ2[0] << " " << PQ2[1] << std::endl;

        if (dist1 < dist2) {
            // use PQ1 as starting pos
            x = isnan(PQ1[0]) || PQ1[0] < 0 ? 0 : PQ1[0];
            y = isnan(PQ1[1]) || PQ1[1] < 0 ? 0 : PQ1[1];
        } else {
            // use PQ2 as starting pos
            x = isnan(PQ2[0]) || PQ2[0] < 0 ? 0 : PQ2[0];
            y = isnan(PQ2[1]) || PQ2[1] < 0 ? 0 : PQ2[1];
        }
        bootstrapped = true;
        // std::cout << node_id << " start with " << x << ", " << y << " vs true
        // "
        //           << pos.x << ", " << pos.y << std::endl;
    }

    void update_pos(double rx_x, double rx_y, double rx_d) {
        double cal_d = sqrt((rx_x - x) * (rx_x - x) + (rx_y - y) * (rx_y - y));
        double d_d = 1 - cal_d / rx_d;
        double d_x = (x - rx_x) * d_d;
        double d_y = (y - rx_y) * d_d;
        x = x + LOCATION_UPDATE_STEP_SIZE * d_x;
        y = y + LOCATION_UPDATE_STEP_SIZE * d_y;
        x = x < 0 ? 0 : x;
        y = y < 0 ? 0 : y;
        double pos_error =
            sqrt((pos.x - x) * (pos.x - x) + (pos.y - y) * (pos.y - y));
        std::cout << node_id << " " << get_global_time() << " " << pos_error
                  << " true (" << pos.x << ", " << pos.y << ") cur (" << x
                  << ", " << y << ") delta(" << LOCATION_UPDATE_STEP_SIZE * d_x
                  << ", " << LOCATION_UPDATE_STEP_SIZE * d_y << ")"
                  << std::endl;
    }

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        if (is_anchor) return;
        int rx_id = packet.payload[0];
        float rx_x_f, rx_y_f;
        memcpy(&rx_x_f, packet.payload + 1, sizeof(float));
        memcpy(&rx_y_f, packet.payload + 1 + sizeof(float), sizeof(float));
        double rx_x = rx_x_f;
        double rx_y = rx_y_f;
        double rx_d = sensing.distance;
        // std::cout << node_id << " rx " << rx_id << " " << rx_x << " " << rx_y
        //           << " " << rx_d << std::endl;
        if (bootstrapped) {
            update_pos(rx_x, rx_y, rx_d);
        } else {
            for (auto c : bootstrap_candidate) {
                if (c.first == rx_id) return;
            }
            bootstrap_candidate.push_back(
                make_pair(rx_id, std::vector<double>{rx_x, rx_y, rx_d}));
            if (bootstrap_candidate.size() == 3) bootstrap();
        }
        if (bootstrapped) {
            change_color((color_t){0, 0, 255});
        } else {
            change_color((color_t){255, 0, 0});
        }
    }

    bool message_tx(packet_t* packet) {
        if (bootstrapped) {
            packet->payload[0] = node_id;
            float my_x = x, my_y = y;
            memcpy(packet->payload + 1, &my_x, sizeof(float));
            memcpy(packet->payload + 1 + sizeof(float), &my_y, sizeof(float));
            return true;
        } else {
            return false;
        }
    }

    void message_tx_success() {}

    void init() {
        std::cout << "Init robot " << node_id << " at " << pos.x << ", "
                  << pos.y << std::endl;
        rand_seed = node_id;
        turn(rand_r(&rand_seed) % 360 - 180);
        if (find(anchors.begin(), anchors.end(), node_id) == anchors.end()) {
            // not anchors
            // go_forward();
            x = 0;
            y = 0;
            bootstrapped = false;
        } else {
            // anchors
            // go_forward();
            is_anchor = true;
            x = pos.x;
            y = pos.y;
            bootstrapped = true;
        }
        if (bootstrapped) {
            change_color((color_t){0, 0, 255});
        } else {
            change_color((color_t){255, 0, 0});
        }
    }
};

// Make sure to include this line, with the argument the same as the class name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
