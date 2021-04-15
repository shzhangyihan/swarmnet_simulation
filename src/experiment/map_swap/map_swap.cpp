#include <cstring>
#include <iostream>

#include "../../plugin/robot/kilobot.h"

namespace swarmnet_sim {

using namespace std;

#define HANDSHAKE_TIMEOUT 4
#define MAX_CONFIRM_COUNT 4

typedef struct {
    int x;
    int y;
} map_state_t;

enum handshake_state_t { advertisement, reply, confirm };

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    map_state_t local_state, tmp_state;
    handshake_state_t handshake_state;
    double handshake_timer;
    int exchange_target;
    int confirm_count;

   public:
    void collision() {}

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        int rx_id = packet.payload[0];
        int target_id = packet.payload[1];
        handshake_state_t rx_state = (handshake_state_t)packet.payload[2];
        int rx_map_x = packet.payload[3];
        int rx_map_y = packet.payload[4];
        float rx_x, rx_y;
        memcpy(&rx_x, packet.payload + 5, sizeof(float));
        memcpy(&rx_y, packet.payload + 5 + sizeof(float), sizeof(float));
        // std::cout << node_id << " " << handshake_state << " " << rx_id << " "
        //           << handshake_state << std::endl;
        if (handshake_state == advertisement && rx_state == advertisement) {
            std::cout << node_id << " ok" << std::endl;
        } else if (handshake_state == advertisement && rx_state == reply) {
            std::cout << node_id << " reply" << std::endl;
        } else if (handshake_state == reply && rx_state == reply) {
            std::cout << node_id << " gg" << std::endl;
        } else if (handshake_state == reply && rx_state == confirm) {
            std::cout << node_id << " omfg" << std::endl;
        } else {
            std::cout << node_id << " what? " << handshake_state << " "
                      << rx_state << std::endl;
        }

        if (handshake_state == reply &&
            get_local_time() - handshake_timer > HANDSHAKE_TIMEOUT) {
            handshake_state = advertisement;
        }

        switch (handshake_state) {
            case advertisement:
                if (rx_state == advertisement) {
                    // std::cout << pos.x << ", " << pos.y << " | " << rx_x <<
                    // ", "
                    //           << rx_y << " | " << local_state.x << ", "
                    //           << local_state.y << " | " << rx_map_x << ", "
                    //           << rx_map_y << std::endl;
                    if ((pos.x > rx_x && local_state.x < rx_map_x) ||
                        (pos.x < rx_x && local_state.x > rx_map_x) ||
                        (pos.y > rx_y && local_state.y < rx_map_y) ||
                        (pos.y < rx_y && local_state.y > rx_map_y)) {
                        // start an exchange
                        std::cout << "exchange" << std::endl;
                        handshake_state = reply;
                        exchange_target = rx_id;
                        tmp_state.x = rx_map_x;
                        tmp_state.y = rx_map_y;
                        handshake_timer = get_local_time();
                    }
                }
                if (rx_state == reply && target_id == node_id) {
                    handshake_state = confirm;
                    exchange_target = rx_id;
                    tmp_state.x = rx_map_x;
                    tmp_state.y = rx_map_y;
                    confirm_count = 0;
                    std::cout << "recv reply " << handshake_state << std::endl;
                }
                break;
            case reply:
                if (rx_state == confirm && target_id == node_id) {
                    std::cout << "recv confirm" << std::endl;
                    // confirmed from an exchange
                    local_state.x = tmp_state.x;
                    local_state.y = tmp_state.y;
                    handshake_state = advertisement;
                }
                break;
        }
        std::string log = "(" + std::to_string(local_state.x) + "," +
                          std::to_string(local_state.y) + ")";
        update_log(log);
    }

    bool message_tx(packet_t* packet) {
        // std::cout << node_id << std::endl;
        if (handshake_state == reply &&
            get_local_time() - handshake_timer > HANDSHAKE_TIMEOUT) {
            handshake_state = advertisement;
        }
        if (handshake_state == confirm && confirm_count >= MAX_CONFIRM_COUNT) {
            std::cout << node_id << "confirm finished" << std::endl;
            handshake_state = advertisement;
            local_state.x = tmp_state.x;
            local_state.y = tmp_state.y;
        }
        float my_x = pos.x, my_y = pos.y;
        std::cout << node_id << " tx " << handshake_state << std::endl;
        switch (handshake_state) {
            case advertisement:
                packet->payload[0] = node_id;
                packet->payload[1] = 0;
                packet->payload[2] = handshake_state;
                packet->payload[3] = local_state.x;
                packet->payload[4] = local_state.y;

                memcpy(packet->payload + 5, &my_x, sizeof(float));
                memcpy(packet->payload + 5 + sizeof(float), &my_y,
                       sizeof(float));
                break;
            case reply:
                packet->payload[0] = node_id;
                packet->payload[1] = exchange_target;
                packet->payload[2] = handshake_state;
                packet->payload[3] = local_state.x;
                packet->payload[4] = local_state.y;

                memcpy(packet->payload + 5, &my_x, sizeof(float));
                memcpy(packet->payload + 5 + sizeof(float), &my_y,
                       sizeof(float));
                break;
            case confirm:
                packet->payload[0] = node_id;
                packet->payload[1] = exchange_target;
                packet->payload[2] = handshake_state;
                packet->payload[3] = local_state.x;
                packet->payload[4] = local_state.y;

                memcpy(packet->payload + 5, &my_x, sizeof(float));
                memcpy(packet->payload + 5 + sizeof(float), &my_y,
                       sizeof(float));
                std::cout << node_id << " confirm count " << confirm_count
                          << std::endl;
                confirm_count++;
                break;
        }

        std::string log = "(" + std::to_string(local_state.x) + "," +
                          std::to_string(local_state.y) + ")";
        update_log(log);
        return true;
    }

    void message_tx_success() {}

    void init() {
        handshake_state = advertisement;
        switch (node_id) {
            case 0:
                local_state = {1, 1};
                change_color({0, 0, 255});
                break;
            case 1:
                local_state = {1, 0};
                change_color({0, 255, 0});
                break;
            case 2:
                local_state = {0, 1};
                change_color({255, 0, 0});
                break;
            case 3:
                local_state = {0, 0};
                change_color({255, 255, 255});
                break;
        }
        std::string log = "(" + std::to_string(local_state.x) + "," +
                          std::to_string(local_state.y) + ")";
        update_log(log);
    }
};

// Make sure to include this line, with the argument the same as the class
// name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
