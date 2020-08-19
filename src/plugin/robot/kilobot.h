#ifndef __SIM_KILOBOT_H__
#define __SIM_KILOBOT_H__

#include "../../core/node.h"

namespace swarmnet_sim {

#define ROBOT_RADIUS 7
#define COMM_RADIUS 50
#define TX_PERIOD_SECOND 0.5
#define VELOCITY_PER_SECOND 1

typedef struct packet {
    unsigned char payload[9];
} packet_t;

typedef struct situated_sensing {
    int distance;
    int bearing;
} situated_sensing_t;

typedef struct physical_state {
    position2d_t pos;
    color_t color;
    float velocity;
} physical_state_t;

class Kilobot : public Node {
   public:
    void collision_wrapper();
    void message_rx_wrapper(packet_t packet, situated_sensing_t sensing);
    bool message_tx_wrapper(packet_t* packet);
    void message_tx_success_wrapper();
    void init_wrapper();

    virtual void collision();
    virtual void message_rx(packet_t packet, situated_sensing_t sensing);
    virtual bool message_tx(packet_t* packet);
    virtual void message_tx_success();
    virtual void init();

    void go_forward();
    void go_forward(float seconds);
    void change_color(color_t color);
    void turn(float angle);
    physical_state_t init_user_state();
    bool check_state_change(physical_state_t old_state);
    void add_state_change_event();

    Kilobot(void* arena, int node_id, position2d_t pos);

   private:
    physical_state_t user_state;
};

}  // namespace swarmnet_sim

#endif