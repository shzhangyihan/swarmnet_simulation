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

class Kilobot : public Node {
   public:
    virtual void collision();
    virtual void message_rx(packet_t packet, situated_sensing_t sensing);
    virtual bool message_tx(packet_t* packet);
    virtual void message_tx_success();
    virtual void init();
    void start();
    void go_forward();
    void go_forward(float seconds);
    void change_color(color_t color);
    void turn(float angle);

    Kilobot(void* arena, int node_id, position2d_t pos);
};

}  // namespace swarmnet_sim

#endif