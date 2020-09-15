#ifndef __SIM_KILO_FIXED_MEDIUM_H__
#define __SIM_KILO_FIXED_MEDIUM_H__

#include <tuple>
#include <vector>

#include "../../../core/medium.h"
#include "../../robot/kilobot.h"

namespace swarmnet_sim {

#define SPEED_BYTE_PER_SECOND 0.001
#define TX_PERIOD_NOISE_RANGE_SECOND 0.02
#define MAX_SUCCESS_RATE 1
#define DROPPING_SHARPNESS 2 /* smaller, steeper */
// #define MAX_DIST_SUCCESS_RATE 1.0
// #define MIN_DIST_SUCCESS_RATE 0.05
// #define FALLOFF_SHARPNESS 0.3

typedef struct {
    packet_t packet;
    situated_sensing_t sensing;
    bool corrupted;
} rx_buffer_t;

class Kilo_fixed_medium : public Medium {
   public:
    void start_tx(int tx_node_id);
    void end_tx(int tx_node_id, bool success);
    void start_rx(int rx_node_id, packet_t rx_packet,
                  situated_sensing_t sensing);
    void end_rx(int rx_node_id);
    void init();
    Kilo_fixed_medium(void* arena);

   protected:
    void* arena;
    std::vector<int> rx_counter_vector;
    std::vector<rx_buffer_t> rx_buffer;
    std::vector<std::vector<int>> topology;
};

extern "C" {
Medium* medium_builder(void* arena);
}

}  // namespace swarmnet_sim

#endif
