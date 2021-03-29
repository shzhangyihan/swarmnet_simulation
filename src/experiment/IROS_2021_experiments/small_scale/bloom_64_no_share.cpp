#include <string.h>

#include <algorithm>
#include <functional>
#include <iostream>

#include "../../../plugin/robot/kilobot.h"
#include "math.h"

#define ID_SIZE 8

#define LOG_ID()                                                            \
    std::cout << get_global_time() << "|" << node_id << ": " << id << " - " \
              << id_size << " - " << 0 << "\n";

#define RAND_FN() rand_r(&seed)
#define TX_LOG_MAX 200

namespace swarmnet_sim {

#define MSG_BYTES 30
#define BYTE_SIZE 8
#define ID_SIZE_MAX_LEN 1
#define ID_MAX_LEN 2
#define ID_SRC_SIZE_OFFSET 0
#define ID_SRC_OFFSET (ID_SRC_SIZE_OFFSET + ID_SIZE_MAX_LEN)
#define PACKET_LENGTH (ID_SRC_OFFSET + ID_MAX_LEN)
// #define RND_CHECK_OFFSET (ID_SRC_OFFSET + ID_MAX_LEN)

#define BLOOM_FILTER_SIZE 64
typedef struct {
    bool filter[BLOOM_FILTER_SIZE];
    float timer;
} Bloom_filter_t;

enum ID_field { src, rnd_c };

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    // packet_t msg;
    int id_size;  // in terms of bits
    int id;
    Bloom_filter_t my_bloom_filter;
    uint64_t tx_total;
    int tx_log_counter;
    std::hash<int> hash_func;
    unsigned int seed;

   public:
    void stop() {
        std::cout << node_id << " stop " << id << " bloom ";
        for (int i = 0; i < BLOOM_FILTER_SIZE; i++) {
            std::cout << my_bloom_filter.filter[i] << " ";
        }
        std::cout << std::endl;
    }

    void collision() { turn(RAND_FN() % 360 - 180); }

    int get_hash(int val) { return hash_func(val) % BLOOM_FILTER_SIZE; }

    void add_my_bloom_filter(int src_id) {
        int hash_val = get_hash(src_id);
        if (my_bloom_filter.filter[hash_val] == false) {
            my_bloom_filter.filter[hash_val] = true;
            my_bloom_filter.timer = get_local_time();
        }
    }

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        // std::cout << "rx - " << node_id << " " << PACKET_LENGTH << "\n";

        int src_id = read_id(packet, src);
        int src_id_size = packet.payload[ID_SRC_SIZE_OFFSET];
        add_my_bloom_filter(src_id);

        if (id_size != 0 && src_id == id && src_id_size == id_size) {
            id_collided();
        }

        if (id_size < src_id_size) {
            id_size = src_id_size;
            LOG_ID();
        }
    }

    bool message_tx(packet_t* packet) {
        set_id(packet, id, src);
        packet->payload[ID_SRC_SIZE_OFFSET] = id_size % 256;
        return true;
    }

    void message_tx_success() {
        if (tx_log_counter < TX_LOG_MAX) {
            tx_log_counter++;
            tx_total += PACKET_LENGTH;

        } else {
            std::cout << "tx - " << node_id << " " << tx_total << "\n";
            tx_log_counter = 0;
        }
    }

    void id_collided() {
        // if (diminish_function(id_size)) {
        //     // with this prob, increase id_size
        //     id_size = id_size + 1;
        // }
        id = this->new_sample_id(id_size);
        LOG_ID();
    }

    bool diminish_function(int cur_id_size) {
        double cutoff = 1 / pow(2, cur_id_size);
        double roll_dice = ((double)rand() / (RAND_MAX));
        if (roll_dice <= cutoff)
            return true;
        else
            return false;
    }

    int new_sample_id(int id_size) {
        int new_id = 0;
        int available_id_size = (uint64_t)pow(2, id_size);
        bool bloom_full = true;
        for (int i = 0; i < BLOOM_FILTER_SIZE; i++) {
            if (!my_bloom_filter.filter[i]) {
                bloom_full = false;
                break;
            }
        }
        if (bloom_full) {
            new_id = (rand()) % available_id_size;
        } else {
            while (true) {
                // reserve 0
                new_id = (rand()) % available_id_size;
                int hash_val = get_hash(new_id);
                if (my_bloom_filter.filter[hash_val]) {
                    continue;
                } else {
                    break;
                }
            }
        }
        return new_id;
    }

    void set_id(packet_t* in_msg, int in_id, ID_field field) {
        int remain = in_id;
        int index = 0;
        int offset;

        if (field == src) offset = ID_SRC_OFFSET;
        // if (field == rnd_c) offset = RND_CHECK_OFFSET;
        // if (field == dst) offset = ID_DST_OFFSET;

        while (remain != 0) {
            in_msg->payload[offset + index] = remain % 256;
            remain = remain / 256;
            index = index + 1;
        }
    }

    int read_id(packet_t in_msg, ID_field field) {
        int out_id = 0;
        int index = 0;
        int offset;
        int out_id_size;
        int out_id_byte_size;

        if (field == src) {
            offset = ID_SRC_OFFSET;
            out_id_size = in_msg.payload[ID_SRC_SIZE_OFFSET];
        }
        // if (field == rnd_c) {
        //     offset = RND_CHECK_OFFSET;
        //     out_id_size = ID_MAX_LEN * BYTE_SIZE;
        // }
        // if (field == dst) {
        //     offset = ID_DST_OFFSET;
        //     out_id_size = in_msg.payload[ID_DST_SIZE_OFFSET];
        // }

        if (out_id_size == 0) {
            // no id assigned, treat as tmp id with full length
            out_id_size = ID_MAX_LEN * BYTE_SIZE;
        }
        out_id_byte_size = std::ceil((float)out_id_size / BYTE_SIZE);

        for (int i = out_id_byte_size - 1; i >= 0; i--) {
            out_id = out_id * 256 + (uint64_t)in_msg.payload[offset + i];
        }

        return out_id;
    }

    void init() {
        // std::cout << "init " << node_id << " at " << pos.x << ", " << pos.y
        //           << "\n";
        // if (node_id == 0) {
        //     // seed
        seed = node_id;
        id_size = ID_SIZE;
        id = this->new_sample_id(id_size);
        for (int i = 0; i < BLOOM_FILTER_SIZE; i++) {
            my_bloom_filter.filter[i] = false;
        }
        int mem_per_bloom = sizeof(float) + BLOOM_FILTER_SIZE / BYTE_SIZE;
        std::cout << "mem - " << get_global_time() << " " << node_id << " "
                  << sizeof(int) * 2 + mem_per_bloom << "\n";
        // } else {
        //     id_size = 0;
        //     id = 0;
        // }
        color_t c;
        c.blue = 0;
        c.red = 255;
        c.green = 0;
        change_color(c);
        turn(RAND_FN() % 360 - 180);
        go_forward();
        LOG_ID();
        tx_log_counter = 0;
        tx_total = 0;
    }
};

// Make sure to include this line, with the argument the same as the class name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
