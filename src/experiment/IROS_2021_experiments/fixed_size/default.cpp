#include <iostream>

#include "../../../plugin/robot/kilobot.h"
#include "math.h"

#define ID_SIZE 10

#define LOG_ID()                                                            \
    std::cout << get_global_time() << "|" << node_id << ": " << id << " - " \
              << id_size << " - " << 0 << "\n";
// #define LOG_ID()

namespace swarmnet_sim {

#define MSG_BYTES 30
#define BYTE_SIZE 8
#define ID_SIZE_MAX_LEN 1
#define ID_MAX_LEN 2
#define ID_SRC_SIZE_OFFSET 0
#define ID_SRC_OFFSET (ID_SRC_SIZE_OFFSET + ID_SIZE_MAX_LEN)
#define RND_CHECK_OFFSET (ID_SRC_OFFSET + ID_MAX_LEN)
#define PACKET_LENGTH (ID_SRC_OFFSET + ID_MAX_LEN)

enum ID_field { src, rnd_c };

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    int id_size;  // in terms of bits
    int id;
    uint64_t tx_total;
    int tx_log_counter;

   public:
    void collision() {
        float new_theta = rand() % 360 - 180;
        turn(new_theta);
    }

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        // std::cout << "rx - " << node_id << " " << PACKET_LENGTH << "\n";
        int src_id = read_id(packet, src);
        int src_id_size = packet.payload[ID_SRC_SIZE_OFFSET];

        if (src_id == 0 && src_id_size == 0) {
            return;
        }
        if (id_size != 0 && src_id == id && src_id_size == id_size) {
            id_collided();
        }

        if (id_size < src_id_size) {
            id_size = src_id_size;
            LOG_ID();
        }
    }

    bool message_tx(packet_t* packet) {
        if (id_size > 0) {
            set_id(packet, id, src);
            packet->payload[ID_SRC_SIZE_OFFSET] = id_size % 256;
            return true;
        } else {
            return false;
        }
    }

    void message_tx_success() {
        if (tx_log_counter < 15) {
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
        return (rand()) % (uint64_t)pow(2, id_size);
    }

    void set_id(packet_t* in_msg, int in_id, ID_field field) {
        int remain = in_id;
        int index = 0;
        int offset;

        if (field == src) offset = ID_SRC_OFFSET;
        if (field == rnd_c) offset = RND_CHECK_OFFSET;

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
        if (field == rnd_c) {
            offset = RND_CHECK_OFFSET;
            out_id_size = ID_MAX_LEN * BYTE_SIZE;
        }
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
        id_size = ID_SIZE;
        id = this->new_sample_id(id_size);
        color_t c;
        c.blue = 0;
        c.red = 255;
        c.green = 0;
        change_color(c);
        turn(rand() % 360 - 180);
        go_forward();
        std::cout << "mem - " << get_global_time() << " " << node_id << " "
                  << sizeof(int) * 2 << "\n";
        tx_total = 0;
        tx_log_counter = 0;
        LOG_ID();
    }
};

// Make sure to include this line, with the argument the same as the class name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
