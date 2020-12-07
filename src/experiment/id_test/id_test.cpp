#include <iostream>

#include "../../plugin/robot/kilobot.h"
#include "math.h"

#define LOG_ID()                                                            \
    std::cout << get_global_time() << "|" << node_id << ": " << id << " - " \
              << id_size << " - " << collide_checker << std::endl;

namespace swarmnet_sim {

#define MAX_TX_BUF_SIZE 5

#define MSG_BYTES 30
#define BYTE_SIZE 8
#define ID_SIZE_MAX_LEN 1
#define ID_MAX_LEN 4
#define ID_SRC_SIZE_OFFSET 0
#define ID_SRC_OFFSET (ID_SRC_SIZE_OFFSET + ID_SIZE_MAX_LEN)
#define RND_CHECK_OFFSET (ID_SRC_OFFSET + ID_MAX_LEN)

enum ID_field { src, rnd_c };

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    // packet_t msg;
    int id_size;  // in terms of bits
    int id;
    int collide_checker;
    int tx_buf_size;
    int tx_buf_index;
    packet_t tx_buf[MAX_TX_BUF_SIZE];
    float last_collide_time;

   public:
    void collision() {
        turn(rand() % 360 - 180);
        // if (last_collide_time != 0) {
        //     std::cout << "collide diff "
        //               << get_global_time() - last_collide_time << std::endl
        //               << std::flush;
        // }
        // last_collide_time = get_global_time();
    }

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        int src_id = read_id(packet, src);
        int src_id_size = packet.payload[ID_SRC_SIZE_OFFSET];
        int in_checker = read_id(packet, rnd_c);

        if (src_id == 0 && src_id_size == 0) {
            // std::cout << "empty" << std::endl;
            return;
        }

        // std::cout << ((Arena*)arena)->get_sim_tick() << " - " << robot_id <<
        // ": id "
        //           << id << "(" << collide_checker << ")"
        //           << " src_id: " << src_id << "(" << in_checker << ")";
        // std::cout << robot_id << ": " << id << "|" << id_size << " ~ " <<
        // src_id
        //           << "|" << src_id_size << std::endl;
        if (id_size != 0 && src_id == id && src_id_size == id_size) {
            if (in_checker != collide_checker) {
                // std::cout << " collide";
                id_collided();
            } else {
                // from self, do nothing and return to prevent change self id
                return;
            }
        }
        // std::cout << std::endl;

        if (id_size < src_id_size) {
            id_size = src_id_size;
            LOG_ID();
            // id = this->new_sample_id(id_size);
        }
    }

    bool message_tx(packet_t* packet) {
        if (id_size > 0) {
            set_id(packet, id, src);
            set_id(packet, collide_checker, rnd_c);
            packet->payload[ID_SRC_SIZE_OFFSET] = id_size % 256;
            // std::cout << "OUT " << robot_id << " "
            //           << ((Arena*)arena)->get_sim_tick() << " (" << id << ",
            //           "
            //           << id_size << ")" << std::endl;
            return true;
        } else {
            return false;
        }
    }

    void message_tx_success() {}

    void id_collided() {
        if (diminish_function(id_size)) {
            // with this prob, increase id_size
            id_size = id_size + 1;
        }
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
        while (new_id == 0) {
            // reserve 0
            new_id = (rand()) % (uint64_t)pow(2, id_size);
        }
        collide_checker = (rand()) % (int)pow(2, 32);

        // std::cout << get_global_time() << "|" << node_id << ": " << new_id
        //           << " - " << id_size << " - " << collide_checker <<
        //           std::endl;

        // clear_msg();
        // set_id(&msg, new_id, src);
        // set_id(&msg, collide_checker, rnd_c);
        // msg.payload[ID_SRC_SIZE_OFFSET] = id_size % 256;

        return new_id;
    }

    void set_id(packet_t* in_msg, int in_id, ID_field field) {
        int remain = in_id;
        int index = 0;
        int offset;

        if (field == src) offset = ID_SRC_OFFSET;
        if (field == rnd_c) offset = RND_CHECK_OFFSET;
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
        if (field == rnd_c) {
            offset = RND_CHECK_OFFSET;
            out_id_size = ID_MAX_LEN * BYTE_SIZE;
        }
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
        //           << std::endl;
        // if (node_id == 0) {
        // seed
        id_size = 1;
        id = this->new_sample_id(id_size);
        // } else {
        //     id_size = 0;
        //     id = 0;
        // }
        color_t c;
        c.blue = 0;
        c.red = 255;
        c.green = 0;
        change_color(c);
        turn(rand() % 360 - 180);
        go_forward();
        last_collide_time = 0;
    }
};

// Make sure to include this line, with the argument the same as the class name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
