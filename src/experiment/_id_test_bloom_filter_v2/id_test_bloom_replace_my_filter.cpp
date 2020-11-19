#include <string.h>

#include <functional>
#include <iostream>

#include "../../plugin/robot/kilobot.h"
#include "math.h"

namespace swarmnet_sim {

#define BLOOM_FILTER_SIZE 32
#define BLOOM_FILTER_BUFF_SIZE 4

#define BYTE_SIZE 8
#define ID_SIZE_MAX_LEN 1
#define ID_MAX_LEN 4
#define ID_SRC_SIZE_OFFSET 0
#define ID_SRC_OFFSET (ID_SRC_SIZE_OFFSET + ID_SIZE_MAX_LEN)
#define BLOOM_FILTER_OFFSET (ID_SRC_OFFSET + ID_MAX_LEN)
#define BLOOM_FILTER_MAX_SIZE_BYTE 4

#define BLOOM_FILTER_TIME_OUT 600

// enum ID_field { src, rnd_c };

typedef struct {
    float timer;
    bool filter[BLOOM_FILTER_SIZE];
} Bloom_filter_t;

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    int id_size;  // in terms of bits
    int id;
    Bloom_filter_t bloom_filter_buff[BLOOM_FILTER_BUFF_SIZE];
    int buff_size, buff_index;
    Bloom_filter_t my_bloom_filter;
    // Bloom_filter_t check_filter;
    int filter_true_count;
    std::hash<int> hash_func;

   public:
    void clear_bloom_filter() {
        for (int i = 0; i < BLOOM_FILTER_SIZE; i++) {
            my_bloom_filter.filter[i] = false;
        }
        filter_true_count = 0;
        my_bloom_filter.timer = get_local_time();
    }

    int get_hash(int val) { return hash_func(val) % BLOOM_FILTER_SIZE; }

    // void add_id_bloom_filter(int val) {
    //     int hash_val = get_hash(val);
    //     id_bloom_filter[hash_val] = true;
    // }

    // void add_checker_bloom_filter(int val) {
    //     int hash_val = get_hash(val);
    //     checker_bloom_filter[hash_val] = true;
    // }

    void collision() { turn(rand() % 360 - 180); }

    // bool check_bloom_filter(unsigned char* filter, int val) {
    //     int offset = val / BYTE_SIZE;
    //     int index = val % BYTE_SIZE;
    //     unsigned char filter_char = filter[offset];
    //     int filter_value = filter_char >> (BYTE_SIZE - 1 - index) & 0x01;
    //     if (filter_value == 1)
    //         return true;
    //     else
    //         return false;
    // }

    void check_filter_time_out() {
        float local_time = get_local_time();
        if (local_time - my_bloom_filter.timer > BLOOM_FILTER_TIME_OUT) {
            // std::cout << node_id << " clear filter" << std::endl <<
            // std::flush;
            clear_bloom_filter();
        }

        for (int i = 0; i < buff_size; i++) {
            if (local_time - bloom_filter_buff[i].timer >
                BLOOM_FILTER_TIME_OUT) {
                for (int j = i + 1; j < buff_size; j++) {
                    bloom_filter_buff[i] = bloom_filter_buff[j];
                }
                buff_size -= 1;
                i -= 1;
            }
        }
    }

    void parse_and_add_bloom_filter(packet_t packet, int src_id) {
        // std::cout << node_id << " start add blom" << std::endl << std::flush;
        int hash_val = get_hash(src_id);
        // std::cout << node_id << " add " << src_id << " as " << hash_val
        //           << std::endl
        //           << std::flush;
        if (my_bloom_filter.filter[hash_val] == false) {
            // std::cout << node_id << " add " << src_id << "(" << hash_val
            //           << ") to filter" << std::endl
            //           << std::flush;
            filter_true_count++;
            my_bloom_filter.filter[hash_val] = true;
            my_bloom_filter.timer = get_local_time();
        }

        Bloom_filter_t new_filter;
        // memcpy(new_filter.filter, packet.payload + BLOOM_FILTER_OFFSET,
        //        BLOOM_FILTER_MAX_SIZE_BYTE);
        unsigned char* in_filter_start = packet.payload + BLOOM_FILTER_OFFSET;
        for (int i = 0; i < BLOOM_FILTER_SIZE; i++) {
            int offset = i / BYTE_SIZE;
            int index = i % BYTE_SIZE;
            unsigned char filter_char = in_filter_start[offset];
            if (filter_char >> (BYTE_SIZE - 1 - index) & 0x01 == 1)
                new_filter.filter[i] = true;
            else
                new_filter.filter[i] = false;
        }
        new_filter.timer = get_local_time();
        if (buff_size < BLOOM_FILTER_BUFF_SIZE) {
            bloom_filter_buff[buff_size] = new_filter;
            buff_size++;
        } else {
            int replace_index = rand() % BLOOM_FILTER_BUFF_SIZE;
            bloom_filter_buff[replace_index] = new_filter;
        }
        // std::cout << node_id << " end add blom" << std::endl << std::flush;
    }

    void update_my_bloom_filter() {
        int filter_counter[BLOOM_FILTER_SIZE];
        // std::cout << node_id << " update bloom filter from ";

        // for (int i = 0; i < BLOOM_FILTER_SIZE; i++) {
        //     if (my_bloom_filter.filter[i])
        //         std::cout << "1";
        //     else
        //         std::cout << "0";
        // }
        // std::cout << std::endl << std::flush;
        // for (int i = 0; i < buff_size; i++) {
        //     std::cout << i << ": ";
        //     for (int j = 0; j < BLOOM_FILTER_SIZE; j++) {
        //         if (bloom_filter_buff[i].filter[j])
        //             std::cout << "1";
        //         else
        //             std::cout << "0";
        //     }
        //     std::cout << std::endl << std::flush;
        // }

        for (int j = 0; j < BLOOM_FILTER_SIZE; j++) {
            filter_counter[j] = 0;
            for (int i = 0; i < buff_size; i++) {
                if (bloom_filter_buff[i].filter[j]) filter_counter[j]++;
            }
            if (my_bloom_filter.filter[j]) filter_counter[j]++;
        }

        // std::cout << "filter_counter ";
        // for (int j = 0; j < BLOOM_FILTER_SIZE; j++) {
        //     std::cout << filter_counter[j] << " ";
        // }
        // std::cout << std::endl << std::flush;
        int old_filter_true_count = filter_true_count;
        filter_true_count = 0;
        for (int i = 0; i < BLOOM_FILTER_SIZE; i++) {
            int random_num = rand() % (buff_size * BLOOM_FILTER_SIZE + 1 + 1);
            if (random_num < filter_counter[i]) {
                my_bloom_filter.filter[i] = true;
                filter_true_count++;
            } else {
                my_bloom_filter.filter[i] = false;
            }
        }

        if (filter_true_count != old_filter_true_count) {
            std::cout << get_global_time() << " | " << node_id
                      << " bloom filter size " << filter_true_count << std::endl
                      << std::flush;
        }
        // std::cout << node_id << " true count from " << old_filter_true_count
        // << " to " << filter

        // std::cout << node_id << " update bloom filter to ";

        // for (int i = 0; i < BLOOM_FILTER_SIZE; i++) {
        //     if (my_bloom_filter.filter[i])
        //         std::cout << "1";
        //     else
        //         std::cout << "0";
        // }
        // std::cout << std::endl << std::flush;

        my_bloom_filter.timer = get_local_time();
    }

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        int src_id = read_id(packet);
        // std::cout << "src " << src_id << std::endl << std::flush;

        int src_id_size = packet.payload[ID_SRC_SIZE_OFFSET];
        // std::cout << node_id << " rx " << std::endl;

        check_filter_time_out();

        if (src_id_size == 0) {
            // std::cout << "empty" << std::endl;
            return;
        }

        if (id_size != 0 && src_id == id && src_id_size == id_size) {
            id_collided();
        }

        parse_and_add_bloom_filter(packet, src_id);

        if (id_size < src_id_size) {
            id_size = src_id_size;
            id = this->new_sample_id(id_size);
        }

        // std::cout << node_id << " rx done" << std::endl << std::flush;
    }

    bool message_tx(packet_t* packet) {
        // std::cout << node_id << "tx !" << std::endl;
        if (id_size > 0) {
            set_id(packet, id);

            packet->payload[ID_SRC_SIZE_OFFSET] = id_size % 256;
            // std::cout << node_id << " " << id << " tx with " <<
            // read_id(*packet)
            //           << std::endl
            //           << std::flush;
            update_my_bloom_filter();
            unsigned char filter_byte = 0;
            for (int i = 0; i < BLOOM_FILTER_SIZE; i++) {
                int offset = i / BYTE_SIZE;
                int index = i % BYTE_SIZE;
                filter_byte = filter_byte << 1;
                if (my_bloom_filter.filter[i]) filter_byte += 1;
                if (index == BYTE_SIZE - 1) {
                    packet->payload[BLOOM_FILTER_OFFSET + offset] = filter_byte;
                    filter_byte = 0;
                }
            }
            return true;
        } else {
            return false;
        }
    }

    void message_tx_success() {
        // std::cout << node_id << "txs" << std::endl;
    }

    void id_collided() {
        if (diminish_function(id_size)) {
            // with this prob, increase id_size
            id_size = id_size + 1;
        }
        id = this->new_sample_id(id_size);
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
        update_my_bloom_filter();
        // std::cout << "new sample " << filter_true_count << " " << id_size
        //           << std::endl
        //           << std::flush;
        while (true) {
            new_id = (rand()) % (int)pow(2, id_size);
            int hash_val = get_hash(new_id);
            if (filter_true_count < (int)pow(2, id_size) &&
                filter_true_count < BLOOM_FILTER_SIZE &&
                my_bloom_filter.filter[hash_val] == true) {
                continue;
            } else {
                break;
            }
        }
        // std::cout << node_id << " bloom filter size " << filter_true_count
        //           << std::endl
        //           << std::flush;
        std::cout << get_global_time() << "|" << node_id << ": " << new_id
                  << " - " << id_size << " - " << 0 << std::endl
                  << std::flush;

        // clear_msg();
        // set_id(&msg, new_id, src);
        // set_id(&msg, collide_checker, rnd_c);
        // msg.payload[ID_SRC_SIZE_OFFSET] = id_size % 256;

        return new_id;
    }

    void set_id(packet_t* in_msg, int in_id) {
        int remain = in_id;
        int index = 0;
        int offset;

        offset = ID_SRC_OFFSET;

        while (remain != 0) {
            in_msg->payload[offset + index] = remain % 256;
            remain = remain / 256;
            index = index + 1;
        }
    }

    int read_id(packet_t in_msg) {
        int out_id = 0;
        int index = 0;
        int offset;
        int out_id_size;
        int out_id_byte_size;

        offset = ID_SRC_OFFSET;
        out_id_size = in_msg.payload[ID_SRC_SIZE_OFFSET];

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
        std::cout << "init " << node_id << " at " << pos.x << ", " << pos.y
                  << std::endl;
        clear_bloom_filter();
        buff_size = 0;
        buff_index = 0;
        filter_true_count = 0;

        for (int i = 0; i < BLOOM_FILTER_BUFF_SIZE; i++) {
            for (int j = 0; j < BLOOM_FILTER_SIZE; j++) {
                bloom_filter_buff[i].filter[j] = false;
            }
            bloom_filter_buff[i].timer = get_local_time();
        }

        if (node_id == 0) {
            // seed
            id_size = 1;
            id = this->new_sample_id(id_size);
        } else {
            id_size = 0;
            id = 0;
        }
        color_t c;
        c.blue = 0;
        c.red = 255;
        c.green = 0;
        change_color(c);
        turn(rand() % 360 - 180);
        // go_forward();
        buff_index = 0;
        buff_size = 0;
    }
};  // namespace swarmnet_sim

// Make sure to include this line, with the argument the same as the class name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
