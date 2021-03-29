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

#define BYTE_SIZE 8
#define ID_SIZE_MAX_LEN 1
#define ID_MAX_LEN 2
#define ID_SRC_SIZE_OFFSET 0
#define ID_SRC_OFFSET (ID_SRC_SIZE_OFFSET + ID_SIZE_MAX_LEN)
#define RND_CHECK_OFFSET (ID_SRC_OFFSET + ID_MAX_LEN)
#define TTL_OFFSET (RND_CHECK_OFFSET + ID_MAX_LEN)
#define TTL_SIZE 1
#define PACKET_LENGTH (TTL_OFFSET + TTL_SIZE)

#define BLOOM_FILTER_SIZE 64
typedef struct {
    bool filter[BLOOM_FILTER_SIZE];
    float timer;
} Bloom_filter_t;

enum ID_field { src, rnd_c };

#define MAX_TX_BUF_SIZE 20
#define MAX_CACHE_SIZE 40
#define TX_TIMEOUT_MAX 30
#define MAX_TTL 1

typedef struct {
    packet_t cache[MAX_CACHE_SIZE];
    int size;
    int index;
} forward_cache_t;

typedef struct {
    packet_t buf[MAX_TX_BUF_SIZE];
    int size;
    int index;
} forward_buf_t;

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    // packet_t msg;
    int id_size;  // in terms of bits
    int id;
    int rnd_checker;
    Bloom_filter_t my_bloom_filter;
    uint64_t tx_total;
    int tx_log_counter;
    std::hash<int> hash_func;
    unsigned int seed;
    forward_cache_t f_cache;
    forward_buf_t f_buf;
    int tx_timeout;
    int tx_count;
    int cur_mem_size;

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

    void reset_tx_timer() {
        this->tx_timeout = (rand() % TX_TIMEOUT_MAX);
        this->tx_count = 0;
    }

    void remove_and_reorder_cache(int index) {
        for (int i = index; i < f_cache.size - 1; i++) {
            f_cache.cache[i] = f_cache.cache[i + 1];
        }
        f_cache.cache[f_cache.size - 1] = (const packet_t){0};
        f_cache.size = f_cache.size - 1;
        f_cache.index = f_cache.size;
    }

    void clear_cache() {
        f_cache.size = 0;
        f_cache.index = 0;
        for (int i = 0; i < MAX_CACHE_SIZE; i++)
            f_cache.cache[i] = (const packet_t){0};
    }

    void add_to_cache(packet_t pkt) {
        // std::cout << robot_id << "add to cache" << "\n";
        int index = check_cache(pkt);
        if (index != -1) {
            remove_and_reorder_cache(index);
        }
        // normal insert
        f_cache.cache[f_cache.index] = pkt;
        f_cache.index++;
        f_cache.size++;
        if (f_cache.size > MAX_CACHE_SIZE) f_cache.size = MAX_CACHE_SIZE;
        if (f_cache.index >= MAX_CACHE_SIZE)
            f_cache.index = f_cache.index % MAX_CACHE_SIZE;
    }

    int check_cache(packet_t pkt) {
        for (int i = 0; i < f_cache.size; i++) {
            if (pkt_cmp(pkt, f_cache.cache[i])) {
                return i;
            }
        }
        return -1;
    }

    bool pkt_cmp(packet_t pkt_1, packet_t pkt_2) {
        // compare all bytes except for the TTL
        for (int i = 0; i < PACKET_LENGTH; i++) {
            if (i == TTL_OFFSET) continue;
            if (pkt_1.payload[i] != pkt_2.payload[i]) return false;
        }

        return true;
    }

    void add_to_tx_buf(packet_t pkt) {
        int ttl = pkt.payload[TTL_OFFSET];

        for (int i = 0; i < f_buf.size; i++) {
            if (pkt_cmp(pkt, f_buf.buf[i])) {
                int cur_ttl = f_buf.buf[i].payload[TTL_OFFSET];
                if (cur_ttl < ttl) {
                    f_buf.buf[i] = pkt;
                }
                return;
            }
        }

        // normal insert
        f_buf.buf[f_buf.index] = pkt;
        f_buf.index++;
        f_buf.size++;
        if (f_buf.size > MAX_TX_BUF_SIZE) f_buf.size = MAX_TX_BUF_SIZE;
        if (f_buf.index >= MAX_TX_BUF_SIZE)
            f_buf.index = f_buf.index % MAX_TX_BUF_SIZE;
    }

    void add_to_forward(packet_t packet) {
        int ttl = packet.payload[TTL_OFFSET];
        if (ttl > 0) {
            if (check_cache(packet) == -1) {
                // not in cache
                packet.payload[TTL_OFFSET] = ttl - 1;
                add_to_tx_buf(packet);
            }
        }
    }

    void remove_and_reorder_tx_buf(int index) {
        for (int i = index; i < f_buf.size - 1; i++) {
            f_buf.buf[i] = f_buf.buf[i + 1];
        }
        f_buf.buf[MAX_TX_BUF_SIZE - 1] = (const packet_t){0};
        f_buf.size = f_buf.size - 1;
        f_buf.index = f_buf.size;
    }

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        // std::cout << "rx - " << node_id << " " << PACKET_LENGTH << "\n";

        int src_id = read_id(packet, src);
        int src_id_size = packet.payload[ID_SRC_SIZE_OFFSET];
        int in_checker = read_id(packet, rnd_c);

        if (in_checker == rnd_checker) {
            // from self, do nothing
            return;
        }

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
        if (f_buf.size > 0) {
            int index = rand() % (int)f_buf.size;
            *packet = f_buf.buf[index];
            remove_and_reorder_tx_buf(index);
            reset_tx_timer();
            // prev_tx_long = true;
            return true;
        } else if (tx_count >= tx_timeout) {
            set_id(packet, id, src);
            set_id(packet, rnd_checker, rnd_c);
            packet->payload[ID_SRC_SIZE_OFFSET] = id_size % 256;
            packet->payload[TTL_OFFSET] = MAX_TTL;
            // packet->payload[PACKET_LENGTH] = node_id;  // HACK for debug
            add_to_cache(*packet);
            reset_tx_timer();
            // prev_tx_long = true;
            return true;
        } else {
            tx_count++;

            // std::cout << "send STUFF!" << "\n";
            set_id(packet, id, src);
            set_id(packet, rnd_checker, rnd_c);
            packet->payload[ID_SRC_SIZE_OFFSET] = id_size % 256;
            packet->payload[TTL_OFFSET] = 0;
            // packet->payload[PACKET_LENGTH] = node_id;  // HACK for debug
            // prev_tx_long = false;
            return true;
        }
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
        rnd_checker = rand() % (uint64_t)pow(2, ID_MAX_LEN * BYTE_SIZE);

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
        //           << "\n";
        // if (node_id == 0) {
        //     // seed
        seed = node_id;
        id_size = ID_SIZE;
        id = this->new_sample_id(id_size);
        clear_cache();
        f_buf.index = 0;
        f_buf.size = 0;
        for (int i = 0; i < BLOOM_FILTER_SIZE; i++) {
            my_bloom_filter.filter[i] = false;
        }
        int mem_per_bloom = sizeof(float) + BLOOM_FILTER_SIZE / BYTE_SIZE;
        std::cout << "mem - " << get_global_time() << " " << node_id << " "
                  << sizeof(int) * 2 + mem_per_bloom << "\n";
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
