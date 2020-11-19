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
#define TTL_OFFSET (ID_SRC_OFFSET + ID_MAX_LEN)
#define TTL_SIZE 1
#define BLOOM_FILTER_OFFSET (TTL_OFFSET + TTL_SIZE)

#define PACKET_LENGTH (BLOOM_FILTER_OFFSET + BLOOM_FILTER_SIZE / BYTE_SIZE)

#define MAX_TTL 2
// #define BLOOM_FILTER_MAX_SIZE_BYTE 6

#define BLOOM_FILTER_TIME_OUT 300
#define TX_TIMEOUT_MAX 200

// enum ID_field { src, rnd_c };

typedef struct {
    bool filter[BLOOM_FILTER_SIZE];
    float timer;
} Bloom_filter_t;

typedef struct {
    Bloom_filter_t buf[BLOOM_FILTER_BUFF_SIZE];
    int size;
    int index;
} Bloom_filter_buf_t;

#define MAX_TX_BUF_SIZE 5
#define MAX_CACHE_SIZE 10

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
    int id_size;  // in terms of bits
    int id;
    Bloom_filter_buf_t rx_bloom_buf;
    Bloom_filter_t my_bloom_filter;
    int filter_true_count;
    std::hash<int> hash_func;
    forward_cache_t f_cache;
    forward_buf_t f_buf;
    int tx_timeout;
    int tx_count;

   public:
    void clear_bloom_filter() {
        // std::cout << node_id << " cleared self bloom filter" << std::endl
        //           << std::flush;
        for (int i = 0; i < BLOOM_FILTER_SIZE; i++) {
            my_bloom_filter.filter[i] = false;
        }
        filter_true_count = 0;
        my_bloom_filter.timer = get_local_time();
    }

    void reset_tx_timer() {
        this->tx_timeout = (rand() % TX_TIMEOUT_MAX);
        this->tx_count = 0;
    }

    int get_hash(int val) { return hash_func(val) % BLOOM_FILTER_SIZE; }

    void collision() { turn(rand() % 360 - 180); }

    void check_filter_time_out() {
        float local_time = get_local_time();
        if (local_time - my_bloom_filter.timer > BLOOM_FILTER_TIME_OUT) {
            clear_bloom_filter();
        }

        for (int i = 0; i < rx_bloom_buf.size; i++) {
            if (local_time - rx_bloom_buf.buf[i].timer >
                BLOOM_FILTER_TIME_OUT) {
                for (int j = i + 1; j < rx_bloom_buf.size; j++) {
                    rx_bloom_buf.buf[i] = rx_bloom_buf.buf[j];
                }
                rx_bloom_buf.size -= 1;
                i -= 1;
            }
        }
    }

    void add_my_bloom_filter(int src_id) {
        int hash_val = get_hash(src_id);
        if (my_bloom_filter.filter[hash_val] == false) {
            my_bloom_filter.filter[hash_val] = true;
            my_bloom_filter.timer = get_local_time();
        }
    }

    void add_bloom_buffer(packet_t packet) {
        Bloom_filter_t new_filter;
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
        if (rx_bloom_buf.size < BLOOM_FILTER_BUFF_SIZE) {
            rx_bloom_buf.buf[rx_bloom_buf.size] = new_filter;
            rx_bloom_buf.size++;
        } else {
            int replace_index = rand() % BLOOM_FILTER_BUFF_SIZE;
            rx_bloom_buf.buf[replace_index] = new_filter;
        }
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
        // std::cout << robot_id << "add to cache" << std::endl;
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

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        int src_id = read_id(packet);
        int src_id_size = packet.payload[ID_SRC_SIZE_OFFSET];
        int ttl = packet.payload[TTL_OFFSET];

        check_filter_time_out();

        // with only id, no bloom filter, no need to forward
        if (ttl == MAX_TTL + 1) {
            // std::cout << "worked!!!" << std::endl;
            react_to_rx_id(src_id, src_id_size);
            return;
        }

        // last hop
        if (ttl == 0) {
            // add to filter buffer
            // std::cout << node_id << " add bloom from "
            //           << (int)packet.payload[PACKET_LENGTH] << " - " <<
            //           src_id
            //           << std::endl
            //           << std::flush;
            // check if bloom filter is empty

            add_bloom_buffer(packet);
        }
        add_to_forward(packet);

        // first hop
        if (ttl == MAX_TTL) {
            // check id collision
            react_to_rx_id(src_id, src_id_size);
        }
    }

    void react_to_rx_id(int src_id, int src_id_size) {
        if (src_id_size == 0) {
            return;
        }
        add_my_bloom_filter(src_id);
        if (id_size != 0 && src_id == id && src_id_size == id_size) {
            id_collided();
        }

        if (id_size < src_id_size) {
            id_size = src_id_size;
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

    bool message_tx(packet_t* packet) {
        if (f_buf.size > 0) {
            int index = rand() % (int)f_buf.size;
            *packet = f_buf.buf[index];
            remove_and_reorder_tx_buf(index);
            reset_tx_timer();
            return true;
        } else if (tx_count >= tx_timeout) {
            set_id(packet, id);
            packet->payload[ID_SRC_SIZE_OFFSET] = id_size % 256;
            packet->payload[TTL_OFFSET] = MAX_TTL;
            packet->payload[PACKET_LENGTH] = node_id;  // HACK for debug
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
            add_to_cache(*packet);
            reset_tx_timer();
            return true;
        } else {
            tx_count++;

            // std::cout << "send STUFF!" << std::endl;
            set_id(packet, id);
            packet->payload[ID_SRC_SIZE_OFFSET] = id_size % 256;
            packet->payload[TTL_OFFSET] = MAX_TTL + 1;
            packet->payload[PACKET_LENGTH] = node_id;  // HACK for debug

            return true;
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

    Bloom_filter_t combine_filters() {
        Bloom_filter_t combined_filter;
        filter_true_count = 0;
        for (int j = 0; j < BLOOM_FILTER_SIZE; j++) {
            combined_filter.filter[j] = false;
            for (int i = 0; i < rx_bloom_buf.size; i++) {
                if (rx_bloom_buf.buf[i].filter[j])
                    combined_filter.filter[j] = true;
            }
            if (my_bloom_filter.filter[j]) combined_filter.filter[j] = true;
            if (combined_filter.filter[j]) filter_true_count++;
        }

        return combined_filter;
    }

    int new_sample_id(int id_size) {
        int new_id = 0;
        Bloom_filter_t combined_filter = combine_filters();
        while (true) {
            new_id = (rand()) % (int)pow(2, id_size);
            if (filter_true_count == BLOOM_FILTER_SIZE ||
                filter_true_count >= (int)pow(2, id_size)) {
                // filter full
                break;
            }
            int hash_val = get_hash(new_id);
            if (combined_filter.filter[hash_val] == true) {
                // filter hit, resample
                continue;
            } else {
                break;
            }
        }
        std::cout << get_global_time() << "|" << node_id << ": " << new_id
                  << " - " << id_size << " - " << 0 << std::endl
                  << std::flush;
        std::cout << get_global_time() << "|" << node_id << " true count "
                  << filter_true_count << std::endl
                  << std::flush;
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
        clear_bloom_filter();
        filter_true_count = 0;

        for (int i = 0; i < BLOOM_FILTER_BUFF_SIZE; i++) {
            for (int j = 0; j < BLOOM_FILTER_SIZE; j++) {
                rx_bloom_buf.buf[i].filter[j] = false;
            }
            rx_bloom_buf.buf[i].timer = get_local_time();
        }
        rx_bloom_buf.index = 0;
        rx_bloom_buf.size = 0;
        clear_cache();
        f_buf.index = 0;
        f_buf.size = 0;

        id_size = 1;
        id = this->new_sample_id(id_size);
        color_t c;
        c.blue = 0;
        c.red = 255;
        c.green = 0;
        change_color(c);
        turn(rand() % 360 - 180);
        go_forward();
    }
};  // namespace swarmnet_sim

// Make sure to include this line, with the argument the same as the class name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
