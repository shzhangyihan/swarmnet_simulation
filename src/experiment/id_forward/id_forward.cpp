#include <iostream>

#include "../../plugin/robot/kilobot.h"
#include "math.h"

namespace swarmnet_sim {

#define MSG_BYTES 30
#define BYTE_SIZE 8
#define ID_SIZE_MAX_LEN 1
#define ID_MAX_LEN 4
#define ID_SRC_SIZE_OFFSET 0
#define ID_SRC_OFFSET (ID_SRC_SIZE_OFFSET + ID_SIZE_MAX_LEN)
#define RND_CHECK_OFFSET (ID_SRC_OFFSET + ID_MAX_LEN)

#define COLLIDE_PKT_TTL_OFFSET (RND_CHECK_OFFSET + ID_MAX_LEN)
#define TTL 70
#define TX_TIMEOUT_MAX 2000

#define MAX_TX_BUF_SIZE 5
#define MAX_CACHE_SIZE 10

enum ID_field { src, rnd_c };

typedef struct {
    packet_t cache[MAX_CACHE_SIZE];
    int size;
    int index;
} forward_cache_t;

class Default_program : public Kilobot {
    // Inherit the base constructures, make sure to include.
    using Kilobot::Kilobot;
    packet_t msg;
    uint64_t id_size;  // in terms of bits
    uint64_t id;
    uint64_t collide_checker;
    uint64_t tmp_id;
    int tx_buf_size;
    int tx_buf_index;
    packet_t tx_buf[MAX_TX_BUF_SIZE];
    forward_cache_t f_cache;
    packet_t prev_tx;
    bool id_updated;
    bool id_rx[3000];
    bool LED_toggle;
    int tx_timeout;
    int tx_count;
    bool increased_id_size;

   public:
    void clear_msg() {
        // memset(msg.payload, 0, MSG_BYTES);
        msg = (const packet_t){0};
    }

    void clear_tx_buf() {
        tx_buf_size = 0;
        tx_buf_index = 0;
        for (int i = 0; i < MAX_TX_BUF_SIZE; i++) {
            tx_buf[i] = (const packet_t){0};
        }
    }

    uint64_t read_id(packet_t in_msg, ID_field field) {
        uint64_t out_id = 0;
        uint64_t index = 0;
        uint64_t offset;
        uint64_t out_id_size;
        uint64_t out_id_byte_size;

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

    void set_id(packet_t* in_msg, uint64_t in_id, ID_field field) {
        uint64_t remain = in_id;
        uint64_t index = 0;
        uint64_t offset;

        if (field == src) offset = ID_SRC_OFFSET;
        if (field == rnd_c) offset = RND_CHECK_OFFSET;
        // if (field == dst) offset = ID_DST_OFFSET;

        while (remain != 0) {
            in_msg->payload[offset + index] = remain % 256;
            remain = remain / 256;
            index = index + 1;
        }
    }

    uint64_t new_sample_id(uint64_t id_size) {
        uint64_t new_id = 0;
        id_updated = true;
        while (new_id == 0) {
            // reserve 0
            new_id = (rand()) % (uint64_t)pow(2, id_size);
        }
        collide_checker = (rand()) % (int)pow(2, 32);

        std::cout << get_global_time() << "|" << node_id << ": " << new_id
                  << " - " << id_size << " - " << collide_checker << std::endl;

        clear_msg();
        set_id(&msg, new_id, src);
        set_id(&msg, collide_checker, rnd_c);
        msg.payload[ID_SRC_SIZE_OFFSET] = id_size % 256;
        // tx_timeout = rand() % TX_TIMEOUT_MAX;

        return new_id;
    }

    bool diminish_function(uint64_t cur_id_size) {
        // dummy implementation, always increase id_size
        // return true;
        double cutoff = 1 / pow(2, cur_id_size);
        double roll_dice = ((double)rand() / (RAND_MAX));
        if (roll_dice <= cutoff)
            return true;
        else
            return false;
    }

    void remove_and_reorder_cache(int index) {
        for (int i = f_cache.index; i < MAX_CACHE_SIZE - 1; i++) {
            f_cache.cache[i] = f_cache.cache[i + 1];
        }
        f_cache.cache[MAX_CACHE_SIZE - 1] = (const packet_t){0};
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
        if (index == -1) {
        } else {
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
        // std::cout << robot_id << " check" << std::endl;

        for (int i = 0; i < f_cache.size; i++) {
            if (pkt_cmp(pkt, f_cache.cache[i])) {
                return i;
            }
        }
        return -1;
    }

    void reset_tx_timer() {
        this->tx_timeout = (rand() % TX_TIMEOUT_MAX);
        this->tx_count = 0;
    }

    bool pkt_cmp(packet_t pkt_1, packet_t pkt_2) {
        uint64_t src_id_1 = read_id(pkt_1, src);
        uint64_t src_id_size_1 = pkt_1.payload[ID_SRC_SIZE_OFFSET];
        uint64_t in_checker_1 = read_id(pkt_1, rnd_c);

        uint64_t src_id_2 = read_id(pkt_2, src);
        uint64_t src_id_size_2 = pkt_2.payload[ID_SRC_SIZE_OFFSET];
        uint64_t in_checker_2 = read_id(pkt_2, rnd_c);

        if (src_id_1 == src_id_2 && src_id_size_1 == src_id_size_2 &&
            in_checker_1 == in_checker_2)
            return true;
        else
            return false;
    }

    void remove_and_reorder_tx_buf(int index) {
        for (int i = index; i < MAX_TX_BUF_SIZE - 1; i++) {
            tx_buf[i] = tx_buf[i + 1];
        }
        tx_buf[MAX_TX_BUF_SIZE - 1] = (const packet_t){0};
        tx_buf_size = tx_buf_size - 1;
        tx_buf_index = tx_buf_size;
    }

    void collision() { turn(rand() % 360 - 180); }

    void add_to_forward(packet_t pkt) {
        int ttl = pkt.payload[COLLIDE_PKT_TTL_OFFSET];
        if (ttl > 0) {
            if (check_cache(pkt) == -1) {
                // not in cache
                pkt.payload[COLLIDE_PKT_TTL_OFFSET] = ttl - 1;
                add_to_tx_buf(pkt);
            }
        }
    }

    void add_to_tx_buf(packet_t pkt) {
        // std::cout << robot_id << "add to tx buf" << std::endl;

        int ttl = pkt.payload[COLLIDE_PKT_TTL_OFFSET];

        for (int i = 0; i < tx_buf_size; i++) {
            int cur_ttl = tx_buf[i].payload[COLLIDE_PKT_TTL_OFFSET];
            if (pkt_cmp(pkt, tx_buf[i])) {
                if (cur_ttl < ttl) {
                    tx_buf[i] = pkt;
                }
                return;
            }
        }

        // normal insert
        tx_buf[tx_buf_index] = pkt;
        tx_buf_index++;
        tx_buf_size++;
        if (tx_buf_size > MAX_TX_BUF_SIZE) tx_buf_size = MAX_TX_BUF_SIZE;
        if (tx_buf_index >= MAX_TX_BUF_SIZE)
            tx_buf_index = tx_buf_index % MAX_TX_BUF_SIZE;
    }

    void id_collided() {
        // bool increase_size = false;
        if (diminish_function(id_size)) {
            // with this prob, increase id_size
            id_size = id_size + 1;
            increased_id_size = true;
        }
        id = this->new_sample_id(id_size);
        // if (increase_size) {
        //     // need to tell everyone asap
        //     tx_timeout = 0;
        // }
    }

    void message_rx(packet_t packet, situated_sensing_t sensing) {
        uint64_t src_id = read_id(packet, src);
        uint64_t src_id_size = packet.payload[ID_SRC_SIZE_OFFSET];
        uint64_t in_checker = read_id(packet, rnd_c);

        if (src_id == 0 && src_id_size == 0) {
            // std::cout << "empty" << std::endl;
            return;
        }

        add_to_forward(packet);

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
            id = this->new_sample_id(id_size);
        }
    }

    bool message_tx(packet_t* packet) {
        packet_t out_pkt = (const packet_t){0};
        if (tx_buf_size > 0) {
            int index = rand() % (int)tx_buf_size;
            out_pkt = tx_buf[index];
            remove_and_reorder_tx_buf(index);
            reset_tx_timer();
        } else if (tx_count >= tx_timeout || increased_id_size) {
            // start a flooding

            // std::cout << "SEND " << robot_id << " at "
            //           << ((Arena*)arena)->get_sim_tick() << std::endl;
            clear_msg();
            increased_id_size = false;
            if (id_size > 0) {
                set_id(&msg, id, src);
                set_id(&msg, collide_checker, rnd_c);
                msg.payload[ID_SRC_SIZE_OFFSET] = id_size % 256;
                msg.payload[COLLIDE_PKT_TTL_OFFSET] = TTL;
            } else {
                reset_tx_timer();
                return false;
            }
            // new assigned ids have higher priority
            // this->tx_timeout = TX_TIMEOUT_MAX * 10 + (rand() %
            // TX_TIMEOUT_MAX) * 10; this->tx_count = 0;
            reset_tx_timer();
            *packet = msg;
            return true;
        }
        tx_count++;
        *packet = out_pkt;
        return true;
    }

    void message_tx_success() {}

    void init() {
        reset_tx_timer();
        increased_id_size = false;
        this->color = {.red = 0, .green = 0, .blue = 0};
        id_updated = false;
        clear_tx_buf();
        clear_msg();
        if (node_id == 0) {
            // seed
            id_size = 1;
            id = this->new_sample_id(id_size);
        } else {
            id_size = 0;
            id = 0;
            tmp_id = this->new_sample_id(ID_MAX_LEN * BYTE_SIZE);
        }

        color_t c;
        c.blue = 0;
        c.red = 255;
        c.green = 0;
        change_color(c);
        turn(rand() % 360 - 180);
        go_forward();
    }
};

// Make sure to include this line, with the argument the same as the class name
REGISTER_PROGRAM(Default_program);

}  // namespace swarmnet_sim
