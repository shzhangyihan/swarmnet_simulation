#include "kilobot.h"

// #include <dlfcn.h>
#include <math.h>

#include <iostream>

#include "../../core/arena.h"
#include "../../util/lib_loader.h"
#include "../event/kilobot/loop_event.h"
#include "../event/kilobot/rx_event.h"
#include "../event/kilobot/tx_event.h"
#include "../event/kilobot/update_state_event.h"

namespace swarmnet_sim {

void Kilobot::collision_wrapper() {
    update_physical_state();
    this->collision();
    if (if_physical_state_changed()) {
        this->add_state_change_event();
    }
}

void Kilobot::message_rx_wrapper(packet_t packet, situated_sensing_t sensing) {
    update_physical_state();
    this->message_rx(packet, sensing);
    if (if_physical_state_changed()) {
        this->add_state_change_event();
    }
}

bool Kilobot::message_tx_wrapper(packet_t* packet) {
    update_physical_state();
    bool ret = this->message_tx(packet);
    if (if_physical_state_changed()) {
        this->add_state_change_event();
    }
    return ret;
}

void Kilobot::message_tx_success_wrapper() {
    update_physical_state();
    this->message_tx_success();
    if (if_physical_state_changed()) {
        this->add_state_change_event();
    }
}

void Kilobot::init_wrapper() {
    update_physical_state();
    this->init();
    if (if_physical_state_changed()) {
        this->add_state_change_event();
    }
    Arena* arena_ptr = (Arena*)arena;
    double tx_delay =
        (double)std::rand() / (double)RAND_MAX * MAX_RANDOM_DELAY_SECOND;
    TX_start_event* tx_start_event = new TX_start_event(
        arena_ptr, arena_ptr->get_sim_time() + tx_delay, node_id);
    this->add_event(tx_start_event);
    double loop_start_delay =
        (double)std::rand() / RAND_MAX * LOOP_PERIOD_SECOND;
    Loop_event* loop_event =
        new Loop_event(arena_ptr, loop_start_delay, node_id);
    this->add_event(loop_event);
}

void Kilobot::loop_wrapper() {
    update_physical_state();
    this->loop();
    if (if_physical_state_changed()) {
        this->add_state_change_event();
    }
    if (this->with_control_loop) {
        Arena* arena_ptr = (Arena*)arena;
        double next_loop_time =
            this->local_time_to_global_time(LOOP_PERIOD_SECOND);
        Loop_event* loop_event = new Loop_event(
            arena_ptr, arena_ptr->get_sim_time() + next_loop_time, node_id);
        arena_ptr->add_event(loop_event);
    }
}

void Kilobot::update_physical_state() {
    this->physical_state.pos = this->pos;
    this->physical_state.color = this->color;
    this->physical_state.velocity = this->velocity;
    this->physical_state.changed = false;
    this->old_physical_state = this->physical_state;
}

bool Kilobot::if_physical_state_changed() {
    if (!(this->physical_state.pos == this->old_physical_state.pos))
        return true;
    if (!(this->physical_state.color == this->old_physical_state.color))
        return true;
    if (this->physical_state.velocity != this->old_physical_state.velocity)
        return true;
    return false;
}

void Kilobot::add_state_change_event() {
    Arena* arena_ptr = (Arena*)this->arena;
    Update_state_event* event =
        new Update_state_event(arena_ptr, arena_ptr->get_sim_time(), node_id);
    event->update_velocity(this->physical_state.velocity);
    event->update_position(this->physical_state.pos);
    event->update_color(this->physical_state.color);
    this->add_event(event);
}

void Kilobot::stop() {
    // this->physical_state.changed = true;
    this->physical_state.velocity = 0;
}

void Kilobot::go_forward() {
    // this->physical_state.changed = true;
    this->physical_state.velocity = VELOCITY_PER_SECOND;
}

void Kilobot::go_forward(double seconds) {}

void Kilobot::turn(double angle) {
    // this->physical_state.changed = true;
    this->physical_state.pos.theta = fmod(this->pos.theta + angle + 360, 360);
}

void Kilobot::change_color(color_t color) {
    // this->physical_state.changed = true;
    this->physical_state.color = color;
}

double Kilobot::get_global_time() { return ((Arena*)arena)->get_sim_time(); }

double Kilobot::get_local_time() {
    double global_time = get_global_time();
    double local_time =
        this->local_clock_offset + global_time * (1 + this->local_clock_skew);
    return local_time;
}

double Kilobot::local_time_to_global_time(double local_time) {
    return local_time / (1 + this->local_clock_skew);
}

Kilobot::Kilobot(void* arena, int node_id, position2d_t pos)
    : Node(arena, node_id, pos) {
    this->radius = ROBOT_RADIUS;
    this->local_clock_offset =
        (double)std::rand() / (double)RAND_MAX * MAX_CLOCK_OFFSET_SECOND;
    this->local_clock_skew = (double)std::rand() / (double)RAND_MAX *
                                 (MAX_CLOCK_SKEW - MIN_CLOCK_SKEW) +
                             MIN_CLOCK_SKEW;
}

void Kilobot::collision() {}

void Kilobot::message_rx(packet_t packet, situated_sensing_t sensing) {}

bool Kilobot::message_tx(packet_t* packet) { return false; }

void Kilobot::message_tx_success() {}

void Kilobot::init() {}

void Kilobot::loop() { this->with_control_loop = false; }

}  // namespace swarmnet_sim