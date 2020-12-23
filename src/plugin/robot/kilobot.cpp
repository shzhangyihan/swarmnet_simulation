#include "kilobot.h"

// #include <dlfcn.h>
#include <math.h>

#include <iostream>

#include "../../core/arena.h"
#include "../../util/lib_loader.h"
#include "../event/kilobot/rx_event.h"
#include "../event/kilobot/tx_event.h"
#include "../event/kilobot/update_state_event.h"

namespace swarmnet_sim {

void Kilobot::collision_wrapper() {
    physical_state_t old_state = this->init_user_state();
    if (old_state.velocity == 0) {
        std::cout << node_id << " not moving" << std::endl << std::flush;
    }
    this->collision();
    if (this->check_state_change(old_state)) {
        // std::cout << "user state update" << std::endl << std::flush;
        this->add_state_change_event();
    }
}

void Kilobot::message_rx_wrapper(packet_t packet, situated_sensing_t sensing) {
    physical_state_t old_state = this->init_user_state();
    this->message_rx(packet, sensing);
    if (this->check_state_change(old_state)) {
        this->add_state_change_event();
    }
}

bool Kilobot::message_tx_wrapper(packet_t* packet) {
    physical_state_t old_state = this->init_user_state();
    bool ret = this->message_tx(packet);
    if (this->check_state_change(old_state)) {
        this->add_state_change_event();
    }
    return ret;
}

void Kilobot::message_tx_success_wrapper() {
    physical_state_t old_state = this->init_user_state();
    this->message_tx_success();
    if (this->check_state_change(old_state)) {
        this->add_state_change_event();
    }
}

void Kilobot::init_wrapper() {
    physical_state_t old_state = this->init_user_state();
    this->init();
    this->add_state_change_event();
    Arena* arena_ptr = (Arena*)arena;
    double tx_delay =
        (double)std::rand() / (double)RAND_MAX * MAX_RANDOM_DELAY_SECOND;
    TX_start_event* tx_start_event = new TX_start_event(
        arena_ptr, arena_ptr->get_sim_time() + tx_delay, node_id);
    this->add_event(tx_start_event);
}
physical_state_t Kilobot::init_user_state() {
    physical_state_t state;
    state.pos = this->pos;
    state.color = this->color;
    state.velocity = this->velocity;
    this->user_state = state;
    return state;
}

bool Kilobot::check_state_change(physical_state_t old_state) {
    physical_state_t cur_state = this->user_state;
    if (old_state.pos == cur_state.pos && old_state.color == cur_state.color &&
        old_state.velocity == cur_state.velocity)
        return false;
    else
        return true;
}

void Kilobot::add_state_change_event() {
    physical_state_t cur_state = this->user_state;
    Arena* arena_ptr = (Arena*)this->arena;
    Update_state_event* event =
        new Update_state_event(arena_ptr, arena_ptr->get_sim_time(), node_id);
    event->update_velocity(cur_state.velocity);
    event->update_position(cur_state.pos);
    event->update_color(cur_state.color);
    this->add_event(event);
}

void Kilobot::go_forward() {
    // Arena* arena_ptr = (Arena*)this->arena;
    // Update_state_event* event = new Update_state_event(
    //     arena_ptr, arena_ptr->get_current_tick(), node_id);
    // event->update_velocity(VELOCITY_PER_SECOND);
    // this->add_event(event);
    this->user_state.velocity = VELOCITY_PER_SECOND;
}

void Kilobot::go_forward(double seconds) {}

void Kilobot::turn(double angle) {
    // Arena* arena_ptr = (Arena*)this->arena;
    // Update_state_event* event = new Update_state_event(
    //     arena_ptr, arena_ptr->get_current_tick(), node_id);
    // position2d_t new_pos;
    // new_pos.x = this->pos.x;
    // new_pos.y = this->pos.y;
    // double new_theta = fmod(this->pos.theta + angle + 360, 360);
    // new_pos.theta = new_theta;
    // event->update_position(new_pos);
    // this->add_event(event);
    this->user_state.pos.theta = fmod(this->pos.theta + angle + 360, 360);
}

void Kilobot::change_color(color_t color) {
    // Arena* arena_ptr = (Arena*)this->arena;
    // Update_state_event* event = new Update_state_event(
    //     arena_ptr, arena_ptr->get_current_tick(), node_id);
    // event->update_color(color);
    // this->add_event(event);
    this->user_state.color = color;
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
    // std::cout << "offset " << local_clock_offset << " skew " <<
    // local_clock_skew
    //           << std::endl
    //           << std::flush;
}

void Kilobot::collision() {}

void Kilobot::message_rx(packet_t packet, situated_sensing_t sensing) {}

bool Kilobot::message_tx(packet_t* packet) { return false; }

void Kilobot::message_tx_success() {}

void Kilobot::init() {}

}  // namespace swarmnet_sim