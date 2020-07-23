#include "arena.h"

#include <cstdlib>
#include <iostream>

#include "experiments/placement.h"

namespace SIM_NAMESPACE {
void state_log::update_sim_tick(std::int32_t sim_tick) {
    this->sim_tick = sim_tick;
}

std::int32_t state_log::get_sim_tick() { return this->sim_tick; }

void state_log::update_log(std::string log) { this->log = log; }

std::string state_log::get_log() { return this->log; }

// state_log::state_log(std::int32_t sim_tick, std::string log) {
//     this->sim_tick = sim_tick;
//     this->log = log;
// }

std::int32_t Arena::get_sim_tick() { return this->sim_tick; }

std::int32_t Arena::get_size_x() { return this->size_x; }

std::int32_t Arena::get_size_y() { return this->size_y; }

std::int32_t Arena::get_num_robots() { return this->num_robots; }

std::int32_t Arena::get_event_version() { return this->event_version; }

std::int32_t Arena::get_ticks_per_second() { return this->ticks_per_second; }

std::vector<Robot_base *> Arena::get_robot_vector() {
    return this->robot_vector;
}

void Arena::increment_event_version() {
    this->event_version = this->event_version + 1;
}

void Arena::register_event(std::int32_t sim_time, std::int32_t node_id,
                           std::int32_t version,
                           std::function<void(void)> event_fn) {
    this->event_queue->push(Event(sim_time, node_id, version, event_fn));
}

void Arena::update_simulation_state(std::int32_t time) {
    for (int i = 0; i < this->num_robots; i++) {
        position_t rob_pos = this->robot_vector[i]->get_position();
        position_t end_pos =
            calculate_end_pos(rob_pos, this->robot_vector[i]->get_theta(),
                              this->robot_vector[i]->get_velocity(), time);
        this->robot_vector[i]->set_position(end_pos);
        // std::cout << "Set robot " << i << " from (" << rob_pos.x << ", " <<
        // rob_pos.y << ") to (" << end_pos.x << ", " << end_pos.y << ") with t
        // = " << time << std::endl;
    }
}

void Arena::flush_state_log() {
    // std::cout << "flush" << std::endl;
    for (int i = 0; i < log_index; i++) {
        this->log_file << log_buffer[i].get_log();
    }
    this->log_file << std::flush;
    log_index = 0;
}

void Arena::log_simulation_state() {
    // std::cout << "log" << std::endl;

    std::string log;
    for (int i = 0; i < this->num_robots; i++) {
        position_t rob_pos = this->robot_vector[i]->get_position();
        log = log + std::to_string(this->sim_tick) + " " + std::to_string(i) +
              " " + std::to_string((int)rob_pos.x) + " " +
              std::to_string((int)rob_pos.y) + " ";
        log = log + std::to_string(this->robot_vector[i]->get_theta()) + " " +
              std::to_string(this->robot_vector[i]->get_velocity()) + " ";
        color_t rob_color = this->robot_vector[i]->get_color();
        log = log + std::to_string((int)rob_color.red) + " " +
              std::to_string((int)rob_color.green) + " " +
              std::to_string((int)rob_color.blue) + "\n";
    }

    if (log_index == 0) {
        this->log_buffer[log_index].update_sim_tick(this->sim_tick);
        this->log_buffer[log_index].update_log(log);
        log_index = log_index + 1;
    } else {
        if (this->log_buffer[log_index - 1].get_sim_tick() == this->sim_tick) {
            // log at same time, cover the old one
            this->log_buffer[log_index - 1].update_log(log);
        } else {
            if (log_index == LOG_BUFFER_SIZE) {
                // buffer full, flush
                flush_state_log();
            }
            this->log_buffer[log_index].update_sim_tick(this->sim_tick);
            this->log_buffer[log_index].update_log(log);
            log_index = log_index + 1;
        }
    }

    // for(int i = 0; i < this->num_robots; i++) {
    //     position_t rob_pos = this->robot_vector[i]->get_position();
    //     this->log_file << this->sim_tick << " " << i << " " << (int)rob_pos.x
    //     << " " << (int)rob_pos.y << " "; this->log_file <<
    //     this->robot_vector[i]->get_theta() << " " <<
    //     this->robot_vector[i]->get_velocity() << " "; color_t rob_color =
    //     this->robot_vector[i]->get_color(); this->log_file <<
    //     (int)rob_color.red << " " << (int)rob_color.green << " " <<
    //     (int)rob_color.blue << std::endl;
    // }
    // this->log_file << std::flush;
}

void Arena::add_dummy_future_events() {
    std::function<void(void)> event_fn =
        std::bind(&Arena::add_dummy_future_events, this);
    this->register_event(this->sim_tick + MAX_PREDICT_TICKS, -1,
                         this->event_version, event_fn);
}

void Arena::robot_init_recv_packet_event(std::int32_t robot_id,
                                         std::int32_t distance, packet_t packet,
                                         situated_sensing_t sensing) {
    robot_vector[robot_id]->initiate_recv(packet, sensing, distance);
}

void Arena::robot_send_packet_event(std::int32_t sender_robot_id) {
    ///// Here is where robots check their neighbors vector as opposed to
    /// checking around themselves.
    // packet_t packet = robot_vector[sender_robot_id]->message_tx();
    packet_t packet = robot_vector[sender_robot_id]->get_packet();
    if (!packet.valid) {
        robot_vector[sender_robot_id]->sent();
        return;
    }
    position_t src = robot_vector[sender_robot_id]->get_position();

    std::vector<int> neighbors_list =
        robot_vector[sender_robot_id]->get_neighbors();
    std::vector<int> neighbor_distances =
        robot_vector[sender_robot_id]->get_distances();
    std::int32_t neighbors_size = neighbors_list.size();

    for (int i = 0; i < neighbors_size; i++) {
        std::int32_t neighbor_id = neighbors_list[i];
        std::int32_t neighbor_dist = neighbor_distances[i];
        position_t dst = robot_vector[i]->get_position();
        double dist = sqrt((src.x - dst.x) * (src.x - dst.x) +
                           (src.y - dst.y) * (src.y - dst.y));

        situated_sensing_t sensing;
        sensing.distance = dist;
        std::uint32_t theta_diff =
            atan2(src.y - dst.y, src.x - dst.x) * 180 / M_PI;
        sensing.bearing = theta_diff - robot_vector[i]->get_theta();

        if (sensing.bearing > 180) {
            sensing.bearing = sensing.bearing - 360;
        }

        if (sensing.bearing < -180) {
            sensing.bearing = sensing.bearing + 360;
        }
        double fdelay_factor = 0.02 * this->ticks_per_second;
        int delay_factor = (int)fdelay_factor;
        this->register_event(
            this->sim_tick + rand() % delay_factor, -1, this->event_version,
            std::bind(&Arena::robot_init_recv_packet_event, this, neighbor_id,
                      neighbor_dist, packet, sensing));
    }
    robot_vector[sender_robot_id]->sent();
    // std::cout << "HI THIS IS ROBOT " << sender_robot_id << " AT TICK " <<
    // this->sim_tick << std::endl;
}

void Arena::robot_end_recv_event(packet_t packet, situated_sensing_t sensing,
                                 std::int32_t robot_id, std::int32_t collided) {
    if (!collided && !(robot_vector[robot_id]->get_failing_flag()))
        robot_vector[robot_id]->recv(packet, sensing);
    else
        robot_vector[robot_id]->fail_recv();
}

void Arena::run() {
    std::int32_t wait_counter = 0;
    std::int32_t update_counter = 0;
    double fduration = this->experiment_duration * this->ticks_per_second;
    int duration = (int)fduration;
    while (true) {
        if (this->sim_tick >= duration) {
            //// Hacky visualization at end of sim here
            std::string log;
            for (int i = 0; i < this->num_robots; i++) {
                position_t rob_pos = this->robot_vector[i]->get_position();
                log = log + std::to_string(0) + " " + std::to_string(i) + " " +
                      std::to_string((int)rob_pos.x) + " " +
                      std::to_string((int)rob_pos.y) + " ";
                log = log + std::to_string(this->robot_vector[i]->get_theta()) +
                      " " +
                      std::to_string(this->robot_vector[i]->get_velocity()) +
                      " ";
                color_t rob_color = this->robot_vector[i]->get_color();
                log = log + std::to_string((int)rob_color.red) + " " +
                      std::to_string((int)rob_color.green) + " " +
                      std::to_string((int)rob_color.blue) + "\n";
            }
            for (int i = 0; i < this->num_robots; i++) {
                position_t rob_pos = this->robot_vector[i]->get_position();
                log = log + std::to_string(1) + " " + std::to_string(i) + " " +
                      std::to_string((int)rob_pos.x) + " " +
                      std::to_string((int)rob_pos.y) + " ";
                log = log + std::to_string(this->robot_vector[i]->get_theta()) +
                      " " +
                      std::to_string(this->robot_vector[i]->get_velocity()) +
                      " ";
                color_t rob_color = this->robot_vector[i]->get_color();
                log = log + std::to_string((int)rob_color.red) + " " +
                      std::to_string((int)rob_color.green) + " " +
                      std::to_string((int)rob_color.blue) + "\n";
            }
            this->log_file << log;
            break;
        }

        if (this->event_queue->empty()) {
            // waiting for too long without events
            if (wait_counter > MAX_PREDICT_TICKS) {
                add_dummy_future_events();
            }
            wait_counter++;
            continue;
        }
        wait_counter = 0;
        Event event = this->event_queue->pop();
        // check the event version to see if still valid
        if (event.get_node_id() == -1) {
            // this is an event issued by the arena
            if (event.get_version() < this->event_version) {
                continue;
            }
        } else {
            if (event.get_version() <
                robot_vector[event.get_node_id()]->get_event_version()) {
                continue;
            }
        }

        ///// Physics checking can be toggled here.
        // std::int32_t time_to_collide = check_physics(this,
        // event.get_sim_time() - this->sim_tick);
        std::int32_t time_to_collide = -1;
        if (time_to_collide == -1) {
            // no collision happened
            std::int32_t time_diff = event.get_sim_time() - this->sim_tick;
            if (!(time_diff == 0)) {
                ///// Toggling update_simulation_state and log_simulation_state
                /// can be done here/
                update_simulation_state(event.get_sim_time() - this->sim_tick);
                // log_simulation_state();
            }
            this->sim_tick = event.get_sim_time();
            event.exec();
        } else {
            // put the original event back to queue
            event_queue->push(event);

            // update the simulation to before the collision
            // update_simulation_state(time_to_collide);
            // this->sim_tick = this->sim_tick + time_to_collide;

            /* HACK: when buggy comment out */
            // in theory, the next event should always be the collision event,
            // thus can just exec it Event collide_event =
            // this->event_queue->pop();
            // update_simulation_state(collide_event.get_sim_time() -
            // this->sim_tick); this->sim_tick = collide_event.get_sim_time();
            // collide_event.exec();
            // log_simulation_state();
            /* when buggy comment out */
        }
    }
}

Arena::Arena(Event_queue *event_queue, std::int32_t num_robots,
             std::int32_t size_x, std::int32_t size_y,
             std::string log_file_name, std::int32_t rand_seed,
             std::int32_t ticks_per_second, std::int32_t experiment_duration) {
    this->event_queue = event_queue;
    this->num_robots = num_robots;
    this->size_x = size_x;
    this->size_y = size_y;
    this->sim_tick = 0;
    this->event_version = 0;
    this->log_file.open(log_file_name);
    log_file << size_x << " " << size_y << " " << num_robots << std::endl;
    this->log_index = 0;
    this->ticks_per_second = ticks_per_second;
    this->experiment_duration = experiment_duration;

    srand(rand_seed);

    // create robots
    robot_vector = robot_placement(this, num_robots);

    for (int i = 0; i < num_robots; i++) {
        robot_vector[i]->init();
    }

    for (int i = 0; i < this->num_robots; i++) {
        position_t src = robot_vector[i]->get_position();
        for (int j = 0; j < this->num_robots; j++) {
            if (i != j) {
                position_t dst = robot_vector[j]->get_position();
                double dist = sqrt((src.x - dst.x) * (src.x - dst.x) +
                                   (src.y - dst.y) * (src.y - dst.y));
                if (dist <= COMM_RADIUS) {
                    robot_vector[i]->add_neighbors(j, dist);
                }
            }
        }
    }

    run();
}
}  // namespace SIM_NAMESPACE