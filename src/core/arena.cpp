#include "arena.h"

namespace swarmnet_sim {

Sim_config Arena::get_config() const { return this->conf; }

int Arena::get_current_tick() const { return current_tick; }

void Arena::run() {
    // start the sim
}

Arena::Arena(Sim_config conf) { this->conf = conf; }

}  // namespace swarmnet_sim