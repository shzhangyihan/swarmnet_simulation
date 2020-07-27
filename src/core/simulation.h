#ifndef __SIMULATION_H__
#define __SIMULATION_H__

#include <JsonCpp/json/json.h>

#include "../util/lib_loader.h"
#include "arena.h"
#include "config.h"

namespace swarmnet_sim {
void start_simulation(char* config_file);
}  // namespace swarmnet_sim

#endif
