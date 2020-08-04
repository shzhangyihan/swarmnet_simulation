#ifndef __SIM_CONFIG_PARSER_H__
#define __SIM_CONFIG_PARSER_H__

#include "../../external/JsonCpp/json/json.h"
#include "../core/config.h"
#include "lib_loader.h"

namespace swarmnet_sim {

Sim_config parse_config(char* config_file);

}  // namespace swarmnet_sim

#endif
