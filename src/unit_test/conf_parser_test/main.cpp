#include <iostream>

#include "../../util/config_parser.h"

int main(int argc, char *argv[]) {
    swarmnet_sim::Sim_config conf =
        swarmnet_sim::parse_config("./config/experiment/config_30.json");
}