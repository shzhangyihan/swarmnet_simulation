#include <iostream>

#include "swarmnet_sim.h"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Err: No config file provided; exit;" << std::endl;
        exit(1);
    }

    swarmnet_sim::Sim_config conf = swarmnet_sim::parse_config(argv[1]);
    swarmnet_sim::Arena arena(conf);
    arena.run();
}