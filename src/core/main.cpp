#include <iostream>

#include "simulation.h"

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Err: No config file provided; exit;" << std::endl;
        exit(1);
    }

    swarmnet_sim::start_simulation(argv[1]);
}