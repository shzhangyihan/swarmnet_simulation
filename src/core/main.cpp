#include <csignal>
#include <iostream>

#include "swarmnet_sim.h"

swarmnet_sim::Arena *arena_ptr;

void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum
              << ") received. Halting simulation ..." << std::endl;
    arena_ptr->stop();
    std::cout << "Simulation ended!" << std::endl;
    exit(signum);
}

int main(int argc, char *argv[]) {
    if (argc != 2) {
        std::cerr << "Err: No config file provided; exit;" << std::endl;
        exit(1);
    }

    swarmnet_sim::Sim_config conf = swarmnet_sim::parse_config(argv[1]);
    swarmnet_sim::Arena arena(conf);
    arena_ptr = &arena;
    signal(SIGINT, signalHandler);

    arena.run();
}