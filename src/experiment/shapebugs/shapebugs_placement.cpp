#include <math.h>

#include <iostream>
#include <vector>

#include "../../core/node.h"

namespace swarmnet_sim {

// #define ROBOT_SPACING 16

extern "C" {
std::vector<position2d_t>* robot_placement(int arena_max_x, int arena_max_y,
                                           int num_nodes) {
    // Heap allocation is used for the vector.
    // The caller will free the memory.
    std::vector<position2d_t>* pos_vector = new std::vector<position2d_t>;

    int maxsize = ceil(sqrt(num_nodes));
    int robot_spacing_x = arena_max_x / (maxsize + 1);
    int robot_spacing_y = arena_max_y / (maxsize + 1);

    int counter = 0;
    int xpos = robot_spacing_x;
    int ypos = robot_spacing_y;

    for (int i = 0; i < num_nodes; i++) {
        position2d_t pos;
        pos.x = xpos;
        pos.y = ypos;
        pos.theta = 0;
        xpos += robot_spacing_x;
        counter++;
        if (counter == maxsize) {
            counter = 0;
            xpos = robot_spacing_x;
            ypos += robot_spacing_y;
        }
        // std::cout << pos.x << ", " << pos.y << std::endl;
        pos_vector->push_back(pos);
    }

    return pos_vector;
}
}
}  // namespace swarmnet_sim