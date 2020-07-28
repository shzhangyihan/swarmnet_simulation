#include <math.h>

#include <iostream>
#include <vector>

#include "../../core/node.h"

namespace swarmnet_sim {

#define ROBOT_SPACING 16
extern "C" {
std::vector<position2d_t>* robot_placement(int arena_max_x, int arena_max_y,
                                           int num_nodes) {
    std::vector<position2d_t>* pos_vector = new std::vector<position2d_t>;
    int xpos = 20;
    int ypos = 100;
    int maxsize = int(sqrt(num_nodes));
    int counter = 0;

    std::cout << "place " << num_nodes << std::endl;
    for (int i = 0; i < num_nodes; i++) {
        position2d_t pos;
        pos.x = xpos;
        pos.y = ypos;
        pos.theta = 0;
        xpos += ROBOT_SPACING;
        counter++;
        if (counter == maxsize) {
            counter = 0;
            xpos = 20;
            ypos += ROBOT_SPACING;
        }
        pos_vector->push_back(pos);
    }

    return pos_vector;
}
}
}  // namespace swarmnet_sim