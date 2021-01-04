#ifndef __SIM_PHYSICS_H__
#define __SIM_PHYSICS_H__

#include <stdint.h>

namespace swarmnet_sim {

typedef struct position {
    double x;
    double y;
    double theta;
} position2d_t;

typedef struct color {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} color_t;

position2d_t calculate_future_pos(position2d_t start, double v, double seconds);
double calculate_dist(position2d_t pos1, position2d_t pos2);
bool if_collision(position2d_t pos_1, position2d_t pos_2, double radius);
bool if_out_of_bound(position2d_t pos, double radius, double x_max,
                     double y_max);

bool operator==(const position2d_t& lhs, const position2d_t& rhs);
bool operator==(const color_t& lhs, const color_t& rhs);

class Physics_engine {
   public:
    virtual double check_collision(double future_time) = 0;
    virtual void init() = 0;
    Physics_engine(void* arena) { this->arena = arena; };
    Physics_engine(){};

   protected:
    void* arena;
};

}  // namespace swarmnet_sim

#endif