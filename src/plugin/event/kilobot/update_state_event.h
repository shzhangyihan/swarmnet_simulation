#ifndef __SIM_KILO_UPDATE_STATE_EVENT_H__
#define __SIM_KILO_UPDATE_STATE_EVENT_H__

#include "../../../core/arena.h"
#include "../../../core/event.h"

namespace swarmnet_sim {

// enum Attributes { Position, Color, Velocity, ATTRIBUTES_MAX };

class Update_state_event : public Event {
   public:
    void exec();
    void update_position(position2d_t pos);
    void update_color(color_t color);
    void update_velocity(float velocity);
    Update_state_event(void* arena, int exec_tick, int to_id);

   private:
    // bool updated[ATTRIBUTES_MAX];
    position2d_t pos;
    color_t color;
    float velocity;
};

}  // namespace swarmnet_sim

#endif