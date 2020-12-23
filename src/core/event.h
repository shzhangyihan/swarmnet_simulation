#ifndef __SIM_EVENT_H__
#define __SIM_EVENT_H__

namespace swarmnet_sim {

class Event {
   public:
    double get_exec_time() const;
    int get_from_id() const;
    int get_to_id() const;
    void log_node(int id);
    virtual void exec();
    Event(void* arena, double exec_time, int from_id, int to_id);
    Event();

   protected:
    void* arena;
    double exec_time;
    int from_id;
    int to_id;
};

struct CmpEventPtrs {
    bool operator()(const Event* lhs, const Event* rhs) const;
};

}  // namespace swarmnet_sim
#endif