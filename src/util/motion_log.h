#ifndef __SIM_MOTION_LOG_H__
#define __SIM_MOTION_LOG_H__

#include <fstream>
#include <string>

#include "../core/config.h"

namespace swarmnet_sim {

#define MAX_LOG_BUFF_SIZE 50

class Motion_log {
   public:
    void flush();
    void log(const std::string& content);
    Motion_log(int buffer_size, std::string log_file_name);
    ~Motion_log();

   private:
    int buffer_size;
    int buffer_index;
    std::ofstream log_file;
    std::string log_buff[MAX_LOG_BUFF_SIZE];
};

}  // namespace swarmnet_sim

#endif