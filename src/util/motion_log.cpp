#include "motion_log.h"

namespace swarmnet_sim {

void Motion_log::flush() {
    for (int i = 0; i < buffer_size; i++) {
        this->log_file << log_buff[i];
    }
    buffer_index = 0;
}

void Motion_log::log(const std::string& content) {
    log_buff[buffer_index] = content;
    buffer_index++;
    if (buffer_index == buffer_size) {
        flush();
    }
}

Motion_log::Motion_log(int buffer_size, std::string log_file_name) {
    if (buffer_size > MAX_LOG_BUFF_SIZE) {
        this->buffer_size = MAX_LOG_BUFF_SIZE;
    } else {
        this->buffer_size = buffer_size;
    }

    this->log_file.open(log_file_name);
    this->buffer_index = 0;
}

}  // namespace swarmnet_sim