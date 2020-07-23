#ifndef __LIB_LOADER_H__
#define __LIB_LOADER_H__

#include <dlfcn.h>

namespace swarmnet_sim {
void* get_dl_handle(const char* dl_string);
}  // namespace swarmnet_sim

#endif