#include "lib_loader.h"

#include <limits.h>
#include <stdlib.h>

#include <iostream>

namespace swarmnet_sim {
void* get_dl_handle(const char* dl_string) {
    char dl_path[PATH_MAX];
    char* ret = realpath(dl_string, dl_path);
    std::cout << "Try to load dynamic library: \"" << dl_path << "\""
              << std::endl;
    void* hndl = dlopen(dl_path, RTLD_NOW);
    if (hndl == NULL) {
        std::cerr << dlerror() << std::endl;
        exit(-1);
    }
    return hndl;
}

}  // namespace swarmnet_sim
