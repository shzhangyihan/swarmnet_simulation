cmake_minimum_required(VERSION 3.10)

project(swarm_net_sim)
set (CMAKE_CXX_STANDARD 11)
set (CMAKE_CXX_FLAGS "-fPIC")
# add_compile_options(-D_GLIBCXX_USE_CXX11_ABI=0)

add_definitions(-DCMAKE_ROOT_DIR="${CMAKE_SOURCE_DIR}")
add_definitions(-DBUILD_DIR="${CMAKE_SOURCE_DIR}/build/")

# add_definitions(-DSIM_NAMESPACE=robo_sim)

add_subdirectory(src)
add_subdirectory(external)
