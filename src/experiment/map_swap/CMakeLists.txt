add_library(map_swap_placement SHARED map_swap_placement.cpp)

add_library(map_swap SHARED map_swap.cpp)
target_link_libraries(map_swap kilobot)
