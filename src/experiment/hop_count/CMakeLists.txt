add_library(hop_count_placement SHARED hop_count_placement.cpp)
add_library(hop_count SHARED hop_count.cpp)

add_library(hop_count_placement_1500 SHARED hop_count_placement_1500.cpp)
add_library(hop_count_1500 SHARED hop_count_1500.cpp)

target_link_libraries(hop_count kilobot)
target_link_libraries(hop_count_1500 kilobot)
