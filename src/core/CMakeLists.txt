include_directories(${CMAKE_SOURCE_DIR}/external)

# add_executable(simulation main.cpp event.cpp event_queue.cpp)
# target_link_libraries(simulation JsonCpp util)

add_library(arena arena.cpp)
add_library(config config.cpp)
add_library(event_queue event_queue.cpp)
add_library(event event.cpp)
add_library(node node.cpp)
add_library(physics_2d physics_2d.cpp)
add_executable(simulation main.cpp simulation.cpp)

target_link_libraries(simulation JsonCpp util arena config event_queue event node physics_2d dl)
