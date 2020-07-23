include_directories(${CMAKE_SOURCE_DIR}/external)

add_executable(simulation main.cpp simulation.cpp config.cpp)
target_link_libraries(simulation JsonCpp util)

