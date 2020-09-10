add_library(comm_test SHARED comm_test.cpp)
add_library(comm_test_placement SHARED comm_test_placement.cpp)

target_link_libraries(comm_test kilobot)
