add_library(kilobot kilobot.cpp)
target_link_libraries(kilobot node kilo_comm_event event_queue physics_2d)
