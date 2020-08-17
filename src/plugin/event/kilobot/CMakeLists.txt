add_library(kilo_event rx_event.cpp tx_event.cpp update_state_event.cpp)
target_link_libraries(kilo_event event arena)